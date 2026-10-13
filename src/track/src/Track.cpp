#include "Track.hpp"

#include "global/Config.hpp"

#include "function/function.hpp"
Track::Track(SerialPort &sp, const char *mode) : sp_(sp), mode_(mode) {
    J_TRACK.config_["ktrack"]["path"] >> this->path_;

    this->config_.open(this->path_, cv::FileStorage::READ);

    this->last_image_timestamp_ = function::getNowTimestamp();
    this->target_.state = LOST;

    cv::namedWindow("track", cv::WINDOW_NORMAL);
    cv::moveWindow("track", 1180, 0);
    cv::resizeWindow("track", cv::Size(300, 768));
}


void Track::track(LfStack<ImageInfo> &image_info_stack) {
    auto opt = image_info_stack.pop();

    while(!opt.has_value()) {
        opt = image_info_stack.pop();
    }

    if (this->mode_ == "debug") {
        J_TRACK.updateJson();
    }

    this->updateTimeInterval(opt.value().timestamp);
    this->chooseTarget(opt.value());
}

// 更新时间间隔函数
void Track::updateTimeInterval(const foxglove::Time &image_timestamp) {
    this->image_time_interval_ = function::timestampMinus(image_timestamp, this->last_image_timestamp_) / 1e3;
    this->last_image_timestamp_ = image_timestamp;
}

// 重置函数
void Track::reset() {
    this->target_.vec.clear();
    this->target_.drop_frame = 0.;
    this->target_.found_frame = 0.;
    this->target_.state = LOST;

    this->cache_.lights.clear();

    this->init_filter_ = false;
}


// 插入目标向量函数
// 参数：向量 vec、灯光信息 light 和大小 size
void Track::insertTargetVec(
        std::deque<LightInfo> &vec,
        const LightInfo &light,
        const size_t &size) {
    if (vec.size() > size) {
        vec.pop_front();
    }
    
    vec.push_back(light);
}


// 帧限制函数
// 参数：帧数 frame、增加值 plus 和限制值 limit
auto Track::frameLimit(const double &frame, const double &plus, const double &limit) -> double {
    return frame < limit ? frame + plus : limit;
}

void Track::updateState() {     // 更新状态函数
                                // 通过查找 识别到的帧数 和 未识别的帧数 及其 置信度 确定是否识别到
    if (!this->cache_.lights.empty() && this->target_.state == LOST) {
        this->target_.drop_frame = 0.;
        this->target_.found_frame = Track::frameLimit(
                this->target_.found_frame,
                this->image_time_interval_,
                (double) J_TRACK.config_["selection"]["limit_found_frame"]);
        if (this->target_.found_frame >= (double) J_TRACK.config_["selection"]["min_found_frame"]) {
                this->target_.state = FOUND;
                double max_score = 0.;
                LightInfo max_score_light;
                for (const auto &light : this->cache_.lights) {
                    if (light.score > max_score) {
                        max_score = light.score;
                        max_score_light = light;
                    }
                }
                Track::insertTargetVec(
                        this->target_.vec,
                        max_score_light,
                        5);
        }
    } else if (this->target_.state != LOST) {
        if (!this->cache_.lights.empty()) {

            this->target_.state = FOUND;
            this->target_.drop_frame = 0.;
            this->target_.found_frame = Track::frameLimit(
                    this->target_.found_frame,
                    this->image_time_interval_,
                    (double) J_TRACK.config_["selection"]["limit_found_frame"]);

            LightInfo target_light;
            auto min_distance = DBL_MAX;
            LightInfo target_light_back = this->target_.vec.back();
            for (auto &light : this->cache_.lights) {
                double distance = cv::norm(target_light_back.center_point - light.center_point);
                if (distance < min_distance) {
                    min_distance = distance;
                    target_light = light;
                }
            }
            Track::insertTargetVec(
                    this->target_.vec,
                    target_light,
                    5);
        } else {
            this->target_.state = DROP;
            this->target_.found_frame = 0.;
            this->target_.drop_frame = Track::frameLimit(
                    this->target_.drop_frame,
                    this->image_time_interval_,
                    (double) J_TRACK.config_["selection"]["limit_drop_frame"]);
        }
    } else if (this->cache_.lights.empty()) {
        this->target_.found_frame = 0.0;
        this->target_.drop_frame = 0.0;
    }

    if (this->target_.drop_frame >= (double) J_TRACK.config_["selection"]["max_drop_frame"]) {
        std::cout << "drop frame too much, reset." << std::endl;
        this->reset();
    }
}

void Track::chooseTarget(ImageInfo &image_info) {
    this->cache_ = std::move(image_info);
    this->updateState();

    if (this->target_.state != LOST) {
        this->updateVisionData();
    } else {
        this->vd_.yaw_error = 0.;
        this->vd_.target_status = 0;
        this->sp_.send(this->vd_);
    }
}

auto Track::filter(const cv::Point2d &point, const double &dt) -> cv::Point2d {
    return {
            this->filter_x_.filter(point.x, dt),
            this->filter_y_.filter(point.y, dt)};
}

void Track::updateVisionData() {
// 图像绘制：
// 在图像上绘制目标点、目标框、置信度、偏航误差、第几发飞镖、模式、状态和舱门信息

// 显示和发送数据：
// 显示图像
// 通过串口发送视觉数据
    cv::Point2d aim;
    double smoothed_area;

    //std::cout << "--------testing--------" << std::endl;

    if (!this->init_filter_) {
        this->init_filter_ = true;
        this->filter_x_.setParameter(
                this->target_.vec.back().center_point.x,
                0.,
                (double) J_TRACK.config_["euro"]["min_cut_off"],
                (double) J_TRACK.config_["euro"]["beta"],
                (double) J_TRACK.config_["euro"]["d_cut_off"]);

        this->filter_y_.setParameter(
                this->target_.vec.back().center_point.y,
                0.,
                (double) J_TRACK.config_["euro"]["min_cut_off"],
                (double) J_TRACK.config_["euro"]["beta"],
                (double) J_TRACK.config_["euro"]["d_cut_off"]);
        aim = this->target_.vec.back().center_point;

        double initial_area = this->target_.vec.back().box.width * this->target_.vec.back().box.height;

        this->filter_area_.setParameter(
            initial_area,
            0.,
            (double) J_TRACK.config_["euro"]["min_cut_off"],
            (double) J_TRACK.config_["euro"]["beta"],
            (double) J_TRACK.config_["euro"]["d_cut_off"]);

        
        smoothed_area = initial_area;

    } else {
        aim = this->filter(this->target_.vec.back().center_point, this->image_time_interval_);
        double current_area = this->target_.vec.back().box.width * this->target_.vec.back().box.height;
        smoothed_area = this->filter_area_.filter(current_area, this->image_time_interval_);
    }

    auto &vec = this->target_.vec.back();

    this->vd_.target_status = 1;

    double aim_yaw = (double)J_TRACK.config_[std::to_string(this->cache_.cd.number)];
    if (this->cache_.cd.number < 1 && !this->cache_.cd.mode) {
        this->config_.release();
        this->config_.open(this->path_, cv::FileStorage::WRITE);
        this->config_ << "begin" << vec.center_point.x;
        this->config_.release();
        this->config_.open(this->path_, cv::FileStorage::READ);
    }
    std::string choice;
    J_TRACK.config_["choice"] >> choice;



    ////////////标记一处地点//////////
    if(jishuqi==0 || kyou<250){

    double current_speed = (25 * 59 / (smoothed_area*0.05)) * 1+10;
    speed_values_.push_back(current_speed);
    kyou++;
    if (speed_values_.size() == 50) {
        double sum = 0;
        for (double speed : speed_values_) {
            sum += speed;
        }
        double average_speed = sum / speed_values_.size();
        this->vd_.speed = average_speed;
        speed_values_.clear();
        jishuqi=1;
    }

    }
    else if(jishuqi==1&&kyou==250){
        this->vd_.target_status = 1;
    }



    this->vd_.yaw_error = -float(float(this->config_["begin"]) - vec.center_point.x -
            ((float)J_TRACK.config_["group"][choice]["0"] + (float)J_TRACK.config_["group"][choice][std::to_string(this->cache_.cd.number)]));

//    this->vd_.yaw_error = J_TRACK.config_["death"];
    auto ori = this->cache_.ori_mat.clone();
    cv::circle(ori, aim, 3, cv::Scalar(0, 0, 255), -1);
    cv::rectangle(ori, vec.box, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "score: " + std::to_string(vec.score), cv::Point(0, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "error: " + std::to_string(this->vd_.yaw_error), cv::Point(0,  100),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "number: " + std::to_string(this->cache_.cd.number), cv::Point(0, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "mode: " + std::to_string(this->cache_.cd.mode), cv::Point(0, 200),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "status: " + std::to_string(this->cache_.cd.status), cv::Point(0, 250),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "dune: " + std::to_string(this->cache_.cd.dune), cv::Point(0, 300),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(ori,
                "speed: " + std::to_string(this->vd_.speed), cv::Point(0, 350),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::imshow("track", ori);
    cv::waitKey(1);
    this->sp_.send(this->vd_);
}

