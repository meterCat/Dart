#ifndef __GLOBAL_STRUCT_HPP__
#define __GLOBAL_STRUCT_HPP__

#include <opencv2/opencv.hpp>
#include "Time_generated.h"


struct VisionData {             //send

    uint8_t header = 0xA5; 
    uint8_t CRC8 = 0;           // 8位循环冗余校验码，用于数据的错误检测

    float yaw_error;            // 发给电控的yaw
    uint8_t target_status;      // 0：目标丢失，1：跟随中
    float speed;

    uint16_t CRC16 = 0;       
}__attribute__((packed));       



struct CarData {                //receive

    uint8_t header = 0xA5; 
    uint8_t mode;               // 0: 不开自瞄, 不录像 1: 开自瞄且录像 2: 录像
    uint8_t CRC8; 

    uint8_t status;             // 车辆状态
    uint8_t number;             // 第number号飞镖
    uint8_t dune;               //0：完全展开 1：完全关闭 2：正在进行时
    uint16_t CRC16; 

}__attribute__((packed)); 


typedef struct {
    CarData cd;
    foxglove::Time timestamp;   //时间戳
    cv::Mat img;
} FromData;


typedef struct {
    cv::Point2d center_point;   // 引导灯中心点的坐标
    cv::Rect2d box;             // 引导灯的包围盒
    double score;               // 置信度
} LightInfo;

typedef struct {
    std::vector<LightInfo> lights; // 检测到的所有引导灯的信息
    foxglove::Time timestamp;      // 图像捕获的时间戳

    cv::Mat ori_mat;               // 原始图像数据
    CarData cd;                

} ImageInfo;

typedef struct {
    std::deque<LightInfo> vec;     // 缓存的目标信息队列
    double drop_frame;             // 丢帧率
    double found_frame;            // 目标找到的帧数
    int state;                     // 目标跟踪的状态
}TargetCache;

typedef struct {
    CarData cd;                   
    VisionData vd;                
    foxglove::Time timestamp;     
    cv::Mat img;                  
} CaptureInfo;

typedef struct {
    cv::Mat raw_image; 
} RawImageInfo;


#endif // __GLOBAL_STRUCT_HPP__