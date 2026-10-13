#ifndef __INLINE_VARIABLE_HPP__
#define __INLINE_VARIABLE_HPP__

#include <opencv2/opencv.hpp>
#include "json/ReJson.hpp"

inline ReJson J_CAPTURE("/home/w/Vision_Code/config/json/capture.json");
inline ReJson J_DETECT("/home/w/Vision_Code/config/json/detect.json");
inline ReJson J_SENSORS("/home/w/Vision_Code/config/json/sensors.json");
inline ReJson J_TRACK("/home/w/Vision_Code/config/json/track.json");
inline ReJson J_MOUSE("/home/w/Vision_Code/config/json/mouse.json");

#endif