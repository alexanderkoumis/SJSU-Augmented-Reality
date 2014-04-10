//
//  Globals.h
//  SJSUViewfinder
//
//  Created by Henry Tran on 9/18/13.
//  Copyright (c) 2013 Henry Tran. All rights reserved.
//

#ifndef SJSUViewfinder_Globals_h
#define SJSUViewfinder_Globals_h

#include "opencv2/opencv.hpp"

// define this function in your main.cpp file and return your camera's intrinsic values (cx, cy, fx, fy)
cv::Matx33f getCamIntrinsic();

#endif
