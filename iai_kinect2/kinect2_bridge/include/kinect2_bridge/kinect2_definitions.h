/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#ifndef __KINECT2_DEFINITIONS_H__
#define __KINECT2_DEFINITIONS_H__

#include <kinect2_registration/kinect2_console.h>

#define K2_DEFAULT_NS          "kinect2"

#define K2_TF_LINK             "_link"
#define K2_TF_RGB_OPT_FRAME    "_rgb_optical_frame"
#define K2_TF_IR_OPT_FRAME     "_ir_optical_frame"

#define K2_TOPIC_HD            "/hd"
#define K2_TOPIC_QHD           "/qhd"
#define K2_TOPIC_SD            "/sd"

#define K2_TOPIC_IMAGE_RECT    "_rect"
#define K2_TOPIC_IMAGE_COLOR   "/image_color"
#define K2_TOPIC_IMAGE_MONO    "/image_mono"
#define K2_TOPIC_IMAGE_DEPTH   "/image_depth"
#define K2_TOPIC_IMAGE_IR      "/image_ir"

#define K2_TOPIC_COMPRESSED    "/compressed"
#define K2_TOPIC_INFO          "/camera_info"

#define K2_CALIB_COLOR         "calib_color.yaml"
#define K2_CALIB_IR            "calib_ir.yaml"
#define K2_CALIB_POSE          "calib_pose.yaml"
#define K2_CALIB_DEPTH         "calib_depth.yaml"

#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"
#define K2_CALIB_DEPTH_SHIFT   "depthShift"

#endif //__KINECT2_DEFINITIONS_H__
