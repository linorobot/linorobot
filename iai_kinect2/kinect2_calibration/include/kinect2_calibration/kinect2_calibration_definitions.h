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
#ifndef __KINECT2_CALIBRATION_DEFINITIONS_H__
#define __KINECT2_CALIBRATION_DEFINITIONS_H__

#define CALIB_FILE_EXT      ".png"
#define CALIB_FILE_COLOR    "_color" CALIB_FILE_EXT
#define CALIB_FILE_IR       "_ir" CALIB_FILE_EXT
#define CALIB_FILE_IR_GREY  "_grey_ir" CALIB_FILE_EXT
#define CALIB_FILE_DEPTH    "_depth" CALIB_FILE_EXT

#define CALIB_POINTS_COLOR  "_color_points.yaml"
#define CALIB_POINTS_IR     "_ir_points.yaml"

#define CALIB_SYNC          "_sync"
#define CALIB_SYNC_COLOR    CALIB_SYNC CALIB_FILE_COLOR
#define CALIB_SYNC_IR       CALIB_SYNC CALIB_FILE_IR
#define CALIB_SYNC_IR_GREY  CALIB_SYNC CALIB_FILE_IR_GREY

#endif //__KINECT2_CALIBRATION_DEFINITIONS_H__
