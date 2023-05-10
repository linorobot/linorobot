/**
 * Copyright 2015 University of Bremen, Institute for Artificial Intelligence
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
#ifndef __KINECT2_CONSOLE_H__
#define __KINECT2_CONSOLE_H__

#include <string>
#include <ros/console.h>

// Set this to '0' to disable the extended colored output
#define EXTENDED_OUTPUT 1

#if EXTENDED_OUTPUT

#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

const std::string getFunctionName(const std::string &name);
#define OUT_AUX(FUNC_COLOR, MSG_COLOR, STREAM, MSG) STREAM(FUNC_COLOR "[" << getFunctionName(__PRETTY_FUNCTION__) << "] " MSG_COLOR << MSG << NO_COLOR)

#define OUT_DEBUG(msg) OUT_AUX(FG_BLUE, NO_COLOR, ROS_DEBUG_STREAM, msg)
#define OUT_INFO(msg) OUT_AUX(FG_GREEN, NO_COLOR, ROS_INFO_STREAM, msg)
#define OUT_WARN(msg) OUT_AUX(FG_YELLOW, FG_YELLOW, ROS_WARN_STREAM, msg)
#define OUT_ERROR(msg) OUT_AUX(FG_RED, FG_RED, ROS_ERROR_STREAM, msg)

#else

#define NO_COLOR        ""
#define FG_BLACK        ""
#define FG_RED          ""
#define FG_GREEN        ""
#define FG_YELLOW       ""
#define FG_BLUE         ""
#define FG_MAGENTA      ""
#define FG_CYAN         ""

#define OUT_DEBUG(msg) ROS_DEBUG_STREAM(msg)
#define OUT_INFO(msg) ROS_INFO_STREAM(msg)
#define OUT_WARN(msg) ROS_WARN_STREAM(msg)
#define OUT_ERROR(msg) ROS_WARN_STREAM(msg)

#endif

#endif //__KINECT2_CONSOLE_H__
