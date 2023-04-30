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
#ifndef __KINECT2_REGISTRATION_H__
#define __KINECT2_REGISTRATION_H__

#include <vector>

#include <opencv2/opencv.hpp>

class DepthRegistration
{
public:
  enum Method
  {
    DEFAULT = 0,
    CPU,
    OPENCL
  };

protected:
  cv::Mat cameraMatrixRegistered, cameraMatrixDepth, rotation, translation, mapX, mapY;
  cv::Size sizeRegistered, sizeDepth;
  float zNear, zFar;

  DepthRegistration();

  virtual bool init(const int deviceId) = 0;

public:
  virtual ~DepthRegistration();

  bool init(const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
            const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
            const float zNear = 0.5f, const float zFar = 12.0f, const int deviceId = -1);

  virtual bool registerDepth(const cv::Mat &depth, cv::Mat &registered) = 0;

  static DepthRegistration *New(Method method = DEFAULT);
};

#endif //__KINECT2_REGISTRATION_H__
