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

#include "depth_registration_cpu.h"

DepthRegistrationCPU::DepthRegistrationCPU()
  : DepthRegistration()
{
}

DepthRegistrationCPU::~DepthRegistrationCPU()
{
}

bool DepthRegistrationCPU::init(const int deviceId)
{
  createLookup();

  proj(0, 0) = rotation.at<double>(0, 0);
  proj(0, 1) = rotation.at<double>(0, 1);
  proj(0, 2) = rotation.at<double>(0, 2);
  proj(0, 3) = translation.at<double>(0, 0);
  proj(1, 0) = rotation.at<double>(1, 0);
  proj(1, 1) = rotation.at<double>(1, 1);
  proj(1, 2) = rotation.at<double>(1, 2);
  proj(1, 3) = translation.at<double>(1, 0);
  proj(2, 0) = rotation.at<double>(2, 0);
  proj(2, 1) = rotation.at<double>(2, 1);
  proj(2, 2) = rotation.at<double>(2, 2);
  proj(2, 3) = translation.at<double>(2, 0);
  proj(3, 0) = 0;
  proj(3, 1) = 0;
  proj(3, 2) = 0;
  proj(3, 3) = 1;

  fx = cameraMatrixRegistered.at<double>(0, 0);
  fy = cameraMatrixRegistered.at<double>(1, 1);
  cx = cameraMatrixRegistered.at<double>(0, 2) + 0.5;
  cy = cameraMatrixRegistered.at<double>(1, 2) + 0.5;

  return true;
}

inline uint16_t DepthRegistrationCPU::interpolate(const cv::Mat &in, const float &x, const float &y) const
{
  const int xL = (int)floor(x);
  const int xH = (int)ceil(x);
  const int yL = (int)floor(y);
  const int yH = (int)ceil(y);

  if(xL < 0 || yL < 0 || xH >= in.cols || yH >= in.rows)
  {
    return 0;
  }

  const uint16_t pLT = in.at<uint16_t>(yL, xL);
  const uint16_t pRT = in.at<uint16_t>(yL, xH);
  const uint16_t pLB = in.at<uint16_t>(yH, xL);
  const uint16_t pRB = in.at<uint16_t>(yH, xH);
  int vLT = pLT > 0;
  int vRT = pRT > 0;
  int vLB = pLB > 0;
  int vRB = pRB > 0;
  int count = vLT + vRT + vLB + vRB;

  if(count < 3)
  {
    return 0;
  }

  const uint16_t avg = (pLT + pRT + pLB + pRB) / count;
  const uint16_t thres = 0.01 * avg;
  vLT = abs(pLT - avg) < thres;
  vRT = abs(pRT - avg) < thres;
  vLB = abs(pLB - avg) < thres;
  vRB = abs(pRB - avg) < thres;
  count = vLT + vRT + vLB + vRB;

  if(count < 3)
  {
    return 0;
  }

  double distXL = x - xL;
  double distXH = 1.0 - distXL;
  double distYL = y - yL;
  double distYH = 1.0 - distYL;
  distXL *= distXL;
  distXH *= distXH;
  distYL *= distYL;
  distYH *= distYH;
  const double tmp = sqrt(2.0);
  const double fLT = vLT ? tmp - sqrt(distXL + distYL) : 0;
  const double fRT = vRT ? tmp - sqrt(distXH + distYL) : 0;
  const double fLB = vLB ? tmp - sqrt(distXL + distYH) : 0;
  const double fRB = vRB ? tmp - sqrt(distXH + distYH) : 0;
  const double sum = fLT + fRT + fLB + fRB;

  return ((pLT * fLT +  pRT * fRT + pLB * fLB + pRB * fRB) / sum) + 0.5;
}

void DepthRegistrationCPU::remapDepth(const cv::Mat &depth, cv::Mat &scaled) const
{
  scaled.create(sizeRegistered, CV_16U);
  #pragma omp parallel for
  for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r)
  {
    uint16_t *itO = scaled.ptr<uint16_t>(r);
    const float *itX = mapX.ptr<float>(r);
    const float *itY = mapY.ptr<float>(r);
    for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++itO, ++itX, ++itY)
    {
      *itO = interpolate(depth, *itX, *itY);
    }
  }
}

void DepthRegistrationCPU::projectDepth(const cv::Mat &scaled, cv::Mat &registered) const
{
  registered = cv::Mat::zeros(sizeRegistered, CV_16U);

  #pragma omp parallel for
  for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r)
  {
    const uint16_t *itD = scaled.ptr<uint16_t>(r);
    const double y = lookupY.at<double>(0, r);
    const double *itX = lookupX.ptr<double>();

    for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++itD, ++itX)
    {
      const double depthValue = *itD / 1000.0;

      if(depthValue < zNear || depthValue > zFar)
      {
        continue;
      }

      Eigen::Vector4d pointD(*itX * depthValue, y * depthValue, depthValue, 1);
      Eigen::Vector4d pointP = proj * pointD;

      const double z = pointP[2];

      const double invZ = 1 / z;
      const int xP = (fx * pointP[0]) * invZ + cx;
      const int yP = (fy * pointP[1]) * invZ + cy;

      if(xP >= 0 && xP < sizeRegistered.width && yP >= 0 && yP < sizeRegistered.height)
      {
        const uint16_t z16 = z * 1000;
        uint16_t &zReg = registered.at<uint16_t>(yP, xP);
        if(zReg == 0 || z16 < zReg)
        {
          zReg = z16;
        }
      }
    }
  }
}

bool DepthRegistrationCPU::registerDepth(const cv::Mat &depth, cv::Mat &registered)
{
  cv::Mat scaled;
  remapDepth(depth, scaled);
  projectDepth(scaled, registered);
  return true;
}

void DepthRegistrationCPU::createLookup()
{
  const double fx = 1.0 / cameraMatrixRegistered.at<double>(0, 0);
  const double fy = 1.0 / cameraMatrixRegistered.at<double>(1, 1);
  const double cx = cameraMatrixRegistered.at<double>(0, 2);
  const double cy = cameraMatrixRegistered.at<double>(1, 2);
  double *it;

  lookupY = cv::Mat(1, sizeRegistered.height, CV_64F);
  it = lookupY.ptr<double>();
  for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, sizeRegistered.width, CV_64F);
  it = lookupX.ptr<double>();
  for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}
