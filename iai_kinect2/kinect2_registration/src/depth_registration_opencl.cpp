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

#include <fstream>

#include <kinect2_registration/kinect2_console.h>

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS

#ifdef KINECT2_OPENCL_ICD_LOADER_IS_OLD
#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#include <CL/cl.h>
#ifdef CL_VERSION_1_2
#undef CL_VERSION_1_2
#endif //CL_VERSION_1_2
#endif //LIBFREENECT2_OPENCL_ICD_LOADER_IS_OLD

#include <CL/cl.hpp>

#ifndef REG_OPENCL_FILE
#define REG_OPENCL_FILE ""
#endif

#include "depth_registration_opencl.h"

//#define ENABLE_PROFILING_CL

#define CL_FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define PRINT_CL_ERROR(expr, err) OUT_ERROR(FG_BLUE "[" << CL_FILENAME << "]" FG_CYAN "(" << __LINE__ << ") " FG_YELLOW << expr << FG_RED " failed: " << err)

#define CHECK_CL_PARAM(expr) do { cl_int err = CL_SUCCESS; (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(#expr, err); return false; } } while(0)
#define CHECK_CL_RETURN(expr) do { cl_int err = (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(#expr, err); return false; } } while(0)
#define CHECK_CL_ON_FAIL(expr, on_fail) do { cl_int err = (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(#expr, err); on_fail; return false; } } while(0)

struct DepthRegistrationOpenCL::OCLData
{
  cl::Context context;
  cl::Device device;

  cl::Program program;
  cl::CommandQueue queue;

  cl::Kernel kernelSetZero;
  cl::Kernel kernelProject;
  cl::Kernel kernelCheckDepth;
  cl::Kernel kernelRemap;

  size_t sizeDepth;
  size_t sizeRegistered;
  size_t sizeIndex;
  size_t sizeImgZ;
  size_t sizeDists;
  size_t sizeSelDist;
  size_t sizeMap;

  cl::Buffer bufferDepth;
  cl::Buffer bufferScaled;
  cl::Buffer bufferRegistered;
  cl::Buffer bufferIndex;
  cl::Buffer bufferImgZ;
  cl::Buffer bufferDists;
  cl::Buffer bufferSelDist;
  cl::Buffer bufferMapX;
  cl::Buffer bufferMapY;

  cl::Buffer bufferOutput;
  unsigned char *dataOutput;

#ifdef ENABLE_PROFILING_CL
  std::vector<double> timings;
  int count;
#endif
};

DepthRegistrationOpenCL::DepthRegistrationOpenCL()
  : DepthRegistration()
{
  data = new OCLData;
}

DepthRegistrationOpenCL::~DepthRegistrationOpenCL()
{
  delete data;
}

void getDevices(const std::vector<cl::Platform> &platforms, std::vector<cl::Device> &devices)
{
  devices.clear();
  for(size_t i = 0; i < platforms.size(); ++i)
  {
    const cl::Platform &platform = platforms[i];

    std::vector<cl::Device> devs;
    if(platform.getDevices(CL_DEVICE_TYPE_ALL, &devs) != CL_SUCCESS)
    {
      continue;
    }

    devices.insert(devices.end(), devs.begin(), devs.end());
  }
}

std::string deviceString(cl::Device &dev)
{
  std::string devName, devVendor, devType;
  cl_device_type devTypeID;
  dev.getInfo(CL_DEVICE_NAME, &devName);
  dev.getInfo(CL_DEVICE_VENDOR, &devVendor);
  dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

  switch(devTypeID)
  {
  case CL_DEVICE_TYPE_CPU:
    devType = "CPU";
    break;
  case CL_DEVICE_TYPE_GPU:
    devType = "GPU";
    break;
  case CL_DEVICE_TYPE_ACCELERATOR:
    devType = "ACCELERATOR";
    break;
  default:
    devType = "CUSTOM/UNKNOWN";
  }

  return devName + " (" + devType + ")[" + devVendor + ']';
}

bool selectDevice(std::vector<cl::Device> &devices, cl::Device &device, const int deviceId = -1)
{
  if(deviceId != -1 && devices.size() > (size_t)deviceId)
  {
    device = devices[deviceId];
    return true;
  }

  bool selected = false;
  cl_device_type selectedType = 0;

  for(size_t i = 0; i < devices.size(); ++i)
  {
    cl::Device &dev = devices[i];
    cl_device_type devTypeID;
    dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

    if(!selected || (selectedType != CL_DEVICE_TYPE_GPU && devTypeID == CL_DEVICE_TYPE_GPU))
    {
      selectedType = devTypeID;
      selected = true;
      device = dev;
    }
  }
  return selected;
}

bool DepthRegistrationOpenCL::init(const int deviceId)
{
  std::string sourceCode;
  if(!readProgram(sourceCode))
  {
    return false;
  }

  std::vector<cl::Platform> platforms;
  CHECK_CL_RETURN(cl::Platform::get(&platforms));

  if(platforms.empty())
  {
    OUT_ERROR("no opencl platforms found.");
    return false;
  }

  std::vector<cl::Device> devices;
  getDevices(platforms, devices);

  OUT_INFO("devices:");
  for(size_t i = 0; i < devices.size(); ++i)
  {
    OUT_INFO("  " << i << ": " FG_CYAN << deviceString(devices[i]) << NO_COLOR);
  }

  if(!selectDevice(devices, data->device, deviceId))
  {
    OUT_ERROR("could not find any suitable device");
    return false;
  }
  OUT_INFO("selected device: " FG_YELLOW << deviceString(data->device) << NO_COLOR);

  CHECK_CL_PARAM(data->context = cl::Context(data->device, NULL, NULL, NULL, &err));

  std::string options;
  generateOptions(options);

  cl::Program::Sources source(1, std::make_pair(sourceCode.c_str(), sourceCode.length()));
  CHECK_CL_PARAM(data->program = cl::Program(data->context, source, &err));

  CHECK_CL_ON_FAIL(data->program.build(options.c_str()),
                   OUT_ERROR("failed to build program: " << err);
                   OUT_ERROR("Build Status: " << data->program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(data->device));
                   OUT_ERROR("Build Options:\t" << data->program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(data->device));
                   OUT_ERROR("Build Log:\t " << data->program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(data->device)));

#ifdef ENABLE_PROFILING_CL
  data->count = 0;
  CHECK_CL_PARAM(data->queue = cl::CommandQueue(data->context, data->device, CL_QUEUE_PROFILING_ENABLE, &err));
#else
  CHECK_CL_PARAM(data->queue = cl::CommandQueue(data->context, data->device, 0, &err));
#endif

  data->sizeDepth = sizeDepth.height * sizeDepth.width * sizeof(uint16_t);
  data->sizeRegistered = sizeRegistered.height * sizeRegistered.width * sizeof(uint16_t);
  data->sizeIndex = sizeRegistered.height * sizeRegistered.width * sizeof(cl_int4);
  data->sizeImgZ = sizeRegistered.height * sizeRegistered.width * sizeof(uint16_t);
  data->sizeDists = sizeRegistered.height * sizeRegistered.width * sizeof(cl_float4);
  data->sizeSelDist = sizeRegistered.height * sizeRegistered.width * sizeof(float);
  data->sizeMap = sizeRegistered.height * sizeRegistered.width * sizeof(float);

  CHECK_CL_PARAM(data->bufferDepth = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeDepth, NULL, &err));
  CHECK_CL_PARAM(data->bufferScaled = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeRegistered, NULL, &err));
  CHECK_CL_PARAM(data->bufferRegistered = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeRegistered, NULL, &err));
  CHECK_CL_PARAM(data->bufferIndex = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeIndex, NULL, &err));
  CHECK_CL_PARAM(data->bufferImgZ = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeImgZ, NULL, &err));
  CHECK_CL_PARAM(data->bufferDists = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeDists, NULL, &err));
  CHECK_CL_PARAM(data->bufferSelDist = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeSelDist, NULL, &err));
  CHECK_CL_PARAM(data->bufferMapX = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeMap, NULL, &err));
  CHECK_CL_PARAM(data->bufferMapY = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeMap, NULL, &err));
  CHECK_CL_PARAM(data->bufferOutput = cl::Buffer(data->context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR, data->sizeRegistered, NULL, &err));

  CHECK_CL_PARAM(data->kernelSetZero = cl::Kernel(data->program, "setZero", &err));
  CHECK_CL_RETURN(data->kernelSetZero.setArg(0, data->bufferRegistered));
  CHECK_CL_RETURN(data->kernelSetZero.setArg(1, data->bufferSelDist));

  CHECK_CL_PARAM(data->kernelProject = cl::Kernel(data->program, "project", &err));
  CHECK_CL_RETURN(data->kernelProject.setArg(0, data->bufferScaled));
  CHECK_CL_RETURN(data->kernelProject.setArg(1, data->bufferIndex));
  CHECK_CL_RETURN(data->kernelProject.setArg(2, data->bufferImgZ));
  CHECK_CL_RETURN(data->kernelProject.setArg(3, data->bufferDists));
  CHECK_CL_RETURN(data->kernelProject.setArg(4, data->bufferSelDist));
  CHECK_CL_RETURN(data->kernelProject.setArg(5, data->bufferRegistered));

  CHECK_CL_PARAM(data->kernelCheckDepth = cl::Kernel(data->program, "checkDepth", &err));
  CHECK_CL_RETURN(data->kernelCheckDepth.setArg(0, data->bufferIndex));
  CHECK_CL_RETURN(data->kernelCheckDepth.setArg(1, data->bufferImgZ));
  CHECK_CL_RETURN(data->kernelCheckDepth.setArg(2, data->bufferDists));
  CHECK_CL_RETURN(data->kernelCheckDepth.setArg(3, data->bufferSelDist));
  CHECK_CL_RETURN(data->kernelCheckDepth.setArg(4, data->bufferRegistered));

  CHECK_CL_PARAM(data->kernelRemap = cl::Kernel(data->program, "remapDepth", &err));
  CHECK_CL_RETURN(data->kernelRemap.setArg(0, data->bufferDepth));
  CHECK_CL_RETURN(data->kernelRemap.setArg(1, data->bufferScaled));
  CHECK_CL_RETURN(data->kernelRemap.setArg(2, data->bufferMapX));
  CHECK_CL_RETURN(data->kernelRemap.setArg(3, data->bufferMapY));

  CHECK_CL_RETURN(data->queue.enqueueWriteBuffer(data->bufferMapX, CL_TRUE, 0, data->sizeMap, mapX.data));
  CHECK_CL_RETURN(data->queue.enqueueWriteBuffer(data->bufferMapY, CL_TRUE, 0, data->sizeMap, mapY.data));

  CHECK_CL_PARAM(data->dataOutput = (unsigned char *)data->queue.enqueueMapBuffer(data->bufferOutput, CL_TRUE, CL_MAP_READ, 0, data->sizeRegistered, NULL, NULL, &err));
  return true;
}

bool DepthRegistrationOpenCL::registerDepth(const cv::Mat &depth, cv::Mat &registered)
{
  cl::Event eventRead;
  std::vector<cl::Event> eventZero(2), eventRemap(1), eventProject(1), eventCheckDepth1(1), eventCheckDepth2(1);
  cl::NDRange range(sizeRegistered.height * sizeRegistered.width);

  CHECK_CL_RETURN(data->queue.enqueueWriteBuffer(data->bufferDepth, CL_FALSE, 0, data->sizeDepth, depth.data, NULL, &eventZero[0]));
  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelSetZero, cl::NullRange, range, cl::NullRange, NULL, &eventZero[1]));

  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelRemap, cl::NullRange, range, cl::NullRange, &eventZero, &eventRemap[0]));

  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelProject, cl::NullRange, range, cl::NullRange, &eventRemap, &eventProject[0]));

  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelCheckDepth, cl::NullRange, range, cl::NullRange, &eventProject, &eventCheckDepth1[0]));

  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelCheckDepth, cl::NullRange, range, cl::NullRange, &eventCheckDepth1, &eventCheckDepth2[0]));

  CHECK_CL_RETURN(data->queue.enqueueReadBuffer(data->bufferRegistered, CL_FALSE, 0, data->sizeRegistered, data->dataOutput, &eventCheckDepth2, &eventRead));

  CHECK_CL_RETURN(eventRead.wait());

  registered = cv::Mat(sizeRegistered, CV_16U, data->dataOutput);

#ifdef ENABLE_PROFILING_CL
  if(data->count == 0)
  {
    data->timings.clear();
    data->timings.resize(7, 0.0);
  }

  data->timings[0] += eventZero[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventZero[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[1] += eventZero[1].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventZero[1].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[2] += eventRemap[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventRemap[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[3] += eventProject[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventProject[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[4] += eventCheckDepth1[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventCheckDepth1[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[5] += eventCheckDepth2[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventCheckDepth2[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[6] += eventRead.getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventRead.getProfilingInfo<CL_PROFILING_COMMAND_START>();

  if(++data->count == 100)
  {
    double sum = data->timings[0] + data->timings[1] + data->timings[2] + data->timings[3] + data->timings[4] + data->timings[5] + data->timings[6];
    OUT_INFO("writing depth: " << data->timings[0] / 100000000.0 << " ms.");
    OUT_INFO("setting zero: " << data->timings[1] / 100000000.0 << " ms.");
    OUT_INFO("remap: " << data->timings[2] / 100000000.0 << " ms.");
    OUT_INFO("project: " << data->timings[3] / 100000000.0 << " ms.");
    OUT_INFO("check depth 1: " << data->timings[4] / 100000000.0 << " ms.");
    OUT_INFO("check depth 2: " << data->timings[5] / 100000000.0 << " ms.");
    OUT_INFO("read registered: " << data->timings[6] / 100000000.0 << " ms.");
    OUT_INFO("overall: " << sum / 100000000.0 << " ms.");
    data->count = 0;
  }
#endif
  return true;
}

void DepthRegistrationOpenCL::generateOptions(std::string &options) const
{
  std::ostringstream oss;
  oss.precision(16);
  oss << std::scientific;

  // Rotation
  oss << " -D r00=" << rotation.at<double>(0, 0) << "f";
  oss << " -D r01=" << rotation.at<double>(0, 1) << "f";
  oss << " -D r02=" << rotation.at<double>(0, 2) << "f";
  oss << " -D r10=" << rotation.at<double>(1, 0) << "f";
  oss << " -D r11=" << rotation.at<double>(1, 1) << "f";
  oss << " -D r12=" << rotation.at<double>(1, 2) << "f";
  oss << " -D r20=" << rotation.at<double>(2, 0) << "f";
  oss << " -D r21=" << rotation.at<double>(2, 1) << "f";
  oss << " -D r22=" << rotation.at<double>(2, 2) << "f";

  // Translation
  oss << " -D tx=" << translation.at<double>(0, 0) << "f";
  oss << " -D ty=" << translation.at<double>(1, 0) << "f";
  oss << " -D tz=" << translation.at<double>(2, 0) << "f";

  // Camera parameter upscaled depth
  oss << " -D fxR=" << cameraMatrixRegistered.at<double>(0, 0) << "f";
  oss << " -D fyR=" << cameraMatrixRegistered.at<double>(1, 1) << "f";
  oss << " -D cxR=" << cameraMatrixRegistered.at<double>(0, 2) << "f";
  oss << " -D cyR=" << cameraMatrixRegistered.at<double>(1, 2) << "f";
  oss << " -D fxRInv=" << (1.0 / cameraMatrixRegistered.at<double>(0, 0)) << "f";
  oss << " -D fyRInv=" << (1.0 / cameraMatrixRegistered.at<double>(1, 1)) << "f";

  // Clipping distances
  oss << " -D zNear=" << (uint16_t)(zNear * 1000);
  oss << " -D zFar=" << (uint16_t)(zFar * 1000);

  // Size registered image
  oss << " -D heightR=" << sizeRegistered.height;
  oss << " -D widthR=" << sizeRegistered.width;

  // Size depth image
  oss << " -D heightD=" << sizeDepth.height;
  oss << " -D widthD=" << sizeDepth.width;

  options = oss.str();
}

bool DepthRegistrationOpenCL::readProgram(std::string &source) const
{
  std::ifstream file(REG_OPENCL_FILE);

  if(!file.is_open())
  {
    return false;
  }

  source = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  return true;
}
