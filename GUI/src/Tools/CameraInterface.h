/**
 * @file CameraInterface.h
 * @author guoqing (1337841346@qq.com)
 * @brief 相机接口虚基类
 * @version 0.1
 * @date 2020-01-04
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

// ?
#include "ThreadMutexObject.h"

/** @brief 相机接口虚基类 */
class CameraInterface
{
    public:
      // 使用默认构造函数

      
      virtual ~CameraInterface() {}

      virtual bool ok() = 0;
      virtual std::string error() = 0;

      static const int numBuffers = 10;
      ThreadMutexObject<int> latestDepthIndex;
      std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> frameBuffers[numBuffers];

      virtual void setAutoExposure(bool value) = 0;
      virtual void setAutoWhiteBalance(bool value) = 0;
};
