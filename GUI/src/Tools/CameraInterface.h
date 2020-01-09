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

// STL
#include <string>
#include <iostream>
#include <algorithm>
#include <map>

// 加强的线程互斥锁对象
#include "ThreadMutexObject.h"

/** @brief 相机接口虚基类 */
class CameraInterface
{
    public:
      /** @brief 默认析构函数 */
      virtual ~CameraInterface() {}

      /**
       * @brief 判断相机是否正确初始化
       * @return true 
       * @return false 
       */
      virtual bool ok() = 0;

      /**
       * @brief 获取错误描述字符串
       * @return std::string 错误描述字符串
       */
      virtual std::string error() = 0;
      
      static const int numBuffers = 10;   ///< 图像缓冲区大小, 指的是缓冲的图像的个数
      
      ThreadMutexObject<int> latestDepthIndex;    ///< 被线程锁保护的变量, 记录最近一帧的深度图id
      std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> frameBuffers[numBuffers];   ///< 帧图像缓冲区, 前两个应该分别是深度图和彩色图, 最后是深度图像的时间戳

      /**
       * @brief 设置自动曝光
       * @param[in] value 是否
       */
      virtual void setAutoExposure(bool value) = 0;

      /**
       * @brief 设置自动白平衡
       * @param[in] value 是否
       */
      virtual void setAutoWhiteBalance(bool value) = 0;
};
