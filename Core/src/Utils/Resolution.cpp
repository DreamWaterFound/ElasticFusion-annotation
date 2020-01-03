/**
 * @file Resolution.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 分辨率对象的功能实现
 * @version 0.1
 * @date 2020-01-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
*
*/

#include "Resolution.h"

// 获取给定分辨率的分辨率对象实例
const Resolution & Resolution::getInstance(int width,int height)
{
  static const Resolution instance(width,height);
  return instance;
}

