/**
 * @file Intrinsics.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 相机内参类对象的实现
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

#include "Intrinsics.h"

// 使用给定内参生成内参类对象
const Intrinsics & Intrinsics::getInstance(float fx,float fy,float cx,float cy)
{
  static const Intrinsics instance(fx,fy,cx,cy);
  return instance;
}

