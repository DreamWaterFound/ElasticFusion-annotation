/**
 * @file WindowsExtras.h
 * @author guoqing (1337841346@qq.com)
 * @brief 由于 windows 下没有Windows下的一些函数, 所以这里撰写了用于 windows 下工作的同名替代函数
 * @version 0.1
 * @date 2020-01-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
* 
*/

#pragma once

#include "../Defines.h"

#ifdef WIN32

EFUSION_API int gettimeofday(struct timeval * tp,struct timezone * tzp);

EFUSION_API void *mempcpy(void *dest,const void *src,size_t n);

#endif // WIN32