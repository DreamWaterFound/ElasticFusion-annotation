/**
 * @file Defines.h
 * @author guoqing (1337841346@qq.com)
 * @brief 一些定义
 * @version 0.1
 * @date 2020-01-06
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

#ifndef WIN32
// Linux 下就没有这么多事情
#  define EFUSION_API
#else
// windows 下编译的 *.dll 文件有权限限制, 外部程序如果想使用 dll 中的内容, 它自己需要 dllimport(其实不用也可以);
// 同样我们必须需要给暴露给外部程序的函数前面添加这个 dllexport
// ref:[https://blog.csdn.net/huangyimo/article/details/81748939]
#  ifdef efusion_EXPORTS
#    define EFUSION_API __declspec(dllexport)
#  else
#    define EFUSION_API __declspec(dllimport)
#  endif
#endif
