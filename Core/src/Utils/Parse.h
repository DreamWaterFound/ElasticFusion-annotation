/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */
#ifndef PARSE_H_
#define PARSE_H_

#ifdef WIN32
#  include <Windows.h>
#else
#  include <dirent.h>
#endif

#include <string>
#include <cassert>
#ifndef WIN32
#  include <unistd.h>
#endif
#include <string.h>
#include <pangolin/utils/file_utils.h>

#include "../Defines.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)

/** @brief 命令行参数解析 */
class Parse
{
    public:
        /**
         * @brief 获取当前解析器的一个实例. 静态成员函数
         * @return EFUSION_API const& get 实例引用
         */
        EFUSION_API static const Parse & get();

        /**
         * @brief 对给出的字符型参数进行解析操作
         * @param[in] argc 命令行参数个数
         * @param[in] argv 命令行参数表列
         * @param[in] str 要寻找的参数名称
         * @param[in] val 找到的命令行参数项
         * @return 参数项的id. 负数表示没有找到
         */
        EFUSION_API int arg(int argc, char** argv, const char* str, std::string &val) const;

        EFUSION_API int arg(int argc, char** argv, const char* str, float &val) const;

        EFUSION_API int arg(int argc, char** argv, const char* str, int &val) const;

        EFUSION_API std::string shaderDir() const;

        EFUSION_API std::string baseDir() const;

    private:
        EFUSION_API Parse();

        /**
         * @brief 寻找给定的参数名称出现在的参数id
         * @param[in] argc 命令行参数个数
         * @param[in] argv 命令行参数表列
         * @param[in] argument_name 要寻找的参数名称
         * @return id. 如果找不到返回-1
         */
        EFUSION_API int findArg(int argc,char** argv,const char* argument_name) const;
};

#endif /* PARSE_H_ */
