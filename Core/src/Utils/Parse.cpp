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

#include "Parse.h"

// 空构造函数
Parse::Parse()
{

}

// 获取当前解析器的一个实例
const Parse & Parse::get()
{
  static const Parse instance;
  return instance;
}

// 对给出的字符型参数进行解析工作
int Parse::arg(int argc, char** argv, const char* str, std::string &val) const
{
    // 寻找这个参数在的参数项id
    int index = findArg(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        // 说明找到了, 获取这个参数项的完整内容
        val = argv[index];
    }

    // 如果找不到, 这里就是负数
    return index - 1;
}

// 对给出的 float 型参数进行解析工作
int Parse::arg(int argc, char** argv, const char* str, float &val) const
{
    int index = findArg(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        // 将对应的字符串转换成为浮点数
        val = atof(argv[index]);
    }

    return index - 1;
}

// 对给出的 int 型参数进行解析工作
int Parse::arg(int argc, char** argv, const char* str, int &val) const
{
    
    int index = findArg(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        val = atoi(argv[index]);
    }

    return index - 1;
}

// 获取着色器程序的存放路径
std::string Parse::shaderDir() const
{
    // ? STR 就是 #SHADER_DIR
    // SHADER_DIR 这个宏在 <ElasticFusion_root>/Core/src/CMakeLists.txt 中被定义
    std::string currentVal = STR(SHADER_DIR);
    // 判断给定的文件是否存在, 否则就发生断言错误; 后面的字符串是在发生断言错误的时候用于提示我们的
    assert(pangolin::FileExists(currentVal) && "Shader directory not found!");
    return currentVal;
}

std::string Parse::baseDir() const
{
    char buf[256];
#ifdef WIN32
    int length = GetModuleFileName(NULL,buf,sizeof(buf));
#else
    int length = readlink("/proc/self/exe",buf,sizeof(buf));
#endif
    std::string currentVal;
    currentVal.append((char *)&buf, length);

    currentVal = currentVal.substr(0, currentVal
#ifdef WIN32
      .rfind("\\build\\"));
#else
      .rfind("/build/"));
#endif
    return currentVal;
}

// 寻找给定的参数名称出现在的参数id
int Parse::findArg(int argc, char** argv, const char* argument_name) const
{
    for(int i = 1; i < argc; ++i)
    {
        // Search for the string
        if(strcmp(argv[i], argument_name) == 0)
        {
            return i;
        }
    }
    return -1;
}
