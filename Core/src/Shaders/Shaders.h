/**
 * @file Shaders.h
 * @author guoqing (1337841346@qq.com)
 * @brief 用于管理着色器程序的类, 以及加载 GLSL 程序使用的全局静态函数
 * @version 0.1
 * @date 2020-01-06
 * 
 * @copyright Copyright (c) 2020
 * 
 */

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

#ifndef SHADERS_SHADERS_H_
#define SHADERS_SHADERS_H_

// GLSL
#include <pangolin/gl/glsl.h>
// STL
#include <memory>

// 命令行参数解析
#include "../Utils/Parse.h"
// ?
#include "Uniform.h"

/** @brief ElasticFusion 对 GLSL 语言程序的封装 */
class Shader : public pangolin::GlSlProgram
{
    public:
        // 空构造函数
        Shader()
        {}

        GLuint programId()
        {
            return prog;
        }

        void setUniform(const Uniform & v)
        {
            GLuint loc = glGetUniformLocation(prog, v.id.c_str());

            switch(v.t)
            {
                case Uniform::INT:
                    glUniform1i(loc, v.i);
                    break;
                case Uniform::FLOAT:
                    glUniform1f(loc, v.f);
                    break;
                case Uniform::VEC2:
                    glUniform2f(loc, v.v2(0), v.v2(1));
                    break;
                case Uniform::VEC3:
                    glUniform3f(loc, v.v3(0), v.v3(1), v.v3(2));
                    break;
                case Uniform::VEC4:
                    glUniform4f(loc, v.v4(0), v.v4(1), v.v4(2), v.v4(3));
                    break;
                case Uniform::MAT4:
                    glUniformMatrix4fv(loc, 1, false, v.m4.data());
                    break;
                default:
                    assert(false && "Uniform type not implemented!");
                    break;
            }
        }
};

// ==================== 静态全局函数 ==================

static inline std::shared_ptr<Shader> loadProgramGeomFromFile(const std::string& vertex_shader_file, const std::string& geometry_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, Parse::get().shaderDir() + "/" + vertex_shader_file, {}, {Parse::get().shaderDir()});
    program->AddShaderFromFile(pangolin::GlSlGeometryShader, Parse::get().shaderDir() + "/" + geometry_shader_file, {}, {Parse::get().shaderDir()});
    program->Link();

    return program;
}

static inline std::shared_ptr<Shader> loadProgramFromFile(const std::string& vertex_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, Parse::get().shaderDir() + "/" + vertex_shader_file, {}, {Parse::get().shaderDir()});
    program->Link();

    return program;
}

static inline std::shared_ptr<Shader> loadProgramFromFile(const std::string& vertex_shader_file, const std::string& fragment_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, Parse::get().shaderDir() + "/" + vertex_shader_file, {}, {Parse::get().shaderDir()});
    program->AddShaderFromFile(pangolin::GlSlFragmentShader, Parse::get().shaderDir() + "/" + fragment_shader_file, {}, {Parse::get().shaderDir()});
    program->Link();

    return program;
}

/**
 * @brief 加载 GLSL 程序, 用于颜色的渲染
 * @param[in] vertex_shader_file        顶点着色器
 * @param[in] fragment_shader_file      片段着色器
 * @param[in] geometry_shader_file      集合着色器
 * @return std::shared_ptr<Shader>      生成的 Shader 对象的共享指针
 */
static inline std::shared_ptr<Shader> loadProgramFromFile(const std::string& vertex_shader_file, const std::string& fragment_shader_file, const std::string& geometry_shader_file)
{
    // 生成一个 Shader 程序对象
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    // 设置顶点着色器
    program->AddShaderFromFile(
        pangolin::GlSlVertexShader,                             // 着色器类型
        Parse::get().shaderDir() + "/" + vertex_shader_file,    // 着色器程序文件名
        {},                                                     // 着色器程序中的宏定义
        {Parse::get().shaderDir()});                            // 着色器程序的搜索路径
    
    // 设置几何着色器
    program->AddShaderFromFile(pangolin::GlSlGeometryShader, Parse::get().shaderDir() + "/" + geometry_shader_file, {}, {Parse::get().shaderDir()});
    // 设置片段着色器
    program->AddShaderFromFile(pangolin::GlSlFragmentShader, Parse::get().shaderDir() + "/" + fragment_shader_file, {}, {Parse::get().shaderDir()});

    // 调用 glLinkProgram, 连接 GLSL 程序, 创建可执行文件对象
    program->Link();

    return program;
}

#endif /* SHADERS_SHADERS_H_ */
