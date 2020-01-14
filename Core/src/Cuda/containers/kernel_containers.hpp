/**
 * @file kernel_containers.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 为了方便, 定义的一些增强型指针, 主要用于对显存中的图像数据进行操作
 * @version 0.1
 * @date 2020-01-14
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */


#ifndef KERNEL_CONTAINERS_HPP_
#define KERNEL_CONTAINERS_HPP_

// ?
#include <cstddef>

// 只有被 CUDA 程序包含时, 使用 NVCC 进行编译, 此时 __CUDACC__ 被定义
// ref: [https://docs.nvidia.com/cuda/cuda-compiler-driver-nvcc/index.html]
#if defined(__CUDACC__)
    #define GPU_HOST_DEVICE__ __host__ __device__ __forceinline__
#else
    #define GPU_HOST_DEVICE__
#endif

/**
 * @brief 指向 GPU 中的类型 T 元素的指针
 * @tparam T 
 */
template<typename T> struct DevPtr
{
    typedef T elem_type;
    /// 这个指针指向的元素的大小
    const static size_t elem_size = sizeof(elem_type);

    /// 指针本身
    T* data;

    /** @brief 空构造函数, 生成空指针 */
    GPU_HOST_DEVICE__ DevPtr() : data(0) {}
    /**
     * @brief 指定初始值的构造函数
     * @param[in] data_arg 初始值
     */
    GPU_HOST_DEVICE__ DevPtr(T* data_arg) : data(data_arg) {}

    /**
     * @brief 获取指向的单个类型 T 的元素的占有字节个数
     * @return elemSize 大小
     */
    GPU_HOST_DEVICE__ size_t elemSize() const { return elem_size; }

    /**
     * @brief 获取指针(可修改)
     * @return T* 
     */
    GPU_HOST_DEVICE__ operator       T*()       { return data; }
    /**
     * @brief 获取指针(不可修改)
     * @return const T* 
     */
    GPU_HOST_DEVICE__ operator const T*() const { return data; }
};

/**
 * @brief 指向 GPU 中的类型 T 元素的指针, 包含了数组中元素的个数信息, 用于调用核函数的时候传递指针
 * @tparam T 
 */
template<typename T> struct PtrSz : public DevPtr<T>
{
    /** @brief 空构造函数, 指针初始化为空指针 */
    GPU_HOST_DEVICE__ PtrSz() : size(0) {}
    /**
     * @brief 构造函数
     * @param[in] data_arg  指针初始值
     * @param[in] size_arg  数组中的元素个数
     */
    GPU_HOST_DEVICE__ PtrSz(T* data_arg, size_t size_arg) : DevPtr<T>(data_arg), size(size_arg) {}

    /// 当前指针指向的区域中有几个连续的 T 类型的变量
    size_t size;
};

/**
 * @brief 指向 GPU 中的类型 T 元素的指针, 支持指定步长访问; 用于获取图像行指针
 * @tparam T 
 */
template<typename T>  struct PtrStep : public DevPtr<T>
{
    /** @brief 空构造函数 */
    GPU_HOST_DEVICE__ PtrStep() : step(0) {}
    /**
     * @brief 构造函数
     * @param[in] data_arg 指针初始值
     * @param[in] step_arg 访问步长(Bytes)
     */
    GPU_HOST_DEVICE__ PtrStep(T* data_arg, size_t step_arg) : DevPtr<T>(data_arg), step(step_arg) {}

    /// stride between two consecutive rows in bytes. Step is stored always and everywhere in bytes!!!
    size_t step;

    /**
     * @brief 获取从当前指针起加 y 行的像素所在地址(可改写)
     * @param[in] y 
     * @return * ptr 
     */
    GPU_HOST_DEVICE__       T* ptr(int y = 0)       { return (      T*)( (      char*)DevPtr<T>::data + y * step); }
    /**
     * @brief 获取从当前指针起加 y 行的像素所在地址(不可改写)
     * @param[in] y 
     * @return * ptr 
     */
    GPU_HOST_DEVICE__ const T* ptr(int y = 0) const { return (const T*)( (const char*)DevPtr<T>::data + y * step); }
};

/**
 * @brief 指向 GPU 中的类型 T 元素的指针, 支持指定步长访问; 用于获取图像中指定位置的像素地址
 * @tparam T 
 */
// ? 但是相应的操作函数... 并没有实现?
template <typename T> struct PtrStepSz : public PtrStep<T>
{
    /** @brief 空构造函数 */
    GPU_HOST_DEVICE__ PtrStepSz() : cols(0), rows(0) {}
    /**
     * @brief 构造函数
     * @param[in] rows_arg 图像行数
     * @param[in] cols_arg 图像列数
     * @param[in] data_arg 数据指针初始值
     * @param[in] step_arg 每行对应的步长(字节数)
     */
    GPU_HOST_DEVICE__ PtrStepSz(int rows_arg, int cols_arg, T* data_arg, size_t step_arg)
        : PtrStep<T>(data_arg, step_arg), cols(cols_arg), rows(rows_arg) {}

    int cols;           ///< 图像列数
    int rows;           ///< 图像行数
};

#endif /* KERNEL_CONTAINERS_HPP_ */

