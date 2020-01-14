/**
 * @file device_array.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 提供对于设备端一维数组和二维数组的模板类封装
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

#ifndef DEVICE_ARRAY_HPP_
#define DEVICE_ARRAY_HPP_

#include "device_memory.hpp"

#include <vector>

// TODO 更改这个文件中的注释格式

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief @b DeviceArray class 说白了就是保存在 GPU 中的模板类数组
  * @note Typed container for GPU memory with reference counting.
  * @tparam T 单元类型
  * @author Anatoly Baksheev
  */
template<class T>
class DeviceArray : public DeviceMemory
{
    public:
        /** @brief Element type. */
        typedef T type;

        /** @brief Element size. */
        // ? 为什么要写成这个样子呢?
        enum { elem_size = sizeof(T) };

        /** @brief Empty constructor. */
        DeviceArray();

        /** @brief Allocates internal buffer in GPU memory
          * @param size_t: number of elements to allocate
          * */
        DeviceArray(size_t size);

        /** @brief Initializes with user allocated buffer. Reference counting is disabled in this case.
          * @param ptr: pointer to buffer
          * @param size: elemens number
          * */
        DeviceArray(T *ptr, size_t size);

        /** @brief Copy constructor. Just increments reference counter. */
        DeviceArray(const DeviceArray& other);

        /** @brief Assigment operator. Just increments reference counter. */
        DeviceArray& operator = (const DeviceArray& other);

        /** @brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
          * @param size: elemens number
          * */
        void create(size_t size);

        /** @brief Decrements reference counter and releases internal buffer if needed. */
        void release();

        /** @brief Performs data copying. If destination size differs it will be reallocated.
          * @param other_arg: destination container
          * */
        void copyTo(DeviceArray& other) const;

        /** @brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
          * @param host_ptr_arg: pointer to buffer to upload
          * @param size: elemens number
          * */
        void upload(const T *host_ptr, size_t size);

        /** @brief Downloads data from internal buffer to CPU memory
          * @param host_ptr_arg: pointer to buffer to download
          * */
        void download(T *host_ptr) const;

        /** @brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough. 指定了分配器 A
          * @param data: host vector to upload from
          * */
        // ? 这个类型的函数应该有其他的用途
        template<class A>
        void upload(const std::vector<T, A>& data);

         /** @brief Downloads data from internal buffer to CPU memory 指定了分配器 A
           * @param data:  host vector to download to
           * */
        template<typename A>
        void download(std::vector<T, A>& data) const;

        /** @brief Performs swap of data pointed with another device array.
          * @param other: device array to swap with
          * */
        void swap(DeviceArray& other_arg);

        /** @brief Returns pointer for internal buffer in GPU memory. */
        T* ptr();

        /** @brief Returns const pointer for internal buffer in GPU memory. */
        const T* ptr() const;

        //using DeviceMemory::ptr;

        /** @brief Returns pointer for internal buffer in GPU memory. 重载了括号运算符*/
        operator T*();

        /** @brief Returns const pointer for internal buffer in GPU memory. */
        operator const T*() const;

        /** @brief Returns size in elements. */
        size_t size() const;
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief @b DeviceArray2D class
  * @note Typed container for pitched GPU memory with reference counting.
  * @author Anatoly Baksheev
  */
template<class T>
class DeviceArray2D : public DeviceMemory2D
{
    public:
        /** @brief Element type. */
        typedef T type;

        /** @brief Element size. */
        enum { elem_size = sizeof(T) };

        /** @brief Empty constructor. */
        DeviceArray2D();

        /** @brief Allocates internal buffer in GPU memory
          * @param rows: number of rows to allocate
          * @param cols: number of elements in each row
          * */
        DeviceArray2D(int rows, int cols);

        /** @brief Initializes with user allocated buffer. Reference counting is disabled in this case.
          * @param rows: number of rows
          * @param cols: number of elements in each row
          * @param data: pointer to buffer
          * @param stepBytes: stride between two consecutive rows in bytes
          * */
        DeviceArray2D(int rows, int cols, void *data, size_t stepBytes);

        /** @brief Copy constructor. Just increments reference counter. */
        DeviceArray2D(const DeviceArray2D& other);

        /** @brief Assigment operator. Just increments reference counter. */
        DeviceArray2D& operator = (const DeviceArray2D& other);

        /** @brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
          * @param rows: number of rows to allocate
          * @param cols: number of elements in each row
          * */
        void create(int rows, int cols);

        /** @brief Decrements reference counter and releases internal buffer if needed. */
        void release();

        /** @brief Performs data copying. If destination size differs it will be reallocated.
          * @param other: destination container
          * */
        void copyTo(DeviceArray2D& other) const;

        /** @brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
          * @param host_ptr: pointer to host buffer to upload
          * @param host_step: stride between two consecutive rows in bytes for host buffer
          * @param rows: number of rows to upload
          * @param cols: number of elements in each row
          * */
        void upload(const void *host_ptr, size_t host_step, int rows, int cols);

        /** @brief Downloads data from internal buffer to CPU memory. User is resposible for correct host buffer size.
          * @param host_ptr: pointer to host buffer to download
          * @param host_step: stride between two consecutive rows in bytes for host buffer
          * */
        void download(void *host_ptr, size_t host_step) const;

        /** @brief Performs swap of data pointed with another device array.
          * @param other: device array to swap with
          * */
        void swap(DeviceArray2D& other_arg);

        /** @brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
          * @param data: host vector to upload from
          * @param cols: stride in elements between  two consecutive rows in bytes for host buffer
          * */
        //  ? 这个形式应该有特殊用途
        template<class A>
        void upload(const std::vector<T, A>& data, int cols);

        /** @brief Downloads data from internal buffer to CPU memory
          * @param data: host vector to download to
          * \@aram cols: Output stride in elements between two consecutive rows in bytes for host vector.
          * */
        template<class A>
        void download(std::vector<T, A>& data, int& cols) const;

        /** @brief Returns pointer to given row in internal buffer.
          * @param y_arg: row index
          * */
        T* ptr(int y = 0);

        /** @brief Returns const pointer to given row in internal buffer.
          * @param y_arg: row index
          * */
        const T* ptr(int y = 0) const;
        
        //using DeviceMemory2D::ptr;

        /** @brief Returns pointer for internal buffer in GPU memory. */
        operator T*();

        /** @brief Returns const pointer for internal buffer in GPU memory. */
        operator const T*() const;
        
        /** @brief Returns number of elements in each row. */
        int cols() const;

        /** @brief Returns number of rows. */
        int rows() const;

        /** @brief Returns step in elements. */
        size_t elem_step() const;
};

// 包含了 DeviceArray DeviceArray2D 的内联函数实现
#include "device_array_impl.hpp"

#endif /* DEVICE_ARRAY_HPP_ */
