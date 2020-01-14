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

#ifndef DEVICE_ARRAY_IMPL_HPP_
#define DEVICE_ARRAY_IMPL_HPP_


/////////////////////  Inline implementations of DeviceArray ////////////////////////////////////////////

// 空构造函数
template<class T> inline DeviceArray<T>::DeviceArray() {}
// 构造函数, 分配 size 个对象
template<class T> inline DeviceArray<T>::DeviceArray(size_t size) : DeviceMemory(size * elem_size) {}
// 构造函数, 外部已经分配好空间
template<class T> inline DeviceArray<T>::DeviceArray(T *ptr, size_t size) : DeviceMemory(ptr, size * elem_size) {}
// 拷贝构造函数
template<class T> inline DeviceArray<T>::DeviceArray(const DeviceArray& other) : DeviceMemory(other) {}
// 重载赋值运算符
template<class T> inline DeviceArray<T>& DeviceArray<T>::operator=(const DeviceArray& other)
{ DeviceMemory::operator=(other); return *this; }

// 分配 size 个元素的显存空间
template<class T> inline void DeviceArray<T>::create(size_t size)
{ DeviceMemory::create(size * elem_size); }
// 释放已经分配的显存空间
template<class T> inline void DeviceArray<T>::release()
{ DeviceMemory::release(); }

// 显存之间的拷贝
template<class T> inline void DeviceArray<T>::copyTo(DeviceArray& other) const
{ DeviceMemory::copyTo(other); }
// 将内存中的数据上传到显存中
template<class T> inline void DeviceArray<T>::upload(const T *host_ptr, size_t size)
{ DeviceMemory::upload(host_ptr, size * elem_size); }
// 将显存中的数据下载到内存中
template<class T> inline void DeviceArray<T>::download(T *host_ptr) const
{ DeviceMemory::download( host_ptr ); }

// 和目标对象交换数据
template<class T> void DeviceArray<T>::swap(DeviceArray& other_arg) { DeviceMemory::swap(other_arg); }

template<class T> inline DeviceArray<T>::operator T*() { return ptr(); }                                // 重载了括号运算符获取原始指针(可修改)
template<class T> inline DeviceArray<T>::operator const T*() const { return ptr(); }                    // 重载了括号运算符获取原始指针(只读)
template<class T> inline size_t DeviceArray<T>::size() const { return sizeBytes() / elem_size; }        // 获得数组内元素的个数

template<class T> inline       T* DeviceArray<T>::ptr()       { return DeviceMemory::ptr<T>(); }        // 获取原始指针(可修改)
template<class T> inline const T* DeviceArray<T>::ptr() const { return DeviceMemory::ptr<T>(); }        // 获取原始指针(只读)

// 指定了 vector 的分配器类型 A 的上传和下载函数
template<class T> template<class A> inline void DeviceArray<T>::upload(const std::vector<T, A>& data) { upload(&data[0], data.size()); }
template<class T> template<class A> inline void DeviceArray<T>::download(std::vector<T, A>& data) const { data.resize(size()); if (!data.empty()) download(&data[0]); }

/////////////////////  Inline implementations of DeviceArray2D ////////////////////////////////////////////

// 空构造函数
template<class T> inline DeviceArray2D<T>::DeviceArray2D() {}
// 构造函数, 按照指定的行数和列数分配显存空间
template<class T> inline DeviceArray2D<T>::DeviceArray2D(int rows, int cols) : DeviceMemory2D(rows, cols * elem_size) {}
// 构造函数, 使用外部已经分配的显存空间信息进行初始化
template<class T> inline DeviceArray2D<T>::DeviceArray2D(int rows, int cols, void *data, size_t stepBytes) : DeviceMemory2D(rows, cols * elem_size, data, stepBytes) {}
// 拷贝构造函数
template<class T> inline DeviceArray2D<T>::DeviceArray2D(const DeviceArray2D& other) : DeviceMemory2D(other) {}
// 重载了赋值运算符
template<class T> inline DeviceArray2D<T>& DeviceArray2D<T>::operator=(const DeviceArray2D& other)
{ DeviceMemory2D::operator=(other); return *this; }

// 分配指定大小的显存区域
template<class T> inline void DeviceArray2D<T>::create(int rows, int cols)
{ DeviceMemory2D::create(rows, cols * elem_size); }
// 释放已经分配的显存区域
template<class T> inline void DeviceArray2D<T>::release()
{ DeviceMemory2D::release(); }

// 显存到显存的拷贝函数
template<class T> inline void DeviceArray2D<T>::copyTo(DeviceArray2D& other) const
{ DeviceMemory2D::copyTo(other); }
// 将二维数组数据从内存上传到显存中, 注意这里也需要提供内存端的 step 参数
template<class T> inline void DeviceArray2D<T>::upload(const void *host_ptr, size_t host_step, int rows, int cols)
{ DeviceMemory2D::upload(host_ptr, host_step, rows, cols * elem_size); }
// 将二维数组数据从显存中下载到内存中
template<class T> inline void DeviceArray2D<T>::download(void *host_ptr, size_t host_step) const
{ DeviceMemory2D::download( host_ptr, host_step ); }

// 上传, 格式略有不同
template<class T> template<class A> inline void DeviceArray2D<T>::upload(const std::vector<T, A>& data, int cols)
{ upload(&data[0], cols * elem_size, data.size()/cols, cols); }
// 下载, 格式略有不同
template<class T> template<class A> inline void DeviceArray2D<T>::download(std::vector<T, A>& data, int& elem_step) const
{ elem_step = cols(); data.resize(cols() * rows()); if (!data.empty()) download(&data[0], colsBytes());  }

// 交换两个二维GPU数组对象的数据
template<class T> void  DeviceArray2D<T>::swap(DeviceArray2D& other_arg) { DeviceMemory2D::swap(other_arg); }

template<class T> inline       T* DeviceArray2D<T>::ptr(int y)       { return DeviceMemory2D::ptr<T>(y); }  // 获取原始指针(可修改)
template<class T> inline const T* DeviceArray2D<T>::ptr(int y) const { return DeviceMemory2D::ptr<T>(y); }  // 获取原始指针(只读)
            
template<class T> inline DeviceArray2D<T>::operator T*() { return ptr(); }                                  // 重载括号运算符获取原始指针(可修改)
template<class T> inline DeviceArray2D<T>::operator const T*() const { return ptr(); }                      // 重载括号运算符获取原始指针(只读)

// 获取图像列数
template<class T> inline int DeviceArray2D<T>::cols() const { return DeviceMemory2D::colsBytes()/elem_size; }
// 获取图像行数
template<class T> inline int DeviceArray2D<T>::rows() const { return DeviceMemory2D::rows(); }
// 获取相当于 step 了多少个元素
template<class T> inline size_t DeviceArray2D<T>::elem_step() const { return DeviceMemory2D::step()/elem_size; }


#endif /* DEVICE_ARRAY_IMPL_HPP_ */
