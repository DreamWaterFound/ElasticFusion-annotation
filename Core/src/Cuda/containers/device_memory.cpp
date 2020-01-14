/**
 * @file device_memory.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 显存中的"团块" 容器类, 具有引用计数. 实现了一维和二维的容器
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

// 增强型指针
#include "device_memory.hpp"
// 自定义的 CUDA 工具函数
#include "../convenience.cuh"

// CUDA
#include "cuda_runtime_api.h"
// GNU C
#include "assert.h"

//  TODO
//////////////////////////    XADD    ///////////////////////////////

// 判断编译器类型
#ifdef __GNUC__
    // 如果版本号大于 4.2
    #if __GNUC__*10 + __GNUC_MINOR__ >= 42
        // 要求: 不是 windows 平台; 是 x86 架构CPU 或 支持 MMX SSE 指令集中的一种
        #if !defined WIN32 && (defined __i486__ || defined __i586__ || defined __i686__ || defined __MMX__ || defined __SSE__  || defined __ppc__)
            // 实现加法的原子操作 (type *ptr, type value) 后面的操作都差不多, 只不过是宏定义名称不同
            #define CV_XADD __sync_fetch_and_add
        #else
            #include <ext/atomicity.h>
            #define CV_XADD __gnu_cxx::__exchange_and_add
        #endif
    #else
        #include <bits/atomicity.h>
        #if __GNUC__*10 + __GNUC_MINOR__ >= 34
            #define CV_XADD __gnu_cxx::__exchange_and_add
        #else
            #define CV_XADD __exchange_and_add
        #endif
  #endif
    
#elif defined WIN32 || defined _WIN32
    #include <intrin.h>
    #define CV_XADD(addr,delta) _InterlockedExchangeAdd((long volatile*)(addr), (delta))
#else

    template<typename _Tp> static inline _Tp CV_XADD(_Tp* addr, _Tp delta)
    { int tmp = *addr; *addr += delta; return tmp; }
    
#endif

////////////////////////    DeviceArray    /////////////////////////////

// 空构造函数
DeviceMemory::DeviceMemory() : data_(0), sizeBytes_(0), refcount_(0) {}
// 构造函数, 给定数据区地址和长度. 此种情况下不使用引用计数
DeviceMemory::DeviceMemory(void *ptr_arg, size_t sizeBytes_arg) : data_(ptr_arg), sizeBytes_(sizeBytes_arg), refcount_(0){}
// 构造函数, 分配指定大小的显存空间
DeviceMemory::DeviceMemory(size_t sizeBtes_arg)  : data_(0), sizeBytes_(0), refcount_(0) { create(sizeBtes_arg); }
// 析构函数, 释放已经分配的空间
DeviceMemory::~DeviceMemory() { release(); }

// 拷贝构造函数
DeviceMemory::DeviceMemory(const DeviceMemory& other_arg)
    : data_(other_arg.data_), sizeBytes_(other_arg.sizeBytes_), refcount_(other_arg.refcount_)
{
    // 只增加引用计数. refcount_ == nullptr 说明分配内存区域, 或者是构造时直接给定了已经分配完成的数据区地址和长度(此时不使用引用计数)
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

// 重载赋值运算符
DeviceMemory& DeviceMemory::operator = (const DeviceMemory& other_arg)
{
    // 如果两个对象不相同
    if( this != &other_arg )
    {
        // 增加引用计数
        if( other_arg.refcount_ )
            CV_XADD(other_arg.refcount_, 1);
        // 释放自己曾经的数据
        release();
        // 赋值        
        data_      = other_arg.data_;
        sizeBytes_ = other_arg.sizeBytes_;                
        refcount_  = other_arg.refcount_;
    }
    return *this;
}

// 在显存中分配指定长度的区域
void DeviceMemory::create(size_t sizeBytes_arg)
{
    // 如果已经创建过等长的显存区域, 那么这次我们就重新利用
    if (sizeBytes_arg == sizeBytes_)
        return;
            
    // 如果和已经分配的显存区域长度不同的话, 我们就要重新分配了
    if( sizeBytes_arg > 0)
    {        
        // 先释放之前已经分配的区域
        if( data_ )
            release();

        // 分配显存空间
        sizeBytes_ = sizeBytes_arg;
        cudaSafeCall( cudaMalloc(&data_, sizeBytes_) );        
        // 新建引用计数
        refcount_ = new int;
        *refcount_ = 1;
    }
}

// 深拷贝, 如果目标区域大小和当前区域大小不同, 则会重新分配目标区域
void DeviceMemory::copyTo(DeviceMemory& other) const
{
    // 如果当前区域为空, 那么对方区域也应该为空, 进行释放操作
    if (empty())
        other.release();
    else
    {    
        // 目标区域重新分配等长空间(如果目标区域大小和当前区域大小相同, 并不会真正重新分配)
        other.create(sizeBytes_);    
        cudaSafeCall( cudaMemcpy(other.data_, data_, sizeBytes_, cudaMemcpyDeviceToDevice) );
        // 等待拷贝操作全部完成
        cudaSafeCall( cudaDeviceSynchronize() );
    }
}

// 减少引用计数并释放已经分配的显存区域
void DeviceMemory::release()
{
    // 条件: 已经分配过显存, 并且取消引用(引用计数原子操作减1)之后为引用计数为0, 那么我们自己完成析构操作
    // ?? 这里不应该是为0才执行这样的操作?
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }

    // 否则这个区域的释放操作就给其他使用它引用的程序完成, 这里直接初始化
    data_ = 0;
    sizeBytes_ = 0;
    refcount_ = 0;
}

// 主机端上传指定长度数据到设备端
void DeviceMemory::upload(const void *host_ptr_arg, size_t sizeBytes_arg)
{
    // 分配指定长度数据的内存, 避免大小不一致
    create(sizeBytes_arg);
    cudaSafeCall( cudaMemcpy(data_, host_ptr_arg, sizeBytes_, cudaMemcpyHostToDevice) );
    cudaSafeCall( cudaDeviceSynchronize() );
}

// 将数据从显存中下载到内存中 需要复制的长度由类成员变量指定
void DeviceMemory::download(void *host_ptr_arg) const
{    
    cudaSafeCall( cudaMemcpy(host_ptr_arg, data_, sizeBytes_, cudaMemcpyDeviceToHost) );
    cudaSafeCall( cudaDeviceSynchronize() );
}          

// 交换两个 DeviceMemory 的数据
void DeviceMemory::swap(DeviceMemory& other_arg)
{
    std::swap(data_, other_arg.data_);
    std::swap(sizeBytes_, other_arg.sizeBytes_);
    std::swap(refcount_, other_arg.refcount_);
}

// 当前区域是否为空
bool DeviceMemory::empty() const { return !data_; }
// 获取已分配显存区域的大小
size_t DeviceMemory::sizeBytes() const { return sizeBytes_; }


////////////////////////    DeviceArray2D    /////////////////////////////

// 空构造函数
DeviceMemory2D::DeviceMemory2D() : data_(0), step_(0), colsBytes_(0), rows_(0), refcount_(0) {}

// 构造函数, 指定行数以及每一行的字节数
DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg)
    : data_(0), step_(0), colsBytes_(0), rows_(0), refcount_(0)
{ 
    create(rows_arg, colsBytes_arg); 
}

// 使用给定的信息构造, 不分配显存空间, 不使用引用计数
DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg, void *data_arg, size_t step_arg)
    :  data_(data_arg), step_(step_arg), colsBytes_(colsBytes_arg), rows_(rows_arg), refcount_(0) {}

// 析构, 释放已经分配的显存区域
DeviceMemory2D::~DeviceMemory2D() { release(); }

// 拷贝构造函数
DeviceMemory2D::DeviceMemory2D(const DeviceMemory2D& other_arg) :
    data_(other_arg.data_), step_(other_arg.step_), colsBytes_(other_arg.colsBytes_), rows_(other_arg.rows_), refcount_(other_arg.refcount_)
{
    // 引用计数++
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

// 重载赋值运算符
DeviceMemory2D& DeviceMemory2D::operator = (const DeviceMemory2D& other_arg)
{
    // 如果目标变量相同就不需要进行别的操作了
    if( this != &other_arg )
    {
        // 引用计数++
        if( other_arg.refcount_ )
            CV_XADD(other_arg.refcount_, 1);
        
        // 释放自己之前已经分配的显存区域
        release();
        // 使用目标变量的
        colsBytes_ = other_arg.colsBytes_;
        rows_ = other_arg.rows_;
        data_ = other_arg.data_;
        step_ = other_arg.step_;
        refcount_ = other_arg.refcount_;
    }
    return *this;
}

// 分配 rows_arg 行, 一行有 colsBytes_arg 字节的显存区域
void DeviceMemory2D::create(int rows_arg, int colsBytes_arg)
{
    // 如果已经分配的区域大小和这个相同那么就不再重新分配了
    if (colsBytes_ == colsBytes_arg && rows_ == rows_arg)
        return;
            
    if( rows_arg > 0 && colsBytes_arg > 0)
    {        
        // 释放已分配显存(如果有的话)
        if( data_ )
            release();
              
        // 分配新显存区域
        colsBytes_ = colsBytes_arg;
        rows_ = rows_arg;
        // 调用 cudaMallocPitch 分配二维空间的时候会考虑到字节对齐问题
        // ref: [https://www.cnblogs.com/cuancuancuanhao/p/7805892.html]
        // step_ 保存一行的实际字节数
        cudaSafeCall( cudaMallocPitch( (void**)&data_, &step_, colsBytes_, rows_) );        
        // 生成引用计数
        refcount_ = new int;
        *refcount_ = 1;
    }
}

// 释放已经分配的显存空间
void DeviceMemory2D::release()
{
    // 如果到自己这里, 引用-1 后发现为0了, 就需要自己释放
    // ? 问题同上, 判断条件中应该为0才对啊
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }
    // 重新设置成员变量
    colsBytes_ = 0;
    rows_ = 0;    
    data_ = 0;    
    step_ = 0;
    refcount_ = 0;
}

// 拷贝数据到目标变量
void DeviceMemory2D::copyTo(DeviceMemory2D& other) const
{
    if (empty())
        // 若当前变量为空, 拷贝完成后目标变量也得为空
        other.release();
    else
    {
        // 不然目标变量就要重新分配显存空间并且复制数据到目标变量
        other.create(rows_, colsBytes_);    
        cudaSafeCall( cudaMemcpy2D(other.data_, other.step_, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToDevice) );
        cudaSafeCall( cudaDeviceSynchronize() );
    }
}

// 将内存中的二维数据上传到显存, 注意需要提供主机端二维数据的 step
void DeviceMemory2D::upload(const void *host_ptr_arg, size_t host_step_arg, int rows_arg, int colsBytes_arg)
{
    create(rows_arg, colsBytes_arg);
    cudaSafeCall( cudaMemcpy2D(data_, step_, host_ptr_arg, host_step_arg, colsBytes_, rows_, cudaMemcpyHostToDevice) );        
}

// 将显存中的二维数据下载到主机端中
void DeviceMemory2D::download(void *host_ptr_arg, size_t host_step_arg) const
{    
    cudaSafeCall( cudaMemcpy2D(host_ptr_arg, host_step_arg, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToHost) );
}      

// 交换当前对象的数据和目标对象的数据
void DeviceMemory2D::swap(DeviceMemory2D& other_arg)
{    
    std::swap(data_, other_arg.data_);
    std::swap(step_, other_arg.step_);

    std::swap(colsBytes_, other_arg.colsBytes_);
    std::swap(rows_, other_arg.rows_);
    std::swap(refcount_, other_arg.refcount_);                 
}

// 是否分配过显存
bool DeviceMemory2D::empty() const { return !data_; }
// 获取每一行的有效数据字节数
int DeviceMemory2D::colsBytes() const { return colsBytes_; }
// 获取行数
int DeviceMemory2D::rows() const { return rows_; }
// 获取每一行的实际字节数
size_t DeviceMemory2D::step() const { return step_; }
