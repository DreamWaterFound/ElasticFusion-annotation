/**
 * @file Stopwatch.h
 * @author guoqing (1337841346@qq.com)
 * @brief 统计模块的运行时间
 * @version 0.1
 * @date 2020-01-07
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

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

// C STL
#include <stdio.h>
#include <stdlib.h>

#ifdef WIN32
#  define far
#  include <Windows.h>
#  include <WinSock2.h>
#  include <stdint.h> // portable: uint64_t   MSVC: __int64 
#  include <time.h>
#  include "WindowsExtras.h"
#else
// 网络通讯相关
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#endif

// C++ STL
#include <string.h>
#include <vector>
#include <string>
#include <iostream>
#include <map>
#ifndef WIN32
#  include <sys/time.h>
#  include <unistd.h>
#endif

// DLL exports
#include "../Defines.h"

// 通过网络发送耗时数据的间隔, 这里设定的是 10ms 
// ? 根据后面的程序中判断单位应该是us, 这里宏名称中的ms写错了吧
#define SEND_INTERVAL_MS 10000

#ifdef WIN32
typedef char stopwatchPacketType;
#else
// 通过网络发送耗时数据的类型定义, 其实就是uint8_t, 单纯的字节型
typedef unsigned char stopwatchPacketType;
#endif

// 如果使能信息的报告
#ifndef DISABLE_STOPWATCH
// 统计 expression 的执行时间(ms), 然后按照 name 保存到模块运行时间统计表中
#define STOPWATCH(name, expression) \
    do \
    { \
        const unsigned long long int startTime = Stopwatch::getInstance().getCurrentSystemTime(); \
        expression \
        const unsigned long long int endTime   = Stopwatch::getInstance().getCurrentSystemTime(); \
        Stopwatch::getInstance().addStopwatchTiming(name, endTime - startTime); \
    } \
    while(false)

// 为什么这些宏的最后要有个 while(false) 呢? 这里是一个编程上的trick, 可以避免出现错误
// ref: [https://www.cnblogs.com/wubugui/p/4247728.html]
//      [https://www.cnblogs.com/chenhuan001/p/5930487.html]
//      [https://blog.csdn.net/this_capslock/article/details/41843371]

// 开始对 name 的模块展开计时过程
#define TICK(name) \
    do \
    { \
        Stopwatch::getInstance().tick(name, Stopwatch::getInstance().getCurrentSystemTime()); \
    } \
    while(false)

// 结束对 name 的模块的计时过程
#define TOCK(name) \
    do \
    { \
        Stopwatch::getInstance().tock(name, Stopwatch::getInstance().getCurrentSystemTime()); \
    } \
    while(false)
#else
// 如果不使能信息的报告
#define STOPWATCH(name, expression) \
    expression

#define TOCK(name) ((void)0)

#define TICK(name) ((void)0)

#endif

/* 数据包:
 *  +4  unsigned char*4         数据包的大小(算当前)
 *  +8  unsigned long long int  signature // ?
 *  +?  字符串描述+\0
 *  +8  float                   耗费的时间
 *  +?  字符串描述+\0
 *  +8  float                   耗费的时间
 * ...
 */


/* @brief 模块运行计时统计的模块 */
class Stopwatch
{
    public:
        /**
         * @brief 获取对象的静态实例引用, 有点单例模式的感觉
         * @return Stopwatch& 
         */
        static Stopwatch & getInstance()
        {
            static Stopwatch instance;
            return instance;
        }

        /**
         * @brief 记录当前模块的运行时间
         * @param[in] name      当前模块的字符串描述
         * @param[in] duration  耗费的时间, 单位us
         */
        void addStopwatchTiming(std::string name, unsigned long long int duration)
        {
            // 如果的确经历了一段时间
            if(duration > 0)
            {
                // 记录, 除 1000 将单位转换成为 ms
                timings[name] = (float)(duration) / 1000.0f;
            }
        }

        /**
         * @brief 设置 //? 表示是否是触发的信号? 
         * @param[in] newSignature 
         */
        void setCustomSignature(unsigned long long int newSignature)
        {
          signature = newSignature;
        }

        /**
         * @brief 获取统计了的所有模块运行的时间信息
         * @return const std::map<std::string, float>& 信息
         */
        const std::map<std::string, float> & getTimings()
        {
            return timings;
        }

        /** @brief 显示所有模块运行时间的信息 */
        void printAll()
        {
            // 遍历
            for(std::map<std::string, float>::const_iterator it = timings.begin(); it != timings.end(); it++)
            {
                std::cout << it->first << ": " << it->second  << "ms" << std::endl;
            }

            std::cout << std::endl;
        }

        /**
         * @brief //? 什么的脉冲+1? 
         * @param[in] name 模块名称
         */
        void pulse(std::string name)
        {
            timings[name] = 1;
        }

        /** @brief 通过网络发送所有模块的运行时间信息 */
        void sendAll()
        {
            // windows 下会调用 ElasticFusion 自己编写的函数, Linux 下会调用 boost 库中的函数
            // ! 第二个参数为 nullptr 可能更安全一些
            // 获取当前系统的时间, us级别; 第一个参数保存时间信息, 第二个参数保存时区信息
            gettimeofday(&clock, 0);

            // 如果距离上次发送信息的时间已经超过了设定的间隔, 那么我们就要重新打包发送模块的耗时数据了; 不然我们就放弃
            if((currentSend = (clock.tv_sec * 1000000 + clock.tv_usec)) - lastSend > SEND_INTERVAL_MS)
            {
                // 要发送的数据包大小
                int size = 0;
                // 将已经保存的模块耗时数据串行化
                stopwatchPacketType * data = serialiseTimings(size);
                // 然后发送, 函数由 socket.h 提供
                sendto( sockfd,                         // 套接字的id
                        data,                           // 要发送的数据区域头指针
                        size,                           // 要发送的数据区域长度
                        0,                              // 发送标志
                        (struct sockaddr *) &servaddr,  // 目的IP地址
                        sizeof(servaddr));              // IP地址的长度(因为不确定是ipv4 or ipv6)

                // 善后: 释放数据区, 更新发送时间 lastSend
                free(data);
                lastSend = currentSend;
            }
        }

        /**
         * @brief 获取当前的系统时间
         * @return unsigned long long int 时间, 单位us
         */
        static unsigned long long int getCurrentSystemTime()
        {
            timeval tv;
            gettimeofday(&tv, 0);
            unsigned long long int time = (unsigned long long int)(tv.tv_sec * 1000000 + tv.tv_usec);
            return time;
        }

        /**
         * @brief 开始某个模块的开始时间
         * @param[in] name      模块的字符串描述
         * @param[in] start     开始时间, 单位us
         */
        void tick(std::string name, unsigned long long int start)
        {
            // 其实就是将开始的时间先暂存在某个模块中; 存入的 us 单位的时间在计算 duration 的时候会进行单位转换
        	tickTimings[name] = start;
        }

        /**
         * @brief 结束对某个模块的计时过程
         * @param[in] name      模块的字符串描述
         * @param[in] end       结束时间, 单位us
         */
        void tock(std::string name, unsigned long long int end)
        {
            // 此时 tickTimings[name] 中暂存的就是该模块的开始时间, 这里计算经过的时间, 并且转换成为ms表示
        	float duration = (float)(end - tickTimings[name]) / 1000.0f;
            // 只有在确实经过了一段时间后才会记录
            if(duration > 0)
            {
                timings[name] = duration;
            }
        }

    private:

        /* @brief 构造函数 */
        Stopwatch()
        {
            // step 1 生成 IP 地址对象和套接字对象, 默认为本机回环IP, 端口为 45454
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;      // IPv4
            servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
            servaddr.sin_port = htons(45454);
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);

            // step 2 获取当前系统的时间
            gettimeofday(&clock, 0);

            // step 3 初始化状态变量, 都设置为当前的系统时间(us)
            signature = clock.tv_sec * 1000000 + clock.tv_usec;
            currentSend = lastSend = clock.tv_sec * 1000000 + clock.tv_usec;
        }

        /** @brief 析构函数, 主要工作就是关闭套接字 */
        virtual ~Stopwatch()
        {
#ifdef WIN32
            closesocket(sockfd);
#else
            close(sockfd);
#endif
        }

        /**
         * @brief 将当前统计的模块耗时数据串行化, 用于进行网络传输
         * @param[out] packetSize           串行化之后的数据的长度(按字节算)
         * @return stopwatchPacketType*     串行化之后的数据区域的头指针
         */
        stopwatchPacketType * serialiseTimings(int & packetSize)
        {
            // step 1 计算数据包的大小
            // 包头的数据
            packetSize = sizeof(int)                        // 数据包本身的大小
                       + sizeof(unsigned long long int);    // signature

            // 遍历之前统计的各个模块的耗时数据中每一条数据所占用的空间
            for(std::map<std::string, float>::const_iterator it = timings.begin(); it != timings.end(); it++)
            {
                // +1 这个是考虑到了字符串最后的 \0
                packetSize += it->first.length() + 1 + sizeof(float);
            }

            // step 2 生成数据包
            // 本质上就是 uint8_t 类型的, 这里转换成为 int* 只是方便暂时处理
            int * dataPacket = (int *)calloc(packetSize, sizeof(unsigned char));

            // 包头前 4 个字节存储了数据包的大小
            dataPacket[0] = packetSize * sizeof(unsigned char);

            // 接着的 8 个字节保存了这个 signature
            *((unsigned long long int *)&dataPacket[1]) = signature;

            // 获取可以开始存放模块耗时数据的指针
            // 后面的这个操作有点骚, 就是按照前面保存的步骤先指针转换成 int 然后++, 再转换成为 ullint 再++, 再转换成为 float 型给后面使用
            float * valuePointer = (float *)&((unsigned long long int *)&dataPacket[1])[1];

            for(std::map<std::string, float>::const_iterator it = timings.begin(); it != timings.end(); it++)
            {
                // 拷贝字符串, 返回的是字符串的尾(\0)后的单元的指针
                valuePointer = (float *)mempcpy(valuePointer, it->first.c_str(), it->first.length() + 1);
                // 存储耗时时间
                *valuePointer++ = it->second;
            }

            return (stopwatchPacketType *)dataPacket;
        }

        timeval clock;                                                  ///< 保存当前系统的时间信息, tv_sec = 秒, tv_usec = us
        long long int currentSend, lastSend;                            ///< 当前和上次发送数据的时间信息, 时间是系统时间, 单位us
        unsigned long long int signature;                               /// ? 8个字节的数据? 好像是某种标记? 初始化的时候被设置成为当前的系统时间(us)
        int sockfd;                                                     ///< 套接字的id
        struct sockaddr_in servaddr;                                    ///< 发送的目标地址
        std::map<std::string, float> timings;                           ///< 每个步骤耗费的时间, 第一个元素为字符串描述, 第二个元素为耗时, 单位ms
        std::map<std::string, unsigned long long int> tickTimings;
};

#endif /* STOPWATCH_H_ */
