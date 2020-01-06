/**
 * @file ThreadMutexObject.h
 * @author guoqing (1337841346@qq.com)
 * @brief 疑似是用于执行互斥锁相关功能的模板类
 * @version 0.1
 * @date 2020-01-04
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

#ifndef THREADMUTEXOBJECT_H_
#define THREADMUTEXOBJECT_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

/**
 * @brief 使用线程锁封装好的变量类
 * @tparam T 被封装好的变量的类型
 */
template <class T>
class ThreadMutexObject
{
    public:
        /** @brief 默认构造函数 */
        ThreadMutexObject()
        {}

        /**
         * @brief 给定被保护变量初始值的构造函数
         * @param[in] initialValue 初始值
         */
        ThreadMutexObject(T initialValue)
         : object(initialValue),
           lastCopy(initialValue)
        {}

        /**
         * @brief 给这个变量赋值
         * @param[in] newValue 新值
         */
        void assign(T newValue)
        {
            // 上锁 - 赋值 - 解锁
            mutex.lock();
            object = lastCopy = newValue;
            mutex.unlock();
        }

        /**
         * @brief 获取互斥锁
         * @return std::mutex& 保护这个变量的互斥锁
         */
        std::mutex & getMutex()
        {
            return mutex;
        }

        /**
         * @brief 获取被保护变量的引用
         * @note 但是目测这个方法是不安全的, 获取的引用的操作将不再受互斥锁保护
         * @return T& 引用
         */
        T & getReference()
        {
            return object;
        }

        /**
         * @brief 赋值并唤醒关联的其他线程
         * @param[in] newValue 新值
         */
        void assignAndNotifyAll(T newValue)
        {
            mutex.lock();

            object = newValue;

            signal.notify_all();

            mutex.unlock();
        }
        

        /** @brief 唤醒关联的其他线程 */
        void notifyAll()
        {
            mutex.lock();

            signal.notify_all();

            mutex.unlock();
        }

        /** @brief 获取被保护对象的值 */
        T getValue()
        {
            mutex.lock();

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        /**
         * @brief 当前线程进入挂起状态知道被保护的变量更新
         * @return T 被保护变量更新后的值
         */
        T waitForSignal()
        {
            mutex.lock();

            signal.wait(mutex);

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        /**
         * @brief 当前线程挂起一段时间后获取被保护变量的值
         * @param[in] wait 挂起的时间(ms)
         * @return T 被保护变量的值
         */
        T getValueWait(int wait = 33000)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(wait));

            mutex.lock();

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        /**
         * @brief 当前线程挂起一段时间后获取被保护变量的引用
         * @param[in] wait 挂起的时间(ms)
         * @return T 被保护变量的引用
         */
        T & getReferenceWait(int wait = 33000)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(wait));

            mutex.lock();

            lastCopy = object;

            mutex.unlock();

            return lastCopy;
        }

        /**
         * @brief 对被保护的变量进行自增操作
         * @note 但是目测这样只对为整型的被保护变量才有用
         */
        void operator++(int)
        {
            mutex.lock();

            object++;

            mutex.unlock();
        }

    private:
        T object;                               ///< 被线程锁保护的对象
        T lastCopy;                             ///< 用于缓存最近一次访问被保护对象的值

        std::mutex mutex;                       ///< 用于保护这个变量的锁
        std::condition_variable_any signal;     ///< 条件变量, 通知其他线程时使用
};

#endif /* THREADMUTEXOBJECT_H_ */
