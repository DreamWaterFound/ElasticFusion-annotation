/**
 * @file GPUTest.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 通过多次测试确定确定 GPU 中最佳的 Threads/Blocks 的配置
 * @version 0.1
 * @date 2020-01-10
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
 
// ElasticFusion
#include <ElasticFusion.h>
// ?
#include <Utils/RGBDOdometry.h>

// C++ STL
#include <string>
#include <iomanip>
#include <fstream>

// ?
std::ifstream asFile;
std::string directory;      // 保存测试图像所在的文件夹路径
Eigen::Matrix3f K;          // 相机内参矩阵

/**
 * @brief 将指定文件名称的图像加载到纹理中
 * @param[out] image        纹理对象
 * @param[in]  name         输入的文件名称
 */
void loadImage(GPUTexture & image, const std::string & name)
{
    // step 1 生成完整的文件名称
    std::string imageLoc = directory;
    imageLoc.append(name);

    // step 2 读取图像文件
    pangolin::TypedImage img = pangolin::LoadImage(imageLoc);

    // step 3 获取原始图像数据并上传到纹理对象
    Img<Eigen::Matrix<unsigned char, 3, 1>> imageRaw(
            480, 640,                                           // 图像大小
            (Eigen::Matrix<unsigned char, 3, 1> *)img.ptr);     // 图像数据区
    // ! 这一步有点多此一举.. 直接传  (Eigen::Matrix<unsigned char, 3, 1> *)img.ptr 应该是完全相同的效果
    image.texture->Upload(
        imageRaw.data,          // 图像数据区头指针
        GL_RGB,                 // 图像数据格式
        GL_UNSIGNED_BYTE);      // 图像数据中的每个通道的数据
}

/**
 * @brief 加载深度图像到纹理
 * @param[out] depth    目标纹理
 * @param[in]  name     深度图像文件名
 */
void loadDepth(GPUTexture & depth, const std::string & name)
{
    // step 1 组成完整的文件名称
    std::string depthLoc = directory;
    depthLoc.append(name);

    // step 2 加载深度图像, 得到原始数据
    pangolin::TypedImage img = pangolin::LoadImage(depthLoc);
    Img<unsigned short> depthRaw(480, 640, (unsigned short *)img.ptr);

    // step 3 转换深度值, 上传到 GPU 纹理中
    for(unsigned int i = 0; i < 480; i++)
    {
        for(unsigned int j = 0; j < 640; j++)
        {
            // 这里整数除法 5 的原因是, 对于 TUM RGB-D 数据集中的深度图像来说都乘了 5000, 所以这里除 5 之后, 
            // 每个像素的整数值就能够代表这个像素真正的深度值了 (单位 mm )
            depthRaw.at<unsigned short>(i, j) /= 5;
        }
    }
    // 上传
    depth.texture->Upload(depthRaw.data,                // 图像数据区头指针
                          GL_LUMINANCE_INTEGER_EXT,     // 以整数来处理纹理数据
                          GL_UNSIGNED_SHORT);           // 图像数据中的每个通道的数据类型为 unsigned short
}

/**
 * @brief 从给定的深度图像像素计算空间点
 * @param[in] row           像素所在行数
 * @param[in] col           像素所在列数
 * @param[in] depth         深度图像中该像素所对应的深度值
 * @return Eigen::Vector3f  计算得到的顶点坐标
 */
Eigen::Vector3f getVertex(const int row, const int col, const float depth)
{
    // 就是反投影的过程
    return Eigen::Vector3f((col - K(0, 2)) * depth * (1.f / K(0, 0)),
                           (row - K(1, 2)) * depth * (1.f / K(1, 1)),
                           depth);
}

/**
 * @brief 将深度图像加载, 计算顶点和法向, 并且分别上传到 GPU 中的顶点纹理和法向纹理
 * @param[out] vertices     目标顶点纹理
 * @param[out] normals      目标法向纹理
 * @param[in]  name         深度图像文件名
 */
void loadVertices(GPUTexture & vertices, GPUTexture & normals, const std::string & name)
{
    // step 1 加载图像原始数据
    std::string depthLoc = directory;
    depthLoc.append(name);
    pangolin::TypedImage img = pangolin::LoadImage(depthLoc);
    Img<unsigned short> depthRaw(480, 640, (unsigned short *)img.ptr);

    // step 2 计算顶点和法向
    Img<Eigen::Vector4f> verts(480, 640);
    Img<Eigen::Vector4f> norms(480, 640);

    // 暂时保存顶点和法向
    // ? 为什么是四个元
    Eigen::Vector4f point(0, 0, 0, 1);
    Eigen::Vector4f norm(0, 0, 0, 1);

    // 对于图像中不是边缘的像素展开遍历
    for(unsigned int row = 1; row < 480 - 1; row++)
    {
        for(unsigned int col = 1; col < 640 - 1; col++)
        {
            // 当前的点的上下左右的像素(包括自己)都具有深度值的时候
            if(depthRaw.at<unsigned short>(row, col) > 0 &&
               depthRaw.at<unsigned short>(row + 1, col) > 0 &&
               depthRaw.at<unsigned short>(row, col + 1) > 0 &&
               depthRaw.at<unsigned short>(row - 1, col) > 0 &&
               depthRaw.at<unsigned short>(row, col - 1) > 0)
            {
                // 当前像素对应空间点坐标(在当前相机坐标系下), 除 5000 是为了把深度值转换成为以 m 度量
                Eigen::Vector3f actual = getVertex(row, col, depthRaw.at<unsigned short>(row, col) / 5000.f);
                // 右方像素点的顶点坐标
                Eigen::Vector3f fore = getVertex(row, col + 1, depthRaw.at<unsigned short>(row, col + 1) / 5000.f);
                // 图像 u 轴方向上的一个差向量
                Eigen::Vector3f del_x = fore - actual;
                // 上方像素点的顶点坐标
                fore = getVertex(row + 1, col, depthRaw.at<unsigned short>(row + 1, col) / 5000.f);
                // 图像 y 轴方向上的一个差向量
                Eigen::Vector3f del_y = fore - actual;
                // 叉乘, 就可以得到当前遍历到的这个点对应的法向量了
                Eigen::Vector3f normal = (del_x.cross(del_y)).normalized();

                // 另存当前像素点对应的顶点信息和法向信息
                point.head(3) = actual;
                norm.head(3) = normal;
            }
            else
            {
                // 如果上下左右包括自己只要有一个点不具有深度值, 那么就不计算其顶点和法向
                point = Eigen::Vector4f(0, 0, 0, 1);
                norm = Eigen::Vector4f(0, 0, 0, 1);
            }

            // 以图像的形式保存
            verts.at<Eigen::Vector4f>(row, col)(0) = point(0);
            verts.at<Eigen::Vector4f>(row, col)(1) = point(1);
            verts.at<Eigen::Vector4f>(row, col)(2) = point(2);
            verts.at<Eigen::Vector4f>(row, col)(3) = point(3);

            norms.at<Eigen::Vector4f>(row, col)(0) = norm(0);
            norms.at<Eigen::Vector4f>(row, col)(1) = norm(1);
            norms.at<Eigen::Vector4f>(row, col)(2) = norm(2);
            norms.at<Eigen::Vector4f>(row, col)(3) = norm(3);
        }
    }

    // 上传到 GPU 纹理
    vertices.texture->Upload(verts.data, GL_RGBA, GL_FLOAT);
    normals.texture->Upload(norms.data, GL_RGBA, GL_FLOAT);
}

/**
 * @brief 绘制彩色图像
 * @param[in] firstImage    第一张彩色图像的纹理
 * @param[in] secondImage   第二章彩色图像的纹理
 * @param[in] counter       计数器的计数值, 用于控制当前显示哪一张图像
 */
void display(const GPUTexture & firstImage, const GPUTexture & secondImage, const int counter)
{
    // step 1 清空颜色缓冲区和深度缓冲区(其实深度缓冲区这里也没有用到)
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // step 2 根据计数器的计数值决定显示哪一张图像
    pangolin::Display("Image").Activate();
    if(counter % 2 == 0)
        firstImage.texture->RenderToViewport(true);
    else
        secondImage.texture->RenderToViewport(true);

    // step 3 结束绘制
    pangolin::FinishFrame();
    glFinish();
}

/**
 * @brief 主函数
 * @param[in] argc 
 * @param[in] argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
    // step 1 数据准备
    // ?
    Stopwatch::getInstance().setCustomSignature(123412);

    // 使用的图片的相机内参矩阵(粗略)
    K << 528,   0, 320,
           0, 528, 240,
           0,   0,   1;

    // 参数检查
    assert(argc == 2 && "Please supply the folder containing 1c.png, 1d.png, 2c.png and 2d.png");

    // 保存图像所在的文件夹
    directory.append(argv[1]);
    // 确保文件夹的路径最后面有个斜杠, 后面拼接产生完整的文件名要使用
    if(directory.at(directory.size() - 1) != '/')
    {
        directory.append("/");
    }

    // step 2 可视化窗口设置, 加载彩色图像和深度图像的纹理
    // 创建窗口, 只添加一个用于显示图像的视图, 和图像大小(640x480)相同
    pangolin::CreateWindowAndBind("GPUTest", 640, 480);
    pangolin::Display("Image").SetAspect(640.0f / 480.0f);

    // 第一张彩色图像对应的纹理
    GPUTexture firstImage(640, 480,             // 纹理的大小
                          GL_RGBA,              // 纹理的数据格式
                          GL_RGB,               // 如何对待纹理, 这里当做成普通的RGB数据
                          GL_UNSIGNED_BYTE,     // 每个听到的数据类型
                          false,                // ? 是否可以绘制
                          true);                // 使用 CUDA 来协助处理纹理数据
    // 将第一张彩色图像加载到纹理中, 并且上传到GPU
    loadImage(firstImage, "1c.png");

    // 对于第二章图像也要执行相同的操作
    GPUTexture secondImage(640, 480,
                           GL_RGBA,
                           GL_RGB,
                           GL_UNSIGNED_BYTE,
                           false, true);
    loadImage(secondImage, "2c.png");

    // 绘制彩色图像
    display(firstImage, secondImage, 0);

    // 加载第二张深度图像; 第一张深度图像在后面会被用来计算顶点和法向 
    GPUTexture secondDepth(640, 480,
                           GL_LUMINANCE16UI_EXT,        // 指定数据格式为 uint16_t
                           GL_LUMINANCE_INTEGER_EXT,    // 以整数的方式对待纹理数据
                           GL_UNSIGNED_SHORT,           // 每个通道(其实就一个通道)的数据类型是 uint16_t
                           false,
                           true);                       // 同样要在 CUDA 中做处理
    // 加载深度图像并上传到 GPU 纹理
    loadDepth(secondDepth, "2d.png");

    // 使用纹理伪装顶点数据
    GPUTexture vertexTexture(640, 480,
                             GL_RGBA32F,                // RGBA 每个通道都是 32F 类型的数据
                             GL_LUMINANCE,              // 只处理 RGB 通道, 这里应该是把 RGB 通道分别当做顶点的 XYZ 轴坐标进行处理了            
                             GL_FLOAT,                  // 每个通道中都是浮点数数据
                             false, true);

    // 使用纹理伪装法向数据
    GPUTexture normalTexture(640, 480,
                             GL_RGBA32F,
                             GL_LUMINANCE,
                             GL_FLOAT,
                             false, true);
    // 根据第一张深度图, 计算并加载成为顶点数据和法向数据
    loadVertices(vertexTexture, normalTexture, "1d.png");

    // step 3 CUDA 准备
    // 获取默认设备名称(就如 "GTX 1070")
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::string dev(prop.name);
    std::cout << dev << std::endl;

    // step 4 视觉里程计准备 
    // 构造一个使用 RGB-D 数据的视觉里程计对象
    RGBDOdometry odom(640, 480,                             // 图像大小
                      K(0, 2), K(1, 2), K(0, 0), K(1, 1));  // 内参
    
    // 迭代开始的位姿设置为单位阵
    Eigen::Matrix4f currPose = Eigen::Matrix4f::Identity();

    // 因为这些变量中希望存储的变量都是最小值, 所以先放最大值
    // 耗时均值
    float icpStepMeanTime   = std::numeric_limits<float>::max();
    float rgbStepMeanTime   = std::numeric_limits<float>::max();
    float rgbResMeanTime    = std::numeric_limits<float>::max();
    float so3StepMeanTime   = std::numeric_limits<float>::max();

    // 耗时最小值
    float icpStepBestFast   = icpStepMeanTime;
    float rgbStepBestFast   = rgbStepMeanTime;
    float rgbResBestFast    = rgbResMeanTime;
    float so3StepBestFast   = so3StepMeanTime;

    // ? 尝试计数
    int count = 0;
    // CUDA 初始的核函数规模, 这里给的是初始值. 
    // 之所以从 16, 16 开始是因为前面的过程用脑子想想也知道无法利用好 GPU 的并行特性, 线程束中的线程被大量闲置
    int threads = 16;
    int blocks = 16;

    // 用于存储最优值
    int icpStepBestThreads  = threads;
    int rgbStepBestThreads  = threads;
    int rgbResBestThreads   = threads;
    int so3StepBestThreads  = threads;

    int icpStepBestBlocks   = blocks;
    int rgbStepBestBlocks   = blocks;
    int rgbResBestBlocks    = blocks;
    int so3StepBestBlocks   = blocks;

    // step 5 开始尝试
    std::cout << "Searching for the best thread/block configuration for your GPU..." << std::endl;

    float counter = 0;

    // 尝试每一种 thread 和 block 的数量的搭配
    for(threads = 16; threads <= 512 && !pangolin::ShouldQuit(); threads += 16)
    {
        for(blocks = 16; blocks <= 512 && !pangolin::ShouldQuit(); blocks += 16)
        {
            // 清空结果
            icpStepMeanTime = 0.0f;
            rgbStepMeanTime = 0.0f;
            rgbResMeanTime  = 0.0f;
            so3StepMeanTime = 0.0f;

            // ? 表示是在当前的规模下第几次计算
            count = 0;

            // ? 对于每种方案迭代 5 次
            for(int i = 0; i < 5 && !pangolin::ShouldQuit(); i++)
            {
                
                odom.initICPModel(&vertexTexture,   // ? 输入的是当前帧的顶点, 输出的是 predicted 的模型的顶点信息
                                  &normalTexture,   // ? 输入的是当前帧的法向, 输出的是 predicted 的模型的法向信息
                                  20.0f,            // 深度截断值
                                  currPose);        // ? 上一次的位姿迭代误差?

                // ? 继续初始化ICP过程?                                
                odom.initICP(&secondDepth,  // 滤波后的当前帧图像(这里的当前帧可以认为就是第二帧)
                             20.0f);        // 深度截断值

                // ? 获取上一帧迭代后的位移和旋转
                Eigen::Vector3f trans = currPose.topRightCorner(3, 1);
                Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = currPose.topLeftCorner(3, 3);

                // 设置 CUDA 核函数分配策略
                // ? 这四个阶段分别对应着什么?
                GPUConfig::getInstance().icpStepThreads = threads;
                GPUConfig::getInstance().icpStepBlocks = blocks;

                GPUConfig::getInstance().rgbStepThreads = threads;
                GPUConfig::getInstance().rgbStepBlocks = blocks;

                GPUConfig::getInstance().rgbResThreads = threads;
                GPUConfig::getInstance().rgbResBlocks = blocks;

                GPUConfig::getInstance().so3StepThreads = threads;
                GPUConfig::getInstance().so3StepBlocks = blocks;

                // 执行 ICP 配准过程, 获得更新后的相机位姿
                odom.getIncrementalTransformation(
                        trans,  // 输入的时候是上一次迭代的相机平移, 输出是迭代后的相机平移
                        rot,    // 输入的时候是上一次迭代的相机旋转, 输出是迭代后的相机旋转
                        false,  // 使用 RGB-D 的完整数据
                        10,     // RGB 误差占0.1, 对应地几何误差占 0.9
                        false,  // ? 不使用图像金字塔求解
                        false,  // ? 不使用快速求解
                        true);  // ? 使用 SO3 加速求解?  使用这个的目的应该是获得最佳的 so3 加速的时候的 CUDA 核函数规模

                // 一种...不是很合理的时间平均值计算的方法
                icpStepMeanTime = (float(count) * icpStepMeanTime + (Stopwatch::getInstance().getTimings().at("icpStep") * 10)) / float(count + 1);
                rgbStepMeanTime = (float(count) * rgbStepMeanTime + (Stopwatch::getInstance().getTimings().at("rgbStep") * 10)) / float(count + 1);
                rgbResMeanTime  = (float(count) * rgbResMeanTime  + (Stopwatch::getInstance().getTimings().at("computeRgbResidual") * 10)) / float(count + 1);
                so3StepMeanTime = (float(count) * so3StepMeanTime + (Stopwatch::getInstance().getTimings().at("so3Step") * 10)) / float(count + 1);
                // 迭代计数
                count++;
            }

            // 已经尝试的方案计数
            counter++;

            // 更新各种统计数据
            if(icpStepMeanTime < icpStepBestFast)
            {
                icpStepBestFast = icpStepMeanTime;
                icpStepBestThreads = threads;
                icpStepBestBlocks = blocks;
            }

            if(rgbStepMeanTime < rgbStepBestFast)
            {
                rgbStepBestFast = rgbStepMeanTime;
                rgbStepBestThreads = threads;
                rgbStepBestBlocks = blocks;
            }

            if(rgbResMeanTime < rgbResBestFast)
            {
                rgbResBestFast = rgbResMeanTime;
                rgbResBestThreads = threads;
                rgbResBestBlocks = blocks;
            }

            if(so3StepMeanTime < so3StepBestFast)
            {
                so3StepBestFast = so3StepMeanTime;
                so3StepBestThreads = threads;
                so3StepBestBlocks = blocks;
            }

            // 输出
            std::cout << "\rBest: " 
                      << (so3StepBestFast + rgbResBestFast + rgbStepBestFast + icpStepBestFast)     // 当前次迭代得到的最佳的耗时
                      << ", " 
                      << int((counter / 1024.f) * 100.f) << "%    ";                                // 计算 已尝试方案/总方案数 的比例, 以百分数显示
                      
            std::cout.flush();

            // 各模块用时统计并发送
            Stopwatch::getInstance().sendAll();

            // 更新图像显示, 图像每更换一次表示完成了一种方案的尝试
            display(firstImage, secondImage, counter);
        }
    }

    std::cout << std::endl;

    // 迭代结束, 输出最佳结果
    std::cout << "icpStepMap[\"" << dev << "\"] = std::pair<int, int>(" << icpStepBestThreads <<", " << icpStepBestBlocks << ");" << std::endl;
    std::cout << "rgbStepMap[\"" << dev << "\"] = std::pair<int, int>(" << rgbStepBestThreads <<", " << rgbStepBestBlocks << ");" << std::endl;
    std::cout << "rgbResMap[\"" << dev << "\"] = std::pair<int, int>(" << rgbResBestThreads <<", " << rgbResBestBlocks << ");" << std::endl;
    std::cout << "so3StepMap[\"" << dev << "\"] = std::pair<int, int>(" << so3StepBestThreads <<", " << so3StepBestBlocks << ");" << std::endl;
}

