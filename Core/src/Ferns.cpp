/**
 * @file Ferns.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 随机蕨数据库
 * @version 0.1
 * @date 2020-01-15
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

#include "Ferns.h"

// 构造函数, 创建随机蕨数据库对象
Ferns::Ferns(int n,                     // 蕨的棵数
             int maxDepth,              // 深度切断值
             const float photoThresh)   // Global loop closure photometric threshold
 : num(n),                              // 蕨的棵数
   factor(8),                           // 随机蕨采样的图像相对于原图像的缩放系数, 这里直接进行了 1/8 下采样
   width(Resolution::getInstance().width() / factor),       // 采样图像的宽度
   height(Resolution::getInstance().height() / factor),     // 采样图像的高度
   maxDepth(maxDepth),                  // 深度切断值
   photoThresh(photoThresh),            // Global loop closure photometric threshold
   widthDist(0, width - 1),             // 用于随机蕨在宽度维度上进行随机采样的一维均匀分布对象
   heightDist(0, height - 1),           // 用于随机蕨在高度维度上进行随机采样的一维均匀分布对象
   rgbDist(0, 255),                     // 强度值 0-255 的RGB强度均匀分布对象 // ? 有什么用?
   dDist(400, maxDepth),                // 深度值最低从 400 开始, 到切断距离的均匀分布对象 // ? 同样有什么用?
   lastClosest(-1),                     // ? 最近的和当前帧匹配的帧在 frames 中的id?
   badCode(255),                        // ? 表示什么的 Bad code?
   // ? 疑似是在确定几何结构是否配准阶段使用到的视觉里程计对象
   rgbd(Resolution::getInstance().width() / factor,
        Resolution::getInstance().height() / factor,
        Intrinsics::getInstance().cx() / factor,
        Intrinsics::getInstance().cy() / factor,
        Intrinsics::getInstance().fx() / factor,
        Intrinsics::getInstance().fy() / factor),
   // 顶点纹理
   vertFern(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),            // ?
   vertCurrent(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),         // ? 当前帧图像的, 下同
   // 法向纹理
   normFern(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),
   normCurrent(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),
   // 颜色纹理
   colorFern(width, height, GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, false, true),
   colorCurrent(width, height, GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, false, true),
   // 使用GLSL实现的resize对象
   resize(Resolution::getInstance().width(), Resolution::getInstance().height(), width, height),
   imageBuff(width, height),                // 图像缓冲
   vertBuff(width, height),                 // 顶点缓冲
   normBuff(width, height)                  // 法向缓冲
{
    // 设置随机数发生器的种子, 生成随机蕨中的每一棵蕨
    random.seed(time(0));
    generateFerns();
}

Ferns::~Ferns()
{
    for(size_t i = 0; i < frames.size(); i++)
    {
        delete frames.at(i);
    }
}

void Ferns::generateFerns()
{
    // 生成每一棵随机蕨
    for(int i = 0; i < num; i++)
    {
        // 生成一棵蕨 // ? 这每一棵蕨不应该是一对点吗
        Fern f;
        // 随机确定降采样图像中一个点的位置
        f.pos(0)  = widthDist(random);
        f.pos(1)  = heightDist(random);
        // ? 颜色和深度?
        f.rgbd(0) = rgbDist(random);
        f.rgbd(1) = rgbDist(random);
        f.rgbd(2) = rgbDist(random);
        f.rgbd(3) = dDist(random);
        // 保存
        conservatory.push_back(f);
    }
}

bool Ferns::addFrame(GPUTexture * imageTexture, GPUTexture * vertexTexture, GPUTexture * normalTexture, const Eigen::Matrix4f & pose, int srcTime, const float threshold)
{
    Img<Eigen::Matrix<unsigned char, 3, 1>> img(height, width);
    Img<Eigen::Vector4f> verts(height, width);
    Img<Eigen::Vector4f> norms(height, width);

    resize.image(imageTexture, img);
    resize.vertex(vertexTexture, verts);
    resize.vertex(normalTexture, norms);

    Frame * frame = new Frame(num,
                              frames.size(),
                              pose,
                              srcTime,
                              width * height,
                              (unsigned char *)img.data,
                              (Eigen::Vector4f *)verts.data,
                              (Eigen::Vector4f *)norms.data);

    int * coOccurrences = new int[frames.size()];

    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;

        if(verts.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0)
        {
            const Eigen::Matrix<unsigned char, 3, 1> & pix = img.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0));

            code = (pix(0) > conservatory.at(i).rgbd(0)) << 3 |
                   (pix(1) > conservatory.at(i).rgbd(1)) << 2 |
                   (pix(2) > conservatory.at(i).rgbd(2)) << 1 |
                   (int(verts.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;

            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;
            }
        }

        frame->codes[i] = code;
    }

    float minimum = std::numeric_limits<float>::max();

    if(frame->goodCodes > 0)
    {
        for(size_t i = 0; i < frames.size(); i++)
        {
            float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

            float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

            if(dissim < minimum)
            {
                minimum = dissim;
            }
        }
    }

    delete [] coOccurrences;

    if((minimum > threshold || frames.size() == 0) && frame->goodCodes > 0)
    {
        for(int i = 0; i < num; i++)
        {
            if(frame->codes[i] != badCode)
            {
                conservatory.at(i).ids[frame->codes[i]].push_back(frame->id);
            }
        }

        frames.push_back(frame);

        return true;
    }
    else
    {
        delete frame;

        return false;
    }
}

Eigen::Matrix4f Ferns::findFrame(std::vector<SurfaceConstraint> & constraints,
                                 const Eigen::Matrix4f & currPose,
                                 GPUTexture * vertexTexture,
                                 GPUTexture * normalTexture,
                                 GPUTexture * imageTexture,
                                 const int time,
                                 const bool lost)
{
    lastClosest = -1;

    Img<Eigen::Matrix<unsigned char, 3, 1>> imgSmall(height, width);
    Img<Eigen::Vector4f> vertSmall(height, width);
    Img<Eigen::Vector4f> normSmall(height, width);

    resize.image(imageTexture, imgSmall);
    resize.vertex(vertexTexture, vertSmall);
    resize.vertex(normalTexture, normSmall);

    Frame * frame = new Frame(num, 0, Eigen::Matrix4f::Identity(), 0, width * height);

    int * coOccurrences = new int[frames.size()];

    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;

        if(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0)
        {
            const Eigen::Matrix<unsigned char, 3, 1> & pix = imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0));

            code = (pix(0) > conservatory.at(i).rgbd(0)) << 3 |
                   (pix(1) > conservatory.at(i).rgbd(1)) << 2 |
                   (pix(2) > conservatory.at(i).rgbd(2)) << 1 |
                   (int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;

            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;
            }
        }

        frame->codes[i] = code;
    }

    float minimum = std::numeric_limits<float>::max();
    int minId = -1;

    for(size_t i = 0; i < frames.size(); i++)
    {
        float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

        float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

        if(dissim < minimum && time - frames.at(i)->srcTime > 300)
        {
            minimum = dissim;
            minId = i;
        }
    }

    delete [] coOccurrences;

    Eigen::Matrix4f estPose = Eigen::Matrix4f::Identity();

    if(minId != -1 && blockHDAware(frame, frames.at(minId)) > 0.3)
    {
        Eigen::Matrix4f fernPose = frames.at(minId)->pose;

        vertFern.texture->Upload(frames.at(minId)->initVerts, GL_RGBA, GL_FLOAT);
        vertCurrent.texture->Upload(vertSmall.data, GL_RGBA, GL_FLOAT);

        normFern.texture->Upload(frames.at(minId)->initNorms, GL_RGBA, GL_FLOAT);
        normCurrent.texture->Upload(normSmall.data, GL_RGBA, GL_FLOAT);

//        colorFern.texture->Upload(frames.at(minId)->initRgb, GL_RGB, GL_UNSIGNED_BYTE);
//        colorCurrent.texture->Upload(imgSmall.data, GL_RGB, GL_UNSIGNED_BYTE);

        //WARNING initICP* must be called before initRGB*
        rgbd.initICPModel(&vertFern, &normFern, (float)maxDepth / 1000.0f, fernPose);
//        rgbd.initRGBModel(&colorFern);

        rgbd.initICP(&vertCurrent, &normCurrent, (float)maxDepth / 1000.0f);
//        rgbd.initRGB(&colorCurrent);

        Eigen::Vector3f trans = fernPose.topRightCorner(3, 1);
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = fernPose.topLeftCorner(3, 3);

        TICK("fernOdom");
        rgbd.getIncrementalTransformation(trans,
                                          rot,
                                          false,
                                          100,
                                          false,
                                          false,
                                          false);
        TOCK("fernOdom");

        estPose.topRightCorner(3, 1) = trans;
        estPose.topLeftCorner(3, 3) = rot;

        float photoError = photometricCheck(vertSmall, imgSmall, estPose, fernPose, frames.at(minId)->initRgb);

        int icpCountThresh = lost ? 1400 : 2400;

//        std::cout << rgbd.lastICPError << ", " << rgbd.lastICPCount << ", " << photoError << std::endl;

        if(rgbd.lastICPError < 0.0003 && rgbd.lastICPCount > icpCountThresh && photoError < photoThresh)
        {
            lastClosest = minId;

            for(int i = 0; i < num; i += num / 50)
            {
                if(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0 &&
                   int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) < maxDepth)
                {
                    Eigen::Vector4f worldRawPoint = currPose * Eigen::Vector4f(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                                                                               vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                                                                               vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                                                                               1.0f);

                    Eigen::Vector4f worldModelPoint = estPose * Eigen::Vector4f(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                                                                                vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                                                                                vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                                                                                1.0f);

                    constraints.push_back(SurfaceConstraint(worldRawPoint, worldModelPoint));
                }
            }
        }
    }

    delete frame;

    return estPose;
}

float Ferns::photometricCheck(const Img<Eigen::Vector4f> & vertSmall,
                              const Img<Eigen::Matrix<unsigned char, 3, 1>> & imgSmall,
                              const Eigen::Matrix4f & estPose,
                              const Eigen::Matrix4f & fernPose,
                              const unsigned char * fernRgb)
{
    float cx = Intrinsics::getInstance().cx() / factor;
    float cy = Intrinsics::getInstance().cy() / factor;
    float invfx = 1.0f / float(Intrinsics::getInstance().fx() / factor);
    float invfy = 1.0f / float(Intrinsics::getInstance().fy() / factor);

    Img<Eigen::Matrix<unsigned char, 3, 1>> imgFern(height, width, (Eigen::Matrix<unsigned char, 3, 1> *)fernRgb);

    float photoSum = 0;
    int photoCount = 0;

    for(int i = 0; i < num; i++)
    {
        if(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0 &&
           int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) < maxDepth)
        {
            Eigen::Vector4f vertPoint = Eigen::Vector4f(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                                                        vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                                                        vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                                                        1.0f);

            Eigen::Matrix4f diff = fernPose.inverse() * estPose;

            Eigen::Vector4f worldCorrPoint = diff * vertPoint;

            Eigen::Vector2i correspondence((worldCorrPoint(0) * (1/invfx) / worldCorrPoint(2) + cx), (worldCorrPoint(1) * (1/invfy) / worldCorrPoint(2) + cy));

            if(correspondence(0) >= 0 && correspondence(1) >= 0 && correspondence(0) < width && correspondence(1) < height &&
               (imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(0) > 0 ||
                imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(1) > 0 ||
                imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(2) > 0))
            {
                photoSum += abs((int)imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(0) - (int)imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0));
                photoSum += abs((int)imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(1) - (int)imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1));
                photoSum += abs((int)imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(2) - (int)imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2));
                photoCount++;
            }
        }
    }

    return photoSum / float(photoCount);
}

float Ferns::blockHD(const Frame * f1, const Frame * f2)
{
    float sum = 0.0f;

    for(int i = 0; i < num; i++)
    {
        sum += f1->codes[i] == f2->codes[i];
    }

    sum /= (float)num;

    return sum;
}

float Ferns::blockHDAware(const Frame * f1, const Frame * f2)
{
    int count = 0;
    float val = 0;

    for(int i = 0; i < num; i++)
    {
        if(f1->codes[i] != badCode && f2->codes[i] != badCode)
        {
            count++;

            if(f1->codes[i] == f2->codes[i])
            {
                val += 1.0f;
            }
        }
    }

    return val / (float)count;
}
