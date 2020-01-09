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

#ifndef ELASTICFUSION_H_
#define ELASTICFUSION_H_

#include "Utils/RGBDOdometry.h"
#include "Utils/Resolution.h"
#include "Utils/Intrinsics.h"
#include "Utils/Stopwatch.h"
#include "Shaders/Shaders.h"
#include "Shaders/ComputePack.h"
#include "Shaders/FeedbackBuffer.h"
#include "Shaders/FillIn.h"
#include "Deformation.h"
#include "GlobalModel.h"
#include "IndexMap.h"
#include "Ferns.h"
#include "PoseMatch.h"
#include "Defines.h"

#include <iomanip>
#include <pangolin/gl/glcuda.h>

class ElasticFusion
{
    public:
        EFUSION_API ElasticFusion(const int timeDelta = 200,
                      const int countThresh = 35000,
                      const float errThresh = 5e-05,
                      const float covThresh = 1e-05,
                      const bool closeLoops = true,
                      const bool iclnuim = false,
                      const bool reloc = false,
                      const float photoThresh = 115,
                      const float confidence = 10,
                      const float depthCut = 3,
                      const float icpThresh = 10,
                      const bool fastOdom = false,
                      const float fernThresh = 0.3095,
                      const bool so3 = true,
                      const bool frameToFrameRGB = false,
                      const std::string fileName = "");

        virtual ~ElasticFusion();

        /**
         * Process an rgb/depth map pair
         * @param rgb               unsigned char row major order
         * @param depth             unsigned short z-depth in millimeters, invalid depths are 0
         * @param timestamp         nanoseconds (actually only used for the output poses, not important otherwise)
         * @param inPose            optional input SE3 pose (if provided, we don't attempt to perform tracking)
         * @param weightMultiplier  optional full frame fusion weight  //? 
         * @param bootstrap         if true, use inPose as a pose guess rather than replacement
         */
        EFUSION_API void processFrame(
                          const unsigned char *     rgb,
                          const unsigned short *    depth,
                          const int64_t &           timestamp,
                          const Eigen::Matrix4f *   inPose = 0,
                          const float               weightMultiplier = 1.f,
                          const bool                bootstrap = false);

        /**
         * Predicts the current view of the scene, updates the [vertex/normal/image]Tex() members
         * of the indexMap class
         */
        // TODO 更改这里注释的格式
        EFUSION_API void predict();

        /**
         * This class contains all of the predicted renders
         * @return reference
         */
        EFUSION_API IndexMap & getIndexMap();

        /**
         * This class contains the surfel map
         * @return
         */
        EFUSION_API GlobalModel & getGlobalModel();

        /**
         * This class contains the fern keyframe database
         * @return
         */
        EFUSION_API Ferns & getFerns();

        /**
         * This class contains the local deformation graph
         * @return
         */
        EFUSION_API Deformation & getLocalDeformation();

        /**
         * This is the map of raw input textures (you can display these)
         * @return
         */
        EFUSION_API std::map<std::string, GPUTexture*> & getTextures();

        /**
         * This is the list of deformation constraints
         * @return
         */
        EFUSION_API const std::vector<PoseMatch> & getPoseMatches();

        /**
         * This is the tracking class, if you want access
         * @return
         */
        EFUSION_API const RGBDOdometry & getModelToModel();

        /**
         * The point fusion confidence threshold
         * @return
         */
        EFUSION_API const float & getConfidenceThreshold();

        /**
         * If you set this to true we just do 2.5D RGB-only Lucas-Kanade tracking (no fusion)
         * @param val
         */
        EFUSION_API void setRgbOnly(const bool & val);

        /**
         * Weight for ICP in tracking
         * @param val if 100, only use depth for tracking, if 0, only use RGB. Best value is 10
         */
        EFUSION_API void setIcpWeight(const float & val);

        /**
         * Whether or not to use a pyramid for tracking
         * @param val default is true
         */
        EFUSION_API void setPyramid(const bool & val);

        /**
         * Controls the number of tracking iterations
         * @param val default is false
         */
        EFUSION_API void setFastOdom(const bool & val);

        /**
         * Turns on or off SO(3) alignment bootstrapping
         * @param val
         */
        EFUSION_API void setSo3(const bool & val);

        /**
         * Turns on or off frame to frame tracking for RGB
         * @param val
         */
        EFUSION_API void setFrameToFrameRGB(const bool & val);

        /**
         * Raw data fusion confidence threshold
         * @param val default value is 10, but you can play around with this
         */
        EFUSION_API void setConfidenceThreshold(const float & val);

        /**
         * Threshold for sampling new keyframes
         * @param val default is some magic value, change at your own risk
         */
        EFUSION_API void setFernThresh(const float & val);

        /**
         * Cut raw depth input off at this point
         * @param val default is 3 meters
         */
        EFUSION_API void setDepthCutoff(const float & val);

        /**
         * Returns whether or not the camera is lost, if relocalisation mode is on
         * @return
         */
        EFUSION_API const bool & getLost();

        /**
         * Get the internal clock value of the fusion process
         * @return monotonically increasing integer value (not real-world time)
         */
        EFUSION_API const int & getTick();

        /**
         * Get the time window length for model matching
         * @return
         */
        EFUSION_API const int & getTimeDelta();

        /**
         * Cheat the clock, only useful for multisession/log fast forwarding
         * @param val control time itself!
         */
        EFUSION_API void setTick(const int & val);

        /**
         * Internal maximum depth processed, this is defaulted to 20 (for rescaling depth buffers)
         * @return
         */
        EFUSION_API const float & getMaxDepthProcessed();

        /**
         * The current global camera pose estimate
         * @return SE3 pose
         */
        EFUSION_API const Eigen::Matrix4f & getCurrPose();

        /**
         * The number of local deformations that have occurred
         * @return
         */
        EFUSION_API const int & getDeforms();

        /**
         * The number of global deformations that have occured
         * @return
         */
        EFUSION_API const int & getFernDeforms();

        /**
         * These are the vertex buffers computed from the raw input data
         * @return can be rendered
         */
        EFUSION_API std::map<std::string, FeedbackBuffer*> & getFeedbackBuffers();

        /**
         * Calculate the above for the current frame (only done on the first frame normally)
         */
        EFUSION_API void computeFeedbackBuffers();

        /**
         * Saves out a .ply mesh file of the current model
         */
        EFUSION_API void savePly();

        /**
         * Renders a normalised view of the input raw depth for displaying as an OpenGL texture
         * (this is stored under textures[GPUTexture::DEPTH_NORM]
         * @param minVal minimum depth value to render
         * @param maxVal maximum depth value to render
         */
        EFUSION_API void normaliseDepth(const float & minVal, const float & maxVal);

        //Here be dragons
    private:
        IndexMap indexMap;                      ///< 包含了 Predicted 的图像
        RGBDOdometry frameToModel;
        RGBDOdometry modelToModel;              ///< 实现追踪的类, 用于 model to model 的追踪
        GlobalModel globalModel;                ///< Surfel Map
        FillIn fillIn;
        Ferns ferns;                            ///< 随机蕨数据库对象
        Deformation localDeformation;           ///< Local deformation graph
        Deformation globalDeformation;          ///< Global deformation graph

        const std::string saveFilename;
        std::map<std::string, GPUTexture*> textures;    ///< 原始输入图像的纹理, //? 第一个元素是字符串描述, 第二个元素是GPU纹理对象句柄?
        std::map<std::string, ComputePack*> computePacks;
        std::map<std::string, FeedbackBuffer*> feedbackBuffers;     ///? 存储点云的?

        void createTextures();
        void createCompute();
        void createFeedbackBuffers();

        void filterDepth();
        void metriciseDepth();

        bool denseEnough(const Img<Eigen::Matrix<unsigned char, 3, 1>> & img);

        void processFerns();

        Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);

        Eigen::Matrix4f currPose;           ///< 当前帧相机的位姿

        int tick;                           ///< 当前处理过的帧数, 也是处理的图像的时间戳
        const int timeDelta;                ///< 时间窗口的大小, 时间窗口内更新过的模型部分将会用于参与配准过程
        const int icpCountThresh;
        const float icpErrThresh;
        const float covThresh;

        int deforms;                        ///< Local Loop 触发的 defomration 的次数
        int fernDeforms;                    ///< Global Loop 触发的 defomration 的次数
        const int consSample;
        Resize resize;

        std::vector<PoseMatch> poseMatches;             ///< the list of deformation constraints
        std::vector<Deformation::Constraint> relativeCons;

        std::vector<std::pair<unsigned long long int, Eigen::Matrix4f> > poseGraph;
        std::vector<unsigned long long int> poseLogTimes;

        Img<Eigen::Matrix<unsigned char, 3, 1>> imageBuff;
        Img<Eigen::Vector4f> consBuff;
        Img<unsigned short> timesBuff;

        const bool closeLoops;
        const bool iclnuim;

        const bool reloc;
        bool lost;                          ///< 相机当前是否跟丢的标志
        bool lastFrameRecovery;
        int trackingCount;
        const float maxDepthProcessed;

        bool rgbOnly;                       ///< 是否只使用 2.5D RGB-only Lucas-Kanade tracking // ? 2.5D
        float icpWeight;                    ///< Weight for ICP in tracking, 也就是几何误差和色彩误差相互的权重
        bool pyramid;                       ///< 设置是否使用图像金字塔进行追踪
        bool fastOdom;
        float confidenceThreshold;          ///< The point fusion confidence threshold = Raw data fusion confidence threshold
        float fernThresh;
        bool so3;                           ///< Turns on or off SO(3) alignment bootstrapping
        bool frameToFrameRGB;               ///< Turns on or off frame to frame tracking for RGB
        float depthCutoff;                  ///< 原始图像的深度切断值, 超过这个值的深度信息我们认为不准, 不要了
};

#endif /* ELASTICFUSION_H_ */
