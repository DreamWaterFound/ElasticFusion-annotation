/**
 * @file MainController.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 实现 GUI 程序的主要功能
 * @version 0.1
 * @date 2020-01-03
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
 
#include "MainController.h"


// 构造函数
MainController::MainController(int argc, char * argv[])
 : 
   good(true),                      // 是否正确初始化
   // ! 其实下面的指针初始值设置为 nullptr 比较好
   eFusion(0),                      // EasticFusion Core 对象指针
   gui(0),                          // GUI 窗口对象指针
   groundTruthOdometry(0),          // 处理位姿轨迹真值的对象(当然位姿真值从外部输入的)
   logReader(0),                    // 数据源
   framesToSkip(0),                 // 清除因为"仿真实时性"而需要跳帧的个数
   resetButton(false),              // 复位按钮是否被按下
   resizeStream(0)                  // Resize 器, 借助 GPU Shader 实现
{
    // step 0 解析命令行参数
    // 这个变量用于缓存后面多个参数的内容
    std::string empty;
    // 如果找到了, 返回值>=0; 反之找不到. 这个标志是读取 ICL-NUIM 数据集的时候使用的
    iclnuim = Parse::get().arg(argc, argv, "-icl", empty) > -1;

    // 解析相机参数文件
    std::string calibrationFile;
    Parse::get().arg(argc, argv, "-cal", calibrationFile);

    // 设定分辨率对象, 这个分辨率和 Kinect1 的分辨率是相同的
    Resolution::getInstance(640, 480);

    if(calibrationFile.length())
    {
        // 如果命令行参数中的确指定要使用的相机内参
        loadCalibration(calibrationFile);
    }
    else
    {
        // 如果命令行参数中没有指定相机内参文件, 那么就使用默认内参
        Intrinsics::getInstance(528, 528, 320, 240);
    }

    // step 1 查看是否给定要使用录制的文件, 从而决定使用已经录制的文件还是使用实时的相机作为数据源
    Parse::get().arg(argc, argv, "-l", logFile);

    if(logFile.length())
    {
        // 如果要使用录制的文件, 就要生成记录文件读取器
        logReader = new RawLogReader(
            logFile,                                            // 记录文件的路径
            Parse::get().arg(argc, argv, "-f", empty) > -1);    // 是否要左右翻转得到的图像, 这里只需要知道是否存在这个参数标志就可以了
    }
    else
    {
        // 如果使用实际的传感器, 那么就生成所谓的"实时记录读取器", 其实就是获取相机实时图像的接口
        bool flipColors = Parse::get().arg(argc,argv,"-f",empty) > -1;
        logReader = new LiveLogReader(
            logFile,                // 此时这个记录文件路径为空啊, 并没有起到什么作用
            flipColors,             // 是否翻转图像
            LiveLogReader::CameraType::OpenNI2); // 使用的相机类型

        // 是否已经正确初始化数据源
        // ! 其实这里还有一种情况是 cam 为 nullptr, 下面的这个操作可能会触发段错误
        good = ((LiveLogReader *)logReader)->cam->ok();

        // 如果要使用Realsense作输入
#ifdef WITH_REALSENSE
        if(!good)
        {
          delete logReader;
          logReader = new LiveLogReader(logFile, flipColors, LiveLogReader::CameraType::RealSense);

          good = ((LiveLogReader *)logReader)->cam->ok();
        }
#endif
    }

    // step 2 查看是否给出了真值文件
    if(Parse::get().arg(argc, argv, "-p", poseFile) > 0)
    {
        // 那么就从外部文件中加载相机的运动轨迹
        groundTruthOdometry = new GroundTruthOdometry(poseFile);
    }

    // step 3 初始化参数
    confidence      = 10.0f;                // Surfel confidence threshold
    depth           = 3.0f;                 // 深度切断值
    icp             = 10.0f;                // Relative ICP/RGB tracking weight
    icpErrThresh    = 5e-05;                // Local loop closure residual threshold
    covThresh       = 1e-05;                // Local loop closure covariance threshold
    photoThresh     = 115;                  // Global loop closure photometric threshold
    fernThresh      = 0.3095f;              // Fern encoding threshold

    timeDelta       = 200;                  // Time window length
    icpCountThresh  = 40000;                // Local loop closure inlier threshold
    start           = 1;                    // Frames to skip at start of log, 默认就是从第一帧开始处理

    // 是否命令行参数中指定了 -nso , 如果指定了结果为 false; Disables SO(3) pre-alignment in tracking
    so3 = !(Parse::get().arg(argc, argv, "-nso", empty) > -1);
    // 当前 ElasticFusion 能够处理的帧数的上界
    end = std::numeric_limits<unsigned short>::max(); // Funny bound, since we predict times in this format really!

    // 从命令行参数中更新这些数值
    Parse::get().arg(argc, argv, "-c", confidence);         // Surfel confidence threshold
    Parse::get().arg(argc, argv, "-d", depth);              // 深度切断值
    Parse::get().arg(argc, argv, "-i", icp);                // Relative ICP/RGB tracking weight
    Parse::get().arg(argc, argv, "-ie", icpErrThresh);      // Local loop closure residual threshold
    Parse::get().arg(argc, argv, "-cv", covThresh);         // Local loop closure covariance threshold // ? 不知道这个协方差是咋定义的
    Parse::get().arg(argc, argv, "-pt", photoThresh);       // Global loop closure photometric threshold
    Parse::get().arg(argc, argv, "-ft", fernThresh);        // Fern encoding threshold
    Parse::get().arg(argc, argv, "-t", timeDelta);          // Time window length
    Parse::get().arg(argc, argv, "-ic", icpCountThresh);    // Local loop closure inlier threshold
    Parse::get().arg(argc, argv, "-s", start);              // Frames to skip at start of log
    Parse::get().arg(argc, argv, "-e", end);                // Cut off frame of log 指定处理的帧的最大id

    // Flip RGB/BGR
    logReader->flipColors = Parse::get().arg(argc, argv, "-f", empty) > -1;

    
    // ! 有个问题, 如果 groundTruthOdometry 并未初始化, 则这个指针可能不是 nullptr
    openLoop        = !groundTruthOdometry && Parse::get().arg(argc, argv, "-o", empty) > -1;       // Open loop mode, 不使用闭环
    reloc           = Parse::get().arg(argc, argv, "-rl", empty) > -1;                              // Enable relocalisation
    frameskip       = Parse::get().arg(argc, argv, "-fs", empty) > -1;                              // Frame skip if processing a log to simulate real-time
    quiet           = Parse::get().arg(argc, argv, "-q", empty) > -1;                               // Quit when finished a log
    fastOdom        = Parse::get().arg(argc, argv, "-fo", empty) > -1;                              // Fast odometry (single level pyramid)
    rewind          = Parse::get().arg(argc, argv, "-r", empty) > -1;                               // Rewind and loop log forever 对于数据记录文件是否循环读取(播放)
    frameToFrameRGB = Parse::get().arg(argc, argv, "-ftf", empty) > -1;                             // Do frame-to-frame RGB tracking

    // step 4 初始化 GUI 界面
    gui = new GUI(
        logFile.length() == 0,                              // 是否指定了记录文件路径, 没有指定则为 true
        Parse::get().arg(argc, argv, "-sc", empty) > -1);   // MapViewer部分全屏(即全屏展示构建的模型, 完全没有其他的东西)

    // 设置 Viewer 中的一些变量
    gui->flipColors->Ref().Set(logReader->flipColors);      // 彩色通道翻转
    gui->rgbOnly->Ref().Set(false);                         // 是否仅使用 2.5D RGB-only Lucas-Kanade tracking // ? 2.5D?
    gui->pyramid->Ref().Set(true);                          // 使用图像金字塔
    gui->fastOdom->Ref().Set(fastOdom);                     // 使用快速视觉里程计模式()
    gui->confidenceThreshold->Ref().Set(confidence);        // 设置 Surfel confidence threshold
    gui->depthCutoff->Ref().Set(depth);                     // 深度切断\值
    gui->icpWeight->Ref().Set(icp);                         // 设置 Relative ICP/RGB tracking weight
    gui->so3->Ref().Set(so3);                               // 是否 Disables SO(3) pre-alignment in tracking
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);       // 是否使用 frame-to-frame RGB tracking

    // 生成用于将图像从原始图像大小降采样一般的图像缩放对象
    resizeStream = new Resize(Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              Resolution::getInstance().width() / 2,
                              Resolution::getInstance().height() / 2);
}

// 析构函数, 就是释放各个 new 出来的资源
MainController::~MainController()
{
    if(eFusion)
    {
        delete eFusion;
    }

    if(gui)
    {
        delete gui;
    }

    if(groundTruthOdometry)
    {
        delete groundTruthOdometry;
    }

    if(logReader)
    {
        delete logReader;
    }

    if(resizeStream)
    {
        delete resizeStream;
    }
}

// 加载指定文件中的相机内参信息
void MainController::loadCalibration(const std::string & filename)
{
    // 输入文件流对象
    std::ifstream file(filename);
    // 保存每一行的信息
    std::string line;

    // 确保不是空文件
    assert(!file.eof());

    double fx, fy, cx, cy;

    // 读取内参
    std::getline(file, line);

    int n = sscanf(line.c_str(), "%lg %lg %lg %lg", &fx, &fy, &cx, &cy);

    // 严格检查
    assert(n == 4 && "Ooops, your calibration file should contain a single line with fx fy cx cy!");

    // 构造内参对象 静态类对象, 生成一次, 到处均可访问
    Intrinsics::getInstance(fx, fy, cx, cy);
}

// 运行
void MainController::launch()
{
    // 只在前面的准备工作都成功的时候才会执行.
    while(good)
    {
        // 如果已经创建了 ElasticFusion 对象， 那么那就正常运行吧
        if(eFusion)
        {
            run();
        }

        // 如果没有创建 ElasticFusion 对象， 或者是复位按钮被按下
        if(eFusion == 0 || resetButton)
        {
            // 复位标志复位
            resetButton = false;

            // 如果已经构造了 efusion, 那么我们就删除再重新建立它
            if(eFusion)
            {
                delete eFusion;
            }

            // 重新读取记录文件, 如果是跑实时的摄像头呢下面的函数是空的
            logReader->rewind();
            // 重新构建 ElasticFusion
            eFusion = new ElasticFusion(
                openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,     // 如果不使用闭环就没有必要设置时间窗口的长度 
                                                                                // ? 但是这里除2是为什么?
                icpCountThresh,                                                 // Local loop closure inlier threshold
                icpErrThresh,                                                   // Local loop closure residual threshold
                covThresh,                                                      // Local loop closure covariance threshold
                !openLoop,                                                      // 使用闭环(true) 不使用闭环(false) // ? 指的是全局闭环还是局部闭环?
                iclnuim,                                                        // 是否使用 ICL-NUIM 数据集
                reloc,                                                          // 是否使能了重定位
                photoThresh,                                                    // Global loop closure photometric threshold
                confidence,                                                     // Surfel confidence threshold
                depth,                                                          // 深度切断值
                icp,                                                            // Relative ICP/RGB tracking weight
                fastOdom,                                                       // 是否工作于纯里程计模式
                fernThresh,                                                     // Fern encoding threshold
                so3,                                                            // Disables SO(3) pre-alignment in tracking
                frameToFrameRGB,                                                // 是否 Do frame-to-frame RGB tracking
                logReader->getFile());                                          // 记录文件的位置, 对于实时摄像头来说是 basedir/live
        }
        else
        {
            // 运行到这里, 说明 eFusion!=0 并且 resetButton = false. 只能够说明是用户退出了
            break;
        }

    }
}

// 主循环, 实现给 ElasticFusion Core 喂图像的工作
void MainController::run()
{
    // Main loop
    while(!pangolin::ShouldQuit()                       // 终止条件1: Pangolin 窗口收到了退出信息 (比如按下ESC)
          && !((!logReader->hasMore()) && quiet)        // 终止条件2: 记录文件已经读取完成了, 并且也设置了处理完毕后就退出
          && !(eFusion->getTick() == end && quiet))     // 终止条件3: 如果设定的处理帧数已经到了并且设置了处理完毕后就退出
    {
        // step 1 喂图像
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            // 如果没有暂停, 并且也没有处于步进模式的话
            if((logReader->hasMore() || rewind)         // 喂图像条件1: 如果记录文件中还有数据(对live的相机来说一直成立), 如果没有数据的话, 之前设置了数据记录文件循环读取也行
                && eFusion->getTick() < end)            // 喂图像条件2: 如果 ElasticFusion 处理的图像数量在预想的范围内
            {
                // 开始统计 日志文件读取(LogRead) 所耗费的时间
                TICK("LogRead");

                // step 1.1 加载图像
                if(rewind)
                {
                    // 如果设置了数据记录文件要反复读取
                    if(!logReader->hasMore())
                    {
                        // 记录文件正序已经读取完成, 那么就倒着读取上一帧
                        // 由于读取的时候 logReader->currentFrame 一直在增加, 所以 logReader->hasMore() 在倒序读取阶段将会一直false
                        logReader->getBack();
                    }
                    else
                    {
                        // 从记录文件中读取下一帧的图像
                        logReader->getNext();
                    }

                    // 是否记录文件的倒序播放也已经完成
                    if(logReader->rewound())
                    {
                        // 重置读取帧的累计计数
                        logReader->currentFrame = 0;
                    }
                }
                else
                {
                    // 如果没有规定循环读取, 那么我们就老老实实的读取下一帧图像数据吧
                    logReader->getNext();
                }
                // 停止加载图像部分的计时
                TOCK("LogRead");

                // step 1.2 跳帧 - 命令行指定导致的, 或者是由于实时性仿真要求的
                // 如果当前还没有到要开始处理的帧数(由命令行参数指定)
                if(eFusion->getTick() < start)
                {
                    // 那么就"快进"到这个帧数
                    eFusion->setTick(start);
                    logReader->fastForward(start);
                }

                // ? 目测是因为实时性仿真要求导致的跳帧, 进而导致的权重累乘倍数. 就算是没有跳帧, 这里也是1, 也就是基准
                float weightMultiplier = framesToSkip + 1;

                // 如果因为当前系统处理较慢, 按照仿真实时处理有需要跳过的帧
                if(framesToSkip > 0)
                {
                    // 跳帧!
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    // 清空, 因为跳帧之后当前系统已经按照"模拟实时处理"的要求, 处理这个时候应该处理的图像了
                    framesToSkip = 0;
                }

                // step 1.3 相机位姿真值的处理
                // ! 还是写成空指针 nullptr 的形式比较好吧
                Eigen::Matrix4f * currentPose = 0;

                if(groundTruthOdometry)
                {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    // 通过时间戳, 获取相应帧的位姿真值
                    *currentPose = groundTruthOdometry->getTransformation(logReader->timestamp);
                }

                // step 1.4 给劳资喂!
                eFusion->processFrame(
                    logReader->rgb,         // 彩色图像缓冲区
                    logReader->depth,       // 深度图像缓冲区, 单位mm
                    logReader->timestamp,   // 当前帧的时间戳, 单位为纳秒 (ns)
                    currentPose,            // GT 位姿变换矩阵的指针, 为空的时候表示 EF 执行追踪过程, 否则直接使用相机位姿的真值
                    weightMultiplier);      // 因为跳帧导致的权重累乘倍数

                // 及时释放
                if(currentPose)
                {
                    delete currentPose;
                }

                // step 1.5 如果判断是否需要跳帧
                // std::map 的 .at() 用法
                // 判断 RUN 模块(就是上面的eFusion->processFrame()函数的执行时间)的执行时间是否超过了帧间的时间, 如果超过了就要计算需要跳帧的帧数
                // ! 这里的帧数和相机接口或者是记录文件中的也不一致啊
                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            } // 如果可以喂图像
        } // 如果没有暂停, 并且也没有处于步进模式的话
        else
        {
            // 如果处于暂停或者是步进模式下, 那么就只 predict 模型的图像
            eFusion->predict();
        }

        // step 2 GUI 绘制更新

        // 记录 GUI 绘制的时间
        TICK("GUI");
        
        // step 2.1 如果视角跟随了当前的相机位姿
        if(gui->followPose->Get())
        {
            // TODO 这里的注释可以再润色一下, 有点不知道该怎么说
            /* 如果选择了使用相机当前的位姿进行观测, 那么我们的任务就是在这里计算虚拟观测相机应该从什么视角和位置去观测, 渲染已经构建的模型
             * 思考这样的一个问题, 我们有一个物体的模型, 知道了物体的位姿 Two=[Rwo, two;0^T, 1](下标o表示object, w表示world),  怎么在世界坐标下绘制呢?
             * 处理办法一般是将绘制的模型先旋转,再平移, 对于绘制的模型上的一个点, 在其自身坐标系的坐标为 Po, 在世界坐标系下的坐标为 Pw, 那么
             * 这个过程就是求解模型上每个点在世界坐标系下的过程:
             *      Pw = Rwo*Po + two
             * 对于这里,我们的问题是相反的. 我们希望虚拟观察摄像机以位姿Twv去对构建的模型去观测, 就需要对!!坐标系本身!!进行变换, 即将观察视角从世界坐标系的位姿转换到
             * 虚拟观测相机坐标系的位姿; 如果记录一个点在世界坐标系下的坐标为Pw, 在虚拟观察相机坐标系下的坐标为Pv(v=virtual), 那么上述这个过程会带来Pw=>Pv的变化
             * 有:
             *      Pw = Rwv*Pv + twv
             * 如果变换形式, 把 Pv 挪到等式左侧:
             *      Pv = (Rwv^-1)*Pw - ((Rwv^-1)*twv) => R_ * Pw + t_
             * 这里的 R_ 和 t_ 就是将观察位姿从世界坐标系原点移动所需要发生的位姿变换.
             * 
             * 对于旋转矩阵R, 其三个列向量代表了旋转后物体自身坐标系的坐标轴的方向向量, 在旋转前物体自身坐标系下的坐标(有点绕). 如果把 Rwv 写成列向量的形式, 会发现:
             * |           |   | 1 |   |   |
             * | x , y , z | * | 0 | = | x |
             * |           |   | 0 |   |   |
             * 
             * |           |   | 0 |   |   |
             * | x , y , z | * | 1 | = | y |
             * |           |   | 0 |   |   |
             * 
             * |           |   | 0 |   |   |
             * | x , y , z | * | 0 | = | z |
             * |           |   | 1 |   |   |
             * 
             * 这里的列向量 x y z 就是旋转矩阵 R 要转换到的坐标系的坐标轴方向向量
             * 
             * 根据上面, 有: 
             *                                | -x- |
             *      R_ = (Rwv^-1) = (Rwv^T) = | -y- |  (原先的列向量变为了行向量)
             *                                | -z- |
             * 
             *                              | -x- |   | twv.x |   | x.*twv |
             *      t_ = -((Rwv^-1)*twv) =  | -y- | * | twv.y | = | y.*twv | (变成了数量积的形式)
             *                              | -z- |   | twv.z |   | z.*twv |
             */ 

            pangolin::OpenGlMatrix mv;

            // 获取位姿, 构造旋转
            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);
            // 转换成为四元数的目的是减少后面过程的计算量
            Eigen::Quaternionf currQuat(currRot);
            
            // 相机坐标系下指向正前方和正上方的方向向量; 另外的一个轴可以通过叉乘计算得到
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);   // -1 是因为OpenGL中的y轴正方向向下

            // 上述方向向量转换到世界坐标系下的坐标表示
            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            // 构造平移向量
            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));
            // eye 表示虚拟观测相机中心到OpenGL世界坐标系原点的距离, 由于我们希望在原始相机的后面退一点点观看, 所以减去了个 forward 向量作为新的虚拟相机观测地点
            eye -= forward;

            // 然后又加上去了... 变成了之前的 eye , 也就是相机所在的位置
            Eigen::Vector3f at = eye + forward;

            // 计算在虚拟相机应该处的位姿, 这个时候虚拟相机坐标系的三个坐标轴的方向
            Eigen::Vector3f z = (eye - at).normalized();  // Forward 其实和下面是完全等效的
            // Eigen::Vector3f z = (-forward).normalized();  
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);               // 叉乘出来

            Eigen::Matrix4d m;
            // 行优先, 也就是存进去之后其实和下面代码列写的形式是一样的(而不是输出之后反而成为了转置的形式)
            m << x(0),  x(1),  x(2),  -(x.dot(eye)),
                 y(0),  y(1),  y(2),  -(y.dot(eye)),
                 z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            // 将矩阵中的数据复制到OpenGL的位姿矩阵中. 注意Eigen中矩阵存储是按列存储的, 这和OpenGL中的矩阵存储方式一致
            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            // 设置渲染器的观察视图
            gui->s_cam.SetModelViewMatrix(mv);
        }

        // step 2.2 进行绘制 MapViewer 的准备工作, 主要是清空各种缓冲区, 并且激活模型的绘制区域
        gui->preCall();


        // step 2.3 更新当前 ElasticFusion 的 ICP 状态
        // 更新 ICP 过程中的 inlier 数
        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        // 更新 ICP 过程最后的残差量
        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        // 当没有暂停键按下的时候
        if(!gui->pause->Get())
        {
            gui->resLog.Log(
                (std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), // 当前ICP误差
                icpErrThresh);      // Local loop closure residual threshold
            gui->inLog.Log(
                eFusion->getModelToModel().lastICPCount,    // 内点的个数
                icpCountThresh);    // Local loop closure inlier threshold
        }

        // step 2.4 点云的绘制, 这里的 raw 和 filtered 的点云是可以同时显示的
        Eigen::Matrix4f pose = eFusion->getCurrPose();

        // 如果需要绘制当前帧点云信息
        if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())
        {
            // 计算原始点云和滤波后的点云(对, 同时计算, 不管你是否只选中了其中的一个)
            // Calculate the above for the current frame (only done on the first frame normally)
            eFusion->computeFeedbackBuffers();
        }

        // 绘制原始的点云
        if(gui->drawRawCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(
                gui->s_cam.GetProjectionModelViewMatrix(),      // 渲染的时候相机所在的位姿
                pose,                                           // 当前相机的位姿
                gui->drawNormals->Get(),                        // 是否使用法向贴图
                gui->drawColors->Get());                        // 是否使用 RGB 纹理贴图
        }

        // 绘制经过滤波之后的点云
        if(gui->drawFilteredCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        // step 2.5 Global 模型绘制, 这里 FXAA 的优先级高于普通的绘制方式
        if(gui->drawGlobalModel->Get())
        {
            // 将 OpenGL 名列队列中的内容先送给显卡执行, 显卡绘制完成后该函数退出
            // ref: [https://blog.csdn.net/stilling2006/article/details/5549306]
            glFinish();
            TICK("Global");

            // 如果启用快速抗锯齿
            if(gui->drawFxaa->Get())
            {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),        // 获取模型显示视图的投影矩阵
                              gui->s_cam.GetModelViewMatrix(),                  // 获取虚拟观察相机以何种位姿观测, view matrix
                              eFusion->getGlobalModel().model(),                // 获取 ElasticFusion 建立的 Global Model
                              eFusion->getConfidenceThreshold(),                // The point fusion confidence threshold //? 为什么这两个函数都要使用这个东西?
                              eFusion->getTick(),                               // 当前 ElasticFusion 已经处理过的帧数
                              eFusion->getTimeDelta(),                          // 时间窗口的长度
                              iclnuim);                                         // 是否使用 ICL-NUIM 数据集
            }
            else
            {
                // 不然就正常绘制
                eFusion->getGlobalModel().renderPointCloud(                     
                            gui->s_cam.GetProjectionModelViewMatrix(),          // 获取虚拟观察相机以何种位姿观测, view matrix
                            eFusion->getConfidenceThreshold(),                  // The point fusion confidence threshold
                            gui->drawUnstable->Get(),                           // 是否绘制 unstable 的点
                            gui->drawNormals->Get(),                            // 是否法线贴图
                            gui->drawColors->Get(),                             // 是否 RGB 纹理贴图
                            gui->drawPoints->Get(),                             // 是否以点云而不是以 Surfel 的方式绘图
                            gui->drawWindow->Get(),                             // 是否绘制时间窗口
                            gui->drawTimes->Get(),                              // 是否按照面元的创建时间进行着色(但是当使用点的显示方式时不会有效)
                            eFusion->getTick(),                                 // 当前 ElasticFusion 已经处理过的帧数
                            eFusion->getTimeDelta());                           // 时间窗口的长度
            }

            // 送GPU进行绘图
            glFinish();
            TOCK("Global");
        }

        // step 2.6 绘制相机位姿的表示
        if(eFusion->getLost())
        {
            // 跟丢了, 显示成为黄色 --  但是只有在命令行中启用重定位才会处于这种状态
            glColor3f(1, 1, 0);
        }
        else
        {
            // 正常没有跟丢的状态, 则显示成为洋红
            glColor3f(1, 0, 1);
        }
        // 绘制相机的视锥
        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        // step 2.7 绘制 Fern 
        if(gui->drawFerns->Get())
        {
            // 绘制成为黑色
            glColor3f(0, 0, 0);
            // 遍历随机蕨数据库中的所有帧
            for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
            {
                // ? 原则上每次遍历应该只有一帧才会被显示, 但是为什么我亲自实验的时候却几乎是所有的帧都显示了呢?
                if((int)i == eFusion->getFerns().lastClosest)
                    continue;
                // 只绘制匹配上的那一帧的帧
                gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
            }
            glColor3f(1, 1, 1);
        }

        // step 2.7 绘制 Deformation Graph
        if(gui->drawDefGraph->Get())
        {
            // 获取并绘制每一个 Deformation Graph 的 Nodes
            const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();
            for(size_t i = 0; i < graph.size(); i++)
            {
                // Node 本身, 绘制交点
                pangolin::glDrawCross(graph.at(i)->position(0),     // 位置
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);                         // 大小
                // 绘制每个 Node 相邻的边
                for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                {
                    pangolin::glDrawLine(graph.at(i)->position(0),                                  // 第一个点的位置
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),      // 第二个点的位置
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                } // 绘制每个 Node 相邻的边
            } // 绘制每个 Node
        }

        // step 2.8 无论是否指定绘制随机蕨数据库中的帧位置, 都要绘制和当前帧具有最佳匹配的帧的位姿
        if(eFusion->getFerns().lastClosest != -1)
        {
            glColor3f(1, 0, 0);
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        // step 2.9 绘制 deformation 约束, 可以理解为经过 Local Loop 和 Global Loop 之后的约束
        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();
        // 统计所有的 PoseMatch 中距离最大的那个 // ! 但是下面的这个 maxDiff 其实没有用到
        int maxDiff = 0;
        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
            {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            // ! 这个 if 判断感觉放在 for 循环外面更好
            if(gui->drawDeforms->Get())
            {
                // 根据来源的不同进行着色
                if(poseMatches.at(i).fern)
                {
                    // Gloabl Loop - 红色
                    glColor3f(1, 0, 0);
                }
                else
                {
                    // Local Loop - 绿色
                    glColor3f(0, 1, 0);
                }

                // 绘制其中的所有源点和目标点的约束关系
                for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        // step 2.10 绘制和显示小的View的图像(//? 只有输入的两个吧)
        // 首先绘制裁剪过后的深度图像
        eFusion->normaliseDepth(0.3f,                       // 显示的深度图中的最小值
                                gui->depthCutoff->Get());   // 显示的深度图中的最大值
        // 显示这些纹理
        for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
        {
            // 如果这个纹理可显示
            if(it->second->draw)
            {
                gui->displayImg(it->first, it->second);
            }
        }

        // step 2.11 绘制并显示 predicted 的图像
        // 绘制 predicted 的深度图像
        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

        // 显示, 上者是彩色图像, 下者是深度图像
        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        // step 2.12 其他的一些状态量的更新, 并且结束绘制
        // Global Model 中点的数目
        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();
        gui->totalPoints->operator=(strs.str());

        // 获取 Deformation Graph 中 Node 的数目 -- 不过这里的变量命名也是够了...
        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();
        gui->totalNodes->operator=(strs2.str());

        // 获取随机蕨数据库中帧的数目
        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();
        gui->totalFerns->operator=(strs3.str());

        // 获取由于 Local Loop 进行的 deformation 的次数
        std::stringstream strs4;
        strs4 << eFusion->getDeforms();
        gui->totalDefs->operator=(strs4.str());

        // 获取当前已经处理的帧数/总帧数
        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();
        gui->logProgress->operator=(strs5.str());

        // 获取由于 Global Loop 进行的 deformation 的次数
        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();
        gui->totalFernDefs->operator=(strs6.str());

        // 统计并统计并计算剩余的显存, 结束当前Pangolin帧的绘制
        gui->postCall();

        // step 3 结合 GUI 界面的输入, 及时更新 logReader 和 ElasticFusion 的状态
        logReader->flipColors = gui->flipColors->Get();                     // 是否交换颜色通道
        eFusion->setRgbOnly(gui->rgbOnly->Get());                           // 是否只使用 2.5D RGB-only Lucas-Kanade tracking // ? 2.5D
        eFusion->setPyramid(gui->pyramid->Get());                           // 是否使用图像金字塔进行追踪
        eFusion->setFastOdom(gui->fastOdom->Get());                         // 是否运行于快速的里程计模式(不建图, 单层金字塔)
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());   // Raw data fusion confidence threshold
        eFusion->setDepthCutoff(gui->depthCutoff->Get());                   // 设置深度切断值(大于这个距离的深度我们不要了)
        eFusion->setIcpWeight(gui->icpWeight->Get());                       // 设置 Weight for ICP in tracking, 也就是几何误差和色彩误差相互的权重
        eFusion->setSo3(gui->so3->Get());                                   // 是否 Disables SO(3) pre-alignment in tracking
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());           // 是否使用帧-帧RGB方式的位姿估计

        // 获取当前是否要求复位
        resetButton = pangolin::Pushed(*gui->reset);

        // 是否使用 live camera 的自动曝光和自动白平衡设置
        // BUG 如果 autoSettings 为 False 的话, 循环体内的语句不执行啊
        if(gui->autoSettings)
        {
            // 那之前也是这样吗
            static bool last = gui->autoSettings->Get();
            if(gui->autoSettings->Get() != last)
            {
                // 之前不是这样, 说明是刚刚按下的按钮, 我们要打开相机的相关设置
                last = gui->autoSettings->Get();
                static_cast<LiveLogReader *>(logReader)->setAuto(last);
            }
        }

        // 统计完成了所有模块的运行时间数据, 发送
        // ! 但是这个时候 GUI 模块的运行时间还没有统计完成呢?
        Stopwatch::getInstance().sendAll();

        // 如果已经要求复位了后面的东西就不需要执行了 
        // ! 这行代码可以出现在更前面
        if(resetButton)
        {
            break;
        }

        // 如果要保存 Global Model 到磁盘文件
        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        // GUI 界面的内容绘制完成
        TOCK("GUI");
    }
}
