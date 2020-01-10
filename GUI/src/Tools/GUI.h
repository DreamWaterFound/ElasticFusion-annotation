/**
 * @file GUI.h
 * @author guoqing (1337841346@qq.com)
 * @brief GUI 对象
 * @version 0.1
 * @date 2020-01-06
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

#ifndef GUI_H_
#define GUI_H_

// pangolin
#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
// STL
#include <map>

// ElasticFusion 自己声明的 GPU 纹理对象
#include <GPUTexture.h>

// Utils, 提供相机的内参文件
#include <Utils/Intrinsics.h>
// OpenGL Shader
#include <Shaders/Shaders.h>

// 保存有当前显存余量的寄存器
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

// FUTURE_TODO 进一步检查之前的注释, 以及涉及到的变量是否正确

/** @brief 图形用户界面对象 */
class GUI
{
    public:
        /**
         * @brief 构造函数
         * @param[in] liveCap       是否使用实时摄像头
         * @param[in] showcaseMode  是否 MapViewer 全屏
         */
        GUI(bool liveCap, bool showcaseMode)
         : showcaseMode(showcaseMode)
        {
            // step 1 设置窗口参数
            // Map Viewer 窗口尺寸
            width = 1280;
            height = 980;
            // 选项面板尺寸
            panel = 205;

            // 真正的窗口尺寸
            width += panel;

            // 窗口参数对象, 本质上就是一个字符串对
            pangolin::Params windowParams;

            // ? 多重采样设置, 疑似和绘制后面的透明物体(主要是时间窗口吧)有关系
            windowParams.Set("SAMPLE_BUFFERS", 0);
            // ?
            windowParams.Set("SAMPLES", 0);

            // step 2 创建窗口, 并设置 OepnGL 属性, 变量和Shader
            pangolin::CreateWindowAndBind("Main", width, height, windowParams);

            // 为了避免 OepnGL 中从CPU向GPU传输过程中默认的四字节对齐使得图像出现错位, 这里设置为1字节对齐
            // 可以参考: [https://blog.csdn.net/czhzasui/article/details/81183782]
            //          [http://blog.sina.com.cn/s/blog_7d7447e70102xogs.html]
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);      // CPU => GPU,  Unpack
            glPixelStorei(GL_PACK_ALIGNMENT, 1);        // GPU => CPU,  Pack (因为内存中的数据格式更加规整)

            // Internally render at 3840x2160
            // 这个分辨率决定了使用 faxx 方式绘制 Global Model 时候使用的分辨率
            renderBuffer = new pangolin::GlRenderBuffer(3840, 2160),
            
            
            // ? 用于 fxaa 第二阶段的输出纹理
            colorTexture = new GPUTexture(
                renderBuffer->width,        // 图像宽度
                renderBuffer->height,       // 图像高度
                GL_RGBA32F,                 // 图像格式
                GL_LUMINANCE,               // 设置对纹理格式的处理, 这个宏相当于把纹理的不透明度直接设置成为1了
                                            // ref: [https://blog.csdn.net/jinzhilong580231/article/details/6882682]
                GL_FLOAT,                   // 每个通道的数据类型
                true);                      // ? 绘制 = true

            // 帧缓存, 绑定彩色纹理和深度信息为上面的两者
            colorFrameBuffer = new pangolin::GlFramebuffer;
            colorFrameBuffer->AttachColour(*colorTexture->texture);
            colorFrameBuffer->AttachDepth(*renderBuffer);

            // 着色语言 GLSL 加载, 这两个 GLSL 程序都是为 fxaa 渲染服务的
            // ? 第一阶段使用的
            colorProgram = std::shared_ptr<Shader>(
                loadProgramFromFile("draw_global_surface.vert", "draw_global_surface_phong.frag", "draw_global_surface.geom"));
            // ? 第二阶段使用的
            fxaaProgram = std::shared_ptr<Shader>(loadProgramFromFile("empty.vert", "fxaa.frag", "quad.geom"));

            // 设置窗口的状态
            pangolin::SetFullscreen(showcaseMode);

            // 用于绘制透明图形, ref:
            //      [https://www.iteye.com/blog/tiankefeng0520-2008008]
            //      [https://blog.csdn.net/u012463389/article/details/50748128]
            glEnable(GL_DEPTH_TEST);    // 启用深度测试
            glDepthMask(GL_TRUE);       // 设置深度缓冲区可读写
            glDepthFunc(GL_LESS);       // 指定“目标像素与当前像素在z方向上值大小比较”的函数，符合该函数关系的目标像素才进行绘制，否则对目标像素不予绘制

            // step 3 子视图创建
            /* 一共有这样几个视图id: 
             * cam          Global Model 显示视图
             * RGB          输入的原始彩色图像
             * DEPTH_NORM   输入的原始彩色图像, 但是为了效果会对深度值进行裁切
             * ModelImg     Global Model Prediction 的彩色图像
             * Model        Global Model Prediction 的深度图像
             * ui           Panel
             * multi        RGB + DEPTH_NORM + ModelImg + Model + 两个Ploter组成的综合视图
             */ 


            // 场景渲染器
            s_cam = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),    // 投影矩阵
                pangolin::ModelViewLookAt(0, 0, -1, 0, 0, 1, pangolin::AxisNegY));      // 虚拟观测相机位置

            // MapViewer
            pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

            // 当前帧彩色图像窗口
            pangolin::Display(GPUTexture::RGB).SetAspect(640.0f / 480.0f);
            // 当前帧深度图像窗口
            pangolin::Display(GPUTexture::DEPTH_NORM).SetAspect(640.0f / 480.0f);
            // 地图模型的彩色图像窗口
            pangolin::Display("ModelImg").SetAspect(640.0f / 480.0f);
            // 地图模型的深度图像窗口
            pangolin::Display("Model").SetAspect(640.0f / 480.0f);

            // step 4 ploter 创建
            // 第一个 ploter
            std::vector<std::string> labels;
            labels.push_back(std::string("residual"));
            labels.push_back(std::string("threshold"));
            resLog.SetLabels(labels);

            resPlot = new pangolin::Plotter(&resLog, 0, 300, 0, 0.0005, 30, 0.5);
            resPlot->Track("$i");

            // 第二个 ploter
            std::vector<std::string> labels2;
            labels2.push_back(std::string("inliers"));
            labels2.push_back(std::string("threshold"));
            inLog.SetLabels(labels2);

            inPlot = new pangolin::Plotter(&inLog, 0, 300, 0, 40000, 30, 0.5);
            inPlot->Track("$i");

            // step 5 创建 panel, 以及综上述 Viewer 的总 Viewer
            if(!showcaseMode)
            {
                // 只有不是模型全屏的时候才回显示
                // 显示 panel
                pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel));
                // 把这几个其余的Viewer组合成为一个完整的Viewer
                pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, showcaseMode ? 0 : pangolin::Attach::Pix(180), 1.0)
                                          // 横向布局
                                          .SetLayout(pangolin::LayoutEqualHorizontal)
                                          // 这里其实都是之前生成视图, 感觉这些视图都是静态的
                                          .AddDisplay(pangolin::Display(GPUTexture::RGB))
                                          .AddDisplay(pangolin::Display(GPUTexture::DEPTH_NORM))
                                          .AddDisplay(pangolin::Display("ModelImg"))
                                          .AddDisplay(pangolin::Display("Model"))
                                          .AddDisplay(*resPlot)
                                          .AddDisplay(*inPlot);
            }

            // panel 上的变量
            pause       = new pangolin::Var<bool>("ui.Pause", false, true);         // 是否暂停
            step        = new pangolin::Var<bool>("ui.Step", false, false);         // 步进按钮
            save        = new pangolin::Var<bool>("ui.Save", false, false);         // 保存按钮
            reset       = new pangolin::Var<bool>("ui.Reset", false, false);        // 复位按钮
            flipColors  = new pangolin::Var<bool>("ui.Flip RGB", false, true);      // 是否翻转颜色(BGR, RGB)

            if(liveCap)
            {
                // 如果使用实际摄像头, 则有这一项
                autoSettings = new pangolin::Var<bool>("ui.Auto Settings", true, true);
            }
            else
            {
                // 如果跑的是记录文件(也就是数据集), 那么就没有这一项
                autoSettings = 0;
            }

            // ==== Panel 上的变量 - 选项 ====
            pyramid             = new pangolin::Var<bool>("ui.Pyramid", true, true);                        // 是否使用图像金字塔
            so3                 = new pangolin::Var<bool>("ui.SO(3)", true, true);                          // 是否 Disables SO(3) pre-alignment in tracking
            frameToFrameRGB     = new pangolin::Var<bool>("ui.Frame to frame RGB", false, true);            // 是否使用帧-帧RGB方式的位姿估计
            fastOdom            = new pangolin::Var<bool>("ui.Fast Odometry", false, true);                 // 是否使用快速视觉里程计模式
            rgbOnly             = new pangolin::Var<bool>("ui.RGB only tracking", false, true);             // 是否仅使用 2.5D RGB-only Lucas-Kanade tracking // ? 2.5D?

            // Panel 上的变量 - 设置
            confidenceThreshold = new pangolin::Var<float>("ui.Confidence threshold", 10.0, 0.0, 24.0);     // Raw data fusion confidence threshold
            depthCutoff         = new pangolin::Var<float>("ui.Depth cutoff", 3.0, 0.0, 12.0);              // 深度值的切断处
            icpWeight           = new pangolin::Var<float>("ui.ICP weight", 10.0, 0.0, 100.0);              // 设置 Weight for ICP in tracking, 也就是几何误差和色彩误差相互的权重

            // ==== Panel 上的变量 - 选项 ====
            followPose          = new pangolin::Var<bool>("ui.Follow pose", true, true);                    // 观察视角是否跟随相机位姿
            drawRawCloud        = new pangolin::Var<bool>("ui.Draw raw", false, true);                      // 是否显示当前帧的原始点云
            drawFilteredCloud   = new pangolin::Var<bool>("ui.Draw filtered", false, true);                 // 是否显示滤波后的点云
            drawGlobalModel     = new pangolin::Var<bool>("ui.Draw global model", true, true);              // 是否显示构建的全局模型
            drawUnstable        = new pangolin::Var<bool>("ui.Draw unstable points", false, true);          // 是否显示不稳定的点
            drawPoints          = new pangolin::Var<bool>("ui.Draw points", false, true);                   // 是否绘制点而不是surfel
            drawColors          = new pangolin::Var<bool>("ui.Draw colors", showcaseMode, true);            // 是否使用彩色图纹理贴图
            drawFxaa            = new pangolin::Var<bool>("ui.Draw FXAA", showcaseMode, true);              // 是否使用 FXAA
            drawWindow          = new pangolin::Var<bool>("ui.Draw time window", false, true);              // 是否绘制时间窗口
            drawNormals         = new pangolin::Var<bool>("ui.Draw normals", false, true);                  // 是否对模型进行法线贴图
            drawTimes           = new pangolin::Var<bool>("ui.Draw times", false, true);                    // 是否安装Surfel的创建时间进行着色
            drawDefGraph        = new pangolin::Var<bool>("ui.Draw deformation graph", false, true);        // 是否绘制 Deformation graph
            drawFerns           = new pangolin::Var<bool>("ui.Draw ferns", false, true);                    // 是否绘制随机蕨数据库
            drawDeforms         = new pangolin::Var<bool>("ui.Draw deformations", true, true);              // 是否绘制 Local Loop 和 Global Loop后的模型的矫正

            // Panel 上的变量 - 只读
            gpuMem              = new pangolin::Var<int>("ui.GPU memory free", 0);                          // 当前 GPU 显存余量

            totalPoints         = new pangolin::Var<std::string>("ui.Total points", "0");                   // 当前 Global Model 中点的总个数
            totalNodes          = new pangolin::Var<std::string>("ui.Total nodes", "0");                    // deformation graph 中的 node 总个数
            totalFerns          = new pangolin::Var<std::string>("ui.Total ferns", "0");                    // 随机蕨数据库中的帧总个数
            totalDefs           = new pangolin::Var<std::string>("ui.Total deforms", "0");                  // 由 Local Loop 触发的 deformation 次数
            totalFernDefs       = new pangolin::Var<std::string>("ui.Total fern deforms", "0");             // 由 Global Loop 触发的 deformation 次数

            trackInliers        = new pangolin::Var<std::string>("ui.Inliers", "0");                        // 当前次 ICP 过程中的 inliers
            trackRes            = new pangolin::Var<std::string>("ui.Residual", "0");                       // 当前次 ICP 过程中的残差
            logProgress         = new pangolin::Var<std::string>("ui.Log", "0");                            // 已经处理过的帧数(记录数)

            // 如果处于 MapViewer 全屏模式下, 则按空格键执行复位操作
            if(showcaseMode)
            {
                pangolin::RegisterKeyPressCallback(' ', pangolin::SetVarFunctor<bool>("ui.Reset", true));
            }
        }

        /**
         * @brief 析构函数, 就是删除各种 new 出来的对象
         * 
         */
        virtual ~GUI()
        {
            delete pause;
            delete reset;
            delete inPlot;
            delete resPlot;

            // 只有使用 live camera 的时候才回 new 这个对象
            if(autoSettings)
            {
                delete autoSettings;

            }
            delete step;
            delete save;
            delete trackInliers;
            delete trackRes;
            delete confidenceThreshold;
            delete totalNodes;
            delete drawWindow;
            delete so3;
            delete totalFerns;
            delete totalDefs;
            delete depthCutoff;
            delete logProgress;
            delete drawTimes;
            delete drawFxaa;
            delete fastOdom;
            delete icpWeight;
            delete pyramid;
            delete rgbOnly;
            delete totalFernDefs;
            delete drawFerns;
            delete followPose;
            delete drawDeforms;
            delete drawRawCloud;
            delete totalPoints;
            delete frameToFrameRGB;
            delete flipColors;
            delete drawFilteredCloud;
            delete drawNormals;
            delete drawColors;
            delete drawGlobalModel;
            delete drawUnstable;
            delete drawPoints;
            delete drawDefGraph;
            delete gpuMem;

            delete renderBuffer;
            delete colorFrameBuffer;
            delete colorTexture;
        }

        /** @brief 在绘制 MaoViewer 之前准备 */
        void preCall()
        {
            // ! 不太建议的代码技巧...  
            // 这里的原意是如果处于 showcaseMode 模式(MapViewer全屏)那么窗口的背景就要填充黑色, 否则则填充蓝色
            glClearColor(0.05 * !showcaseMode, 0.05 * !showcaseMode, 0.3 * !showcaseMode, 0.0f);
            // GL_COLOR_BUFFER_BIT: 清除之前绘制的像素信息缓冲, 否则显示内容就会每一帧叠加在一起
            // GL_DEPTH_BUFFER_BIT: 清除之间绘制的深度缓冲, 否则之前绘制帧的时候形成的"遮挡关系"就会一直保存, 导致最终绘制出来的效果出现莫名其妙的遮挡
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            // 更新当前窗口的高度和宽度
            width = pangolin::DisplayBase().v.w;
            height = pangolin::DisplayBase().v.h;

            // 激活要绘制的 MapViewer 区域
            pangolin::Display("cam").Activate(s_cam);
        }

        /**
         * @brief 绘制相机的视锥
         * @param[in] pose 相机当前的位姿
         */
        inline void drawFrustum(const Eigen::Matrix4f & pose)
        {
            // showcase 模式下不显示
            if(showcaseMode)
                return;

            // 获取相机内参
            Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
            K(0, 0) = Intrinsics::getInstance().fx();
            K(1, 1) = Intrinsics::getInstance().fy();
            K(0, 2) = Intrinsics::getInstance().cx();
            K(1, 2) = Intrinsics::getInstance().cy();

            Eigen::Matrix3f Kinv = K.inverse();

            // HACK 这里做了一定的修改， 避免找不到函数的问题
            #ifdef USE_OPENNI_PANGOLIN
            // 原来 Pangolin 提供了绘制相机视锥的函数了啊
            pangolin::glDrawFrustrum(Kinv,                                      // 相机内参的逆
                                     Resolution::getInstance().width(),         // 图像宽度
                                     Resolution::getInstance().height(),        // 图像高度
                                     pose,                                      // 相机位姿
                                     0.1f);                                     // 大小                   

            #else
            pangolin::glDrawFrustum(Kinv,                                       // 相机内参的逆
                                    Resolution::getInstance().width(),          // 图像宽度
                                    Resolution::getInstance().height(),         // 图像高度
                                    pose,                                       // 相机位姿
                                    0.1f);                                      // 大小                   
            #endif
        }

        /**
         * @brief 绘制纹理到指定的View
         * @param[in] id    View的id, 使用字符串表示
         * @param[in] img   纹理对象
         */
        void displayImg(const std::string & id, GPUTexture * img)
        {
            // Showcase 模式下不输出
            if(showcaseMode)
                return;

            // 为什么要失能深度测试呢? 因为这里只需要绘制平面的纹理, 并不需要绘制空间物体, 也就无从需要确定物体之间的相互遮挡关系
            glDisable(GL_DEPTH_TEST);
            // 激活对应的 View, 然后执行渲染即可显示
            pangolin::Display(id).Activate();
            img->texture->RenderToViewport(true);
            glEnable(GL_DEPTH_TEST);
        }

        /** @brief 统计并计算剩余的显存, 结束当前Pangolin帧的绘制 */
        void postCall()
        {
            GLint cur_avail_mem_kb = 0;
            // 获取 kb 为单位的显存空余量
            glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &cur_avail_mem_kb);
            // 转换成为 MB 表示
            int memFree = cur_avail_mem_kb / 1024;
            gpuMem->operator=(memFree);
            
            // 结束一次绘制
            pangolin::FinishFrame();
            glFinish();
        }

        /**
         * @brief 使用快速抗锯齿的方式绘制 Global Model
         * @param[in] mvp               模型显示视图的投影矩阵, mvp = model view projection
         * @param[in] mv                虚拟观察相机以何种位姿观测, mv = model view
         * @param[in] model             ElasticFusion 建立的 Global Model
         * @param[in] threshold         The point fusion confidence threshold
         * @param[in] time              当前 ElasticFusion 已经处理过的帧数
         * @param[in] timeDelta         时间窗口的长度
         * @param[in] invertNormals     是否使用 ICL-NUIM 数据集
         */
        // FUTURE_TODO 进一步检查之前的注释, 以及涉及到的变量是否正确
        void drawFXAA(pangolin::OpenGlMatrix mvp,
                      pangolin::OpenGlMatrix mv,
                      const std::pair<GLuint, GLuint> & model,
                      const float threshold,
                      const int time,
                      const int timeDelta,
                      const bool invertNormals)
        {
            // step 1 绘图准备
            // First pass computes positions, colors and normals per pixel
            // 指定当前绘图使用帧缓冲 colorFrameBuffer
            colorFrameBuffer->Bind();
            
            // ? 告知下面的操作是对于 ViewPort 的? 属性设置入栈
            glPushAttrib(GL_VIEWPORT_BIT);
            // 设置视口大小
            glViewport(0, 0,                                            // 视口的左上角点坐标
                       renderBuffer->width, renderBuffer->height);      // 是否的宽度和高度
            // 重设背景
            glClearColor(0.05 * !showcaseMode, 0.05 * !showcaseMode, 0.3 * !showcaseMode, 0);
            // 清空色彩和深度缓冲区
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // step 2 装载着色器程序并传入参数
            colorProgram->Bind();
            // 将对应的 CPU 端的变量 mvp 加载到 GLSL 程序中的 "MVP" 变量中, 下同
            // 模型显示视图的投影矩阵
            colorProgram->setUniform(Uniform("MVP", mvp));                      
            // The point fusion confidence threshold
            colorProgram->setUniform(Uniform("threshold", threshold));
            // 当前 ElasticFusion 已经处理过的帧数
            colorProgram->setUniform(Uniform("time", time));
            // 时间窗口的长度
            colorProgram->setUniform(Uniform("timeDelta", timeDelta));
            // 是否翻转Y轴(不翻转的话, 上方是Y轴的负方向)
            colorProgram->setUniform(Uniform("signMult", invertNormals ? 1.0f : -1.0f));
            // 着色模式, 有三种
            colorProgram->setUniform(Uniform(
                            "colorType",
                             (drawNormals->Get() ? 1 :                  // 1 - 法向贴图
                                    drawColors->Get() ? 2 :             // 2 - RGB 图像纹理贴图
                                        drawTimes->Get() ? 3 : 0)));    // 3 - 按照面元创建的时间先后顺序进行着色
            // 是否绘制 unstable 的点
            colorProgram->setUniform(Uniform("unstable", drawUnstable->Get()));
            // 是否绘制时间窗口
            colorProgram->setUniform(Uniform("drawWindow", drawWindow->Get()));
            // 相机位姿
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            // This is for the point shader
            colorProgram->setUniform(Uniform("pose", pose));

            // 设置 Model View 对应的虚拟观察相机光心处设置为光心
            Eigen::Matrix4f modelView = mv;
            Eigen::Vector3f lightpos = modelView.topRightCorner(3, 1);
            colorProgram->setUniform(Uniform("lightpos", lightpos));

            // step 3 // ? 下面的全部都看不懂了... 一些图像设置
            glBindBuffer(GL_ARRAY_BUFFER, model.first);

            // 注意到这里的 glEnableVertexAttribArray 和后面的 glDisableVertexAttribArray 是成对出现的
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 1));

            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

            glDrawTransformFeedback(GL_POINTS, model.second);

            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);
            glDisableVertexAttribArray(2);
            // ? 解绑到id 0?
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            // 卸载 GLSL 程序
            colorFrameBuffer->Unbind();
            // 解除缓冲区的绑定
            colorProgram->Unbind();
            // 恢复属性堆栈
            glPopAttrib();

            // step 4 解除和缓冲区的绑定, 卸载 GLSL 程序, 弹出先前的设置, 恢复现场xuanran
            fxaaProgram->Bind();

            glActiveTexture(GL_TEXTURE0);
            // ? 绑定输出纹理
            glBindTexture(GL_TEXTURE_2D, colorTexture->texture->tid);

            // // 传参 - 分辨率
            Eigen::Vector2f resolution(renderBuffer->width, renderBuffer->height);
            fxaaProgram->setUniform(Uniform("tex", 0));                     // ? 有何意义？
            fxaaProgram->setUniform(Uniform("resolution", resolution));

            // ? 绘制
            glDrawArrays(GL_POINTS, 0, 1);

            // step 6 结束, 卸载 GLSL 程序, 下面的好多还是看不懂
            fxaaProgram->Unbind();

            glBindFramebuffer(GL_READ_FRAMEBUFFER, colorFrameBuffer->fbid);

            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

            glBlitFramebuffer(0, 0, renderBuffer->width, renderBuffer->height, 0, 0, width, height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);

            glBindTexture(GL_TEXTURE_2D, 0);

            // 调用 GPU 立即根据指令缓冲进行绘制操作
            glFinish();
        }

        bool showcaseMode;                                  ///< 是否 MapViewer 部分全屏
        int width;                                          ///< 窗口宽度, 这个是可以在 preCall() 函数中被更新的
        int height;                                         ///< 窗口高度, 这个是可以在 preCall() 函数中被更新的
        int panel;                                          ///< 面板宽度

        // Panel 上的变量 - 选项
        pangolin::Var<bool> * pause,                        ///< panel - 是否暂停
                            * step,                         ///< panel - 步进按钮
                            * save,                         ///< panel - 保存按钮
                            * reset,                        ///< panel - 复位按钮
                            * flipColors,                   ///< panel - 是否翻转图像的颜色(RGB和BGR)
                            * rgbOnly,                      ///< panel - 是否仅使用 2.5D RGB-only Lucas-Kanade tracking // ? 2.5D?
                            * pyramid,                      ///< panel - 是否使用图像金字塔进行追踪
                            * so3,                          ///< panel - 是否 Disables SO(3) pre-alignment in tracking
                            * frameToFrameRGB,              ///< panel - 是否使用帧-帧RGB方式的位姿估计
                            * fastOdom,                     ///< panel - 是否运行于快速的里程计模式(不建图, 单层金字塔)
                            * followPose,                   ///< panel - 是否观察视角跟随相机位姿
                            * drawRawCloud,                 ///< panel - 是否绘制当前帧观测到的原始点云
                            * drawFilteredCloud,            ///< panel - 是否绘制滤波后的点云
                            * drawNormals,                  ///< panel - 是否绘制法向贴图(优先彩色贴图)
                            * autoSettings,                 ///< panel - 是否使用 live camera 的自动曝光和自动白平衡设置
                            * drawDefGraph,                 ///< panel - 是否绘制 deformation graph
                            * drawColors,                   ///< panel - 是否进行彩色贴图
                            * drawFxaa,                     ///< panel - 是否应用快速近似抗锯齿效果绘制构建的模型?
                            * drawGlobalModel,              ///< panel - 是否绘制构建的全局模型
                            * drawUnstable,                 ///< panel - 是否那些不稳定状态的点? (//? 怎么定义不稳定?)
                            * drawPoints,                   ///< panel - 是否不绘制Surfel而是以点云的方式绘制
                            * drawTimes,                    ///< panel - 是否按照创建的时间先后顺序对 Surfel 进行着色(仅对Surfel有效)
                            * drawFerns,                    ///< panel - 是否显示随机蕨数据库中的图像对应的的相机位置
                            * drawDeforms,                  ///< panel - 是否绘制 Local Loop 和 Global Loop 对构建模型的畸变矫正操作?
                            * drawWindow;                   ///< panel - 是否绘制时间窗口(时间窗口外的surfel将会以半透明状态?显示)
            
        // Panel 上的变量 - 只读
        pangolin::Var<int> * gpuMem;                        ///< panel - 当前空闲的显存大小, 单位MB
        pangolin::Var<std::string> * totalPoints,           ///< panel - 模型中点的总个数
                                   * totalNodes,            ///< deformation graph 中的 node 总个数
                                   * totalFerns,            ///< 随机蕨数据库中的帧总个数
                                   * totalDefs,             ///< 由 Local Loop 触发的 deformation 次数
                                   * totalFernDefs,         ///< 由 Global Loop 触发的 deformation 次数
                                   * trackInliers,          ///< 当前 ICP 配准过程中的 Inlier 对数
                                   * trackRes,              ///< 当前 ICP 配准过程中的残差
                                   * logProgress;           ///< "处理的图像帧数/记录文件总帧数"

        // Panel 上的变量 - 设置
        pangolin::Var<float> * confidenceThreshold,         ///< Raw data fusion confidence threshold
                             * depthCutoff,                 ///< 深度值的切断处
                             * icpWeight;                   ///< 设置 Weight for ICP in tracking, 也就是几何误差和色彩误差相互的权重

        pangolin::DataLog resLog, inLog;                    ///< 记录ICP过程残差和内点的数据记录器
        pangolin::Plotter * resPlot,                        ///< 绘制ICP过程残差的绘图器
                          * inPlot;                         ///< 绘制ICP过程inlier数目的绘图器

        pangolin::OpenGlRenderState s_cam;                  ///< 三维场景渲染器

        pangolin::GlRenderBuffer *  renderBuffer;           ///? 渲染缓存? 好像和深度缓存有关系
        pangolin::GlFramebuffer *   colorFrameBuffer;       ///? 帧缓存? 和上面的有什么区别? 貌似这个包含上面的那个; 用于fxaa第一阶段
        GPUTexture *                colorTexture;           ///? 当前帧彩色图像的纹理? 用于 fxaa 第二阶段
        std::shared_ptr<Shader>     colorProgram;           ///? 存储用于正常着色的着色器对象, 用于 fxaa 渲染第一阶段
        std::shared_ptr<Shader>     fxaaProgram;            ///? 存储用于去锯齿的着色器对象, 用于 fxaa 渲染第二阶段
};


#endif /* GUI_H_ */
