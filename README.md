# C2-403-Infrared-Defect-Detection

## 0.环境依赖
    实验平台：  
      * Windows10+VS2019  
    第三方库：  
      * opecv4.5.5  
      * open3dv0.10.0  
      * AstraSDK（RGBD深度相机）  
      * rokaeSDK（珞石机械臂） 
      * Anaconda VILD环境（目标检测模型）
      * MVCAMSDK（高清视觉相机）
  
## 1.环境配置
### 1.0 包含目录
        项目右击->属性->配置属性->VC++目录->包含目录  
#### 1.0.0 opencv
        lib_root/opencv/build/include
        lib_root/opencv/build/include/opencv2  
#### 1.0.1 open3d
        lib_root/Open3D_v0.10.0/include  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/Eigen  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/GLFW/include  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/glew/include  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/fmt/include  
#### 1.0.2 astra
        lib_root/OpenIN2SDK/Samples/samples.old/ThirdParty/OpenNI2/Include  
        lib_root/OpenIN2SDK/Samples/samples.old/ThirdParty/UvcSwapper/Include  
        lib_root/AstraSDK-vs2015-win64/include  
        lib_root/AstraSDK-vs2015-win64/thirdparty/SFML-min-64/include  
#### 1.0.3 rokae
        lib_root/rokae/include  
#### 1.0.4 Anaconda VILD
        anaconda_root/Anaconda3/envs/Env_VILD/include  
### 1.1 库目录
        项目右击->属性->配置属性->VC++目录->库目录
#### 1.1.0 opencv
        lib_root/opencv/build/x64/vc15/lib  
#### 1.1.1 open3d
        lib_root/Open3D_v0.10.0/lib  
#### 1.1.2 astra
        lib_root/OpenIN2SDK/Samples/samples.old/ThirdParty/OpenNI2/x64-Release  
        lib_root/OpenIN2SDK/Samples/samples.old/ThirdParty/d2c/lib/win64/release  
        lib_root/OpenIN2SDK/Samples/samples.old/ThirdParty/UvcSwapper/x64-Release  
        lib_root/AstraSDK-vs2015-win64/thirdparty/SFML-min-64/lib  
        lib_root/AstraSDK-vs2015-win64/lib  
#### 1.1.3 rokae
        lib_root/rokae/include  
#### 1.1.4 Anaconda VILD
        anaconda_root/Anaconda3/envs/Env_VILD/include  
### 1.2 链接器输入
        项目右击->属性->配置属性->链接器->输入  
        将库目录中的所有*.lib文件名输入
        MVCAMSDK_X64.lib python37.lib RokaeSDK.lib jsoncpp.lib astra.lib astra_core.lib astra_core_api.lib astra_jni.lib Shiny-static.lib opencv_world455.lib UVC-Swapper.lib OpenNI2.lib sfml-graphics-d.lib sfml-graphics.lib sfml-system-d.lib sfml-system.lib sfml-window-d.lib sfml-window.lib glew.lib glfw3.lib Open3D.lib png.lib qhullcpp.lib qhullstatic_r.lib tinyfiledialogs.lib tinyobjloader.lib turbojpeg-static.lib zlib.lib d2c.lib opengl32.lib
### 1.3 系统环境变量
        lib_root/opencv/build/x64/vc15/bin  
        lib_root/AstraSDK-vs2015-win64/thirdparty/SFML-min-64/bin  
        lib_root/AstraSDK-vs2015-win64/thirdparty/SFML-min-64/bin  
        
## 2.工程说明
### 2.0 文件结构
        agv control文件夹：其中主要是AGV控制相关实现  
        algorithm文件夹：主要包括图像、点云、机器人控制等相关算法实现  
        astra文件夹：主要是深度相机相关接口  
        calibration文件夹：主要是相机标定、坐标转换相关实现  
        global文件夹：主要包括宏定义、全局参数（相机参数）等  
        io file文件夹：主要是对读写文件相关实现  
        mindvision文件夹：主要是高清视觉相关操作接口  
        robot arm文件夹：是安川、珞石等机械臂相关接口  
        results文件夹：主要是各阶段处理结果保存根目录
### 2.1 工程
        该工程主要分为实现AGV搭载机械臂机械臂夹持光激励热成像设备实现自动检测。  
        整个过程分为三个阶段：
                一阶段-过程学习：交互并且采集多视角深度、高清数据  
                二阶段-自学习：将多视角数据通过自学习分割目标被测试件三维模型并划分被测试件融合测量场并采集红外光激励数据
                三阶段-融合表征：将深度数据三维模型、光激励缺陷特征图融合表征
### 2.2 修改说明
        每次修改后上传的工程，需要在程序中写名修改者和修改时间并在"版本管理.doc"中进行说明（功能是否测试、是否存在问题等）
### 2.3 库文件下载
        链接: https://pan.baidu.com/s/1lfVumEaTQD0TIQ3u80yCsQ?pwd=C403   
        提取码: C403
