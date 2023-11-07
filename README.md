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
