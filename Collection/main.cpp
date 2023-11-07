#include "EachStageProcess.h"
#include "AgvControl.h"
#include "YaskawaRobotArm.h"
#include "RokaeRobotArm.h"
#include "GlobalVariable.h"
#include "MindVision.h"
//#include <comutil.h> //char* 2 wchar_t 
#include "astra/AstraCameraD2C.h"

int main() {
#pragma comment(lib, "comsuppw.lib") // 解决char* 转 wchar_t报错
#pragma comment(lib, "Ws2_32.lib") // 无法解析inet_addr

#if RGBD_CAMERA
    // 初始化深度相机
    astraCameraD2C = new AstraCameraD2C();
    if (astraCameraD2C->CameraInit(HARDWARE_D2C) != CAMERA_STATUS_SUCCESS) {
        printf("camera init failed\n");
        return 0;
    }
#endif // RGBD_CAMERA

#if HDRGB_CAMERA
    if (!init_mindvision()) {
        printf("mindvision camera init fail! \n");
        return 0;
    }
#endif // HD RGB mindvision

    RokaeRobotArm* robotArm = new RokaeRobotArm();
    //YaskawaRobotArm* robotArm = new YaskawaRobotArm();
    robotArmHandle = BaseRobotArm::GetSingleInstance(robotArm);
    robotArmHandle->InitRobot();

#if ROBOT_ARM
    // 机械臂复位
    robotArmHandle->Move2OnePoint("A+073125-018311+025299-07470+08569-07860BC", 1);    
    //robotArmHandle->Move2OnePoint("A-0325-6254-0629+177-001-176BC", 1);
    
#endif // robot reset

#if AGV_CONTROL
    InitSeerAGV();
#endif // AGV control

#if COLLECT_STAGE == 0
#if INTERACTION
    Eigen::Vector3d goalPos;
    double goalLength, goalWidth = 0.;
    InteractObtainGoalInfo(goalPos, goalLength, goalWidth);
    cout << "distance cal:" << goalPos.norm() << endl;
    if (goalPos.norm() == 0) return 0;
#endif // interaction
    Stage1Process(goalPos);
#elif COLLECT_STAGE == 1
    Stage2Process();
#elif COLLECT_STAGE == 2
    Stage3Process();
#endif

    return 0;
}