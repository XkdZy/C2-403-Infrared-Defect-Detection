#include "RobotAlgorithm.h"
#include "CoordinateTransformation.h"
#include "CalibrateEyeInHand.h"
#include "Seer.h"
#include "GlobalVariable.h"
#include "RoutePlaning.h"
#include "OperatePointCloud.h"

void RobotArmReset() {
    // 使机械臂末端朝向正前方
    // 读取当前机械臂坐标
    string currStrPose = robotArmHandle->ReadRobotArmPosString();
    // 解析string
    cv::Mat curr6DMatPose;
    AnalysisString26D(currStrPose, curr6DMatPose);
    // 转化为4*4矩阵
    cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
    Eigen::Vector3d rx{ H_gripper2base.at<double>(0, 0), H_gripper2base.at<double>(1, 0), H_gripper2base.at<double>(2, 0) };
    Eigen::Vector3d ry{ H_gripper2base.at<double>(0, 1), H_gripper2base.at<double>(1, 1), H_gripper2base.at<double>(2, 1) };
    Eigen::Vector3d position{ H_gripper2base.at<double>(0, 3), H_gripper2base.at<double>(1, 3), H_gripper2base.at<double>(2, 3) };
    Eigen::Vector3d rz{ 1., 0., 0. };
    // 将x、y方向（与设备当前朝向）斯密特正交化
    Eigen::Vector3d dstRx = (rx - rx.transpose() * rz * rz).normalized();
    Eigen::Vector3d dstRy = (ry - ry.transpose() * dstRx * dstRx - ry.transpose() * rz * rz).normalized();
    // 整合位姿
    cv::Mat dstGripperMat = (cv::Mat_<double>(4, 4) <<
        dstRx[0], dstRy[0], rz[0], position[0],
        dstRx[1], dstRy[1], rz[1], position[1],
        dstRx[2], dstRy[2], rz[2], position[2],
        0, 0, 0, 1
        );
    // 4*4位姿转换为string
    string cmdStr;
    ConvertMat2String(dstGripperMat, cmdStr);
    cout << "调整后末端位姿：" << cmdStr << endl;
    // 下达命令
    robotArmHandle->Move2OnePoint(cmdStr, 0);

    return;
}

void RobotArm2Goal(const Eigen::Vector3d& goal, int cameraType) {
    //while (1) {
        //Eigen::Vector3d goal = agi._worldGoalPos;
        //Eigen::Vector3d goal = Eigen::Vector3d{ 3250.22, 31.9879, 38.0136, };
        //if (goal.norm() < 0.2) continue;

    AMRLocalInfo currAmrli;
    // 读取当前agv相对原始位置
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    // 读取当前机械臂坐标
    string currStrPose = robotArmHandle->ReadRobotArmPosString();

    // pose differ
    //cout << "curr - ori:" << (agi._currAgvPos - agi._oriAgvPos).transpose() << endl;
    double xDiff = agi._currAgvPos[0] - agi._oriAgvPos[0];
    double yDiff = agi._currAgvPos[1] - agi._oriAgvPos[1]; // agv坐标系
    double rotateTheta = agi._currAgvPos[2] - agi._oriAgvPos[2];

    //if (goal.norm() < 1.0e-2) continue;  
    Eigen::Vector3d diff{ xDiff, yDiff, rotateTheta };
    Eigen::Vector3d goalRotatedEig = AGVMove2ArmPos(diff, goal, false, agi._oriAgvPos[2]);

    cv::Mat curr6DMatPose;
    AnalysisString26D(currStrPose, curr6DMatPose);
    cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
    // 计算旋转后的机械臂坐标，根据AGV旋转角度，修正机械臂坐标
    // 将末端转换到相机坐标
    cv::Mat curr_camera2base;
    if (cameraType == 0)
        curr_camera2base = H_gripper2base * H_Camera2Gripper;
    else
        curr_camera2base = H_gripper2base * H_Camera2Gripper_HD;

    string cmdStr1, cmdStr2;
    ConvertMat2String(H_gripper2base, cmdStr1);
    ConvertMat2String(curr_camera2base, cmdStr2);
    cout << "当前末端位姿：" << cmdStr1 << endl;
    cout << "当前相机位姿：" << cmdStr2 << endl;
    // rz不变，使rx、ry变换最小，斯密特正交化，rx正交投影
    Eigen::Vector3d rx{ curr_camera2base.at<double>(0, 0), curr_camera2base.at<double>(1, 0), curr_camera2base.at<double>(2, 0) };
    Eigen::Vector3d ry{ curr_camera2base.at<double>(0, 1), curr_camera2base.at<double>(1, 1), curr_camera2base.at<double>(2, 1) };
    Eigen::Vector3d position{ curr_camera2base.at<double>(0, 3), curr_camera2base.at<double>(1, 3), curr_camera2base.at<double>(2, 3) };
    Eigen::Vector3d rz = (goalRotatedEig - position).normalized();
    //Eigen::Vector3d rz {1., 0., 0.};

    // 将x、y方向（与设备当前朝向）斯密特正交化
    Eigen::Vector3d dstRx = (rx - rx.transpose() * rz * rz).normalized();
    Eigen::Vector3d dstRy = (ry - ry.transpose() * dstRx * dstRx - ry.transpose() * rz * rz).normalized();
    //cout << "camera position info:" << position.transpose() << endl;

    cv::Mat dstCameraMat = (cv::Mat_<double>(4, 4) <<
        dstRx[0], dstRy[0], rz[0], position[0],
        dstRx[1], dstRy[1], rz[1], position[1],
        dstRx[2], dstRy[2], rz[2], position[2],
        0, 0, 0, 1
        );
    cv::Mat dstGripperMat;
    if (cameraType == 0)
        dstGripperMat = dstCameraMat * H_Camera2Gripper.inv();
    else
        dstGripperMat = dstCameraMat * H_Camera2Gripper_HD.inv();
    string dstStr1, dstStr2;
    ConvertMat2String(dstCameraMat, dstStr1);
    ConvertMat2String(dstGripperMat, dstStr2);
    cout << "调整后相机位姿：" << dstStr1 << endl;
    cout << "调整后末端位姿：" << dstStr2 << endl;
    cout << "是旋转矩阵吗：" << isRotationMatrix(dstGripperMat) << endl;

    double distance = (goalRotatedEig - position).norm();
    cout << "当前起始点相对目标点的距离：" << distance << endl;

    robotArmHandle->Move2OnePoint(dstStr2, 1);
    cout << endl << endl << endl;
    //}
}

void RobotArm2GoalUpdate(const Eigen::Vector3d& agvGoal, int cameraType) {
    AMRLocalInfo currAmrli;
    // 读取当前agv相对原始位置
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    cv::Mat currBase2Agv = arm2agv(agi._currAgvPos);
    // 读取当前机械臂坐标
    string currStrPose = robotArmHandle->ReadRobotArmPosString();
    cv::Mat curr6DMatPose;
    AnalysisString26D(currStrPose, curr6DMatPose);
    cv::Mat hGripper2Base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
    // 相机坐标转换到基坐标系
    cv::Mat hCamera2Base;
    if (cameraType == 0) hCamera2Base = hGripper2Base * H_Camera2Gripper;
    else hCamera2Base = hGripper2Base * H_Camera2Gripper_HD;
    // 转换到AGV世界坐标
    cv::Mat hBase2Agv = currBase2Agv * hCamera2Base;
    // 统一AGV世界坐标系下，计算相机位姿：rz不变，使rx、ry变换最小，斯密特正交化，rx正交投影
    Eigen::Vector3d rx{ hBase2Agv.at<double>(0, 0), hBase2Agv.at<double>(1, 0), hBase2Agv.at<double>(2, 0) };
    Eigen::Vector3d ry{ hBase2Agv.at<double>(0, 1), hBase2Agv.at<double>(1, 1), hBase2Agv.at<double>(2, 1) };
    Eigen::Vector3d position{ hBase2Agv.at<double>(0, 3), hBase2Agv.at<double>(1, 3), hBase2Agv.at<double>(2, 3) };
    Eigen::Vector3d rz = (agvGoal - position).normalized();

    // 将x、y方向（与设备当前朝向）斯密特正交化
    Eigen::Vector3d dstRx = (rx - rx.transpose() * rz * rz).normalized();
    Eigen::Vector3d dstRy = (ry - ry.transpose() * dstRx * dstRx - ry.transpose() * rz * rz).normalized();
    //cout << "camera position info:" << position.transpose() << endl;
    // 相机在AGV世界坐标系系下的位姿
    cv::Mat dstCamera2Agv = (cv::Mat_<double>(4, 4) <<
        dstRx[0], dstRy[0], rz[0], position[0],
        dstRx[1], dstRy[1], rz[1], position[1],
        dstRx[2], dstRy[2], rz[2], position[2],
        0, 0, 0, 1
        );
    // 将AGV世界坐标系下位姿转换到基坐标系下相机位姿
    cv::Mat dstCamera2Base = currBase2Agv.inv() * dstCamera2Agv;
    // 将相机在基坐标系下坐标转化到末端到基坐标系下位姿
    cv::Mat dstGripper2Base;
    if (cameraType == 0) dstGripper2Base = dstCamera2Base * H_Camera2Gripper.inv();
    else dstGripper2Base = dstCamera2Base * H_Camera2Gripper_HD.inv();
    string dstStr1, dstStr2;
    ConvertMat2String(dstCamera2Base, dstStr1);
    ConvertMat2String(dstGripper2Base, dstStr2);
    cout << "调整后相机位姿：" << dstStr1 << endl;
    cout << "调整后末端位姿：" << dstStr2 << endl;
    cout << "是旋转矩阵吗：" << isRotationMatrix(dstGripper2Base) << endl;

    //double distance = (goalRotatedEig - position).norm();
    //cout << "当前起始点相对目标点的距离：" << distance << endl;

    robotArmHandle->Move2OnePoint(dstStr2, 1);
    cout << endl << endl << endl;
    //}
}

void RobotArm2Goal(cv::Mat gripperPoseMat) {
    AMRLocalInfo currAmrli;
    // 读取当前agv相对原始位置
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    // 读取当前机械臂坐标
    string currStrPose= robotArmHandle->ReadRobotArmPosString();

    cv::Mat arm2agvM = arm2agv(agi._currAgvPos);

    ////AGV当前位置使用机械臂扫查对应的一个分块
    cv::Mat goalarm = arm2agvM.inv() * gripperPoseMat;
    goalarm.at<double>(1, 3) = goalarm.at<double>(1, 3) + 100;
    //cv::Mat goalarm = arm2agvM.inv() * dstCameraMat;
    string  dstStr2;
    ConvertMat2String(goalarm, dstStr2);
    cout << "输入任意数字移动机械臂：" << endl;
    cout << "调整后末端位姿,第" << "次：" << dstStr2 << endl;
    int xxx;
    cin >> xxx;
    robotArmHandle->Move2OnePoint(dstStr2, 0);
    cin >> xxx;
    cout << endl << endl << endl;
}


Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta) {
    cv::Mat diffXY = (cv::Mat_<double>(4, 1) << 0., -100., 0, 1); // 机械臂x、y偏心
    Eigen::Vector3d diffXYEig(0., -100, 0);
    robotPos += diffXYEig;
    double xDiff = agvMoveInfo[0];
    double yDiff = agvMoveInfo[1]; // agv坐标系
    double rotateTheta = agvMoveInfo[2];
    cv::Mat agvTransposeCurr2Ori = (cv::Mat_<double>(4, 4) <<
        cos(oriTheta), sin(oriTheta), 0, 0,
        -sin(oriTheta), cos(oriTheta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
        );
    cv::Mat agvRotateCurr2Ori = (cv::Mat_<double>(4, 4) <<
        cos(rotateTheta), -sin(rotateTheta), 0, 0,
        sin(rotateTheta), cos(rotateTheta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
        ); // 顺时针theta为：负；（b1,b2,b3）=RI，H_curr2ori
    //// 偏心带来的旋转偏移
    //cv::Mat xDiffRotated = agvRotateCurr2Ori * diffXY;
    //cout << "偏心旋转偏移为：" << xDiffRotated.t() << endl;
    //xDiff += xDiffRotated.at<double>(0, 0);
    //yDiff += xDiffRotated.at<double>(1, 0);
    // 计算旋转、平移后的机械臂坐标
    cv::Mat translation, rotate, dstPosition;
    if (curr2ori) {
        translation = agvTransposeCurr2Ori * (cv::Mat_<double>(4, 1) << xDiff, yDiff, 0, 0);
        rotate = agvRotateCurr2Ori * (cv::Mat_<double>(4, 1) << robotPos[0], robotPos[1], robotPos[2], 1);
        dstPosition = translation + rotate - diffXY;
    }
    else {
        cv::Mat tmp = (cv::Mat_<double>(4, 1) << robotPos[0], robotPos[1], robotPos[2], 1) -
            agvTransposeCurr2Ori * (cv::Mat_<double>(4, 1) << xDiff, yDiff, 0, 0);
        dstPosition = agvRotateCurr2Ori.inv() * tmp;
        dstPosition -= diffXY;
    }
    //cout << "平移后的坐标为:" << translation.t() << endl;
    //cout << "旋转后的坐标为:" << rotate.t() << endl;
    //cout << "旋转平移后的坐标为:" << dstPosition.t() << endl;
    return { dstPosition.at<double>(0,0), dstPosition.at<double>(1,0), dstPosition.at<double>(2,0) };
}

Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta, Eigen::Vector3d IMU) {
    cv::Mat diffXY = (cv::Mat_<double>(4, 1) << 0., -100., 0, 1); // 机械臂x、y偏心
    Eigen::Vector3d diffXYEig(0., -100, 0);
    robotPos += diffXYEig;
    double xDiff = agvMoveInfo[0];
    double yDiff = agvMoveInfo[1]; // agv坐标系
    double rotateTheta = agvMoveInfo[2];
    cv::Mat agvTransposeCurr2Ori = (cv::Mat_<double>(4, 4) <<
        cos(oriTheta), sin(oriTheta), 0, 0,
        -sin(oriTheta), cos(oriTheta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
        );

    cv::Mat IMU_Cur = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, cos(IMU[1]), sin(IMU[1]), 0,
        0, -sin(IMU[1]), cos(IMU[1]), 0,
        0, 0, 0, 1
        );
    cv::Mat agvRotateCurr2Ori = (cv::Mat_<double>(4, 4) <<
        cos(rotateTheta), -sin(rotateTheta), 0, 0,
        sin(rotateTheta), cos(rotateTheta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
        ); // 顺时针theta为：负；（b1,b2,b3）=RI，H_curr2ori
    //// 偏心带来的旋转偏移
    //cv::Mat xDiffRotated = agvRotateCurr2Ori * diffXY;
    //cout << "偏心旋转偏移为：" << xDiffRotated.t() << endl;
    //xDiff += xDiffRotated.at<double>(0, 0);
    //yDiff += xDiffRotated.at<double>(1, 0);
    // 计算旋转、平移后的机械臂坐标
    cv::Mat translation, rotate, dstPosition;
    if (curr2ori) {
        translation = agvTransposeCurr2Ori * (cv::Mat_<double>(4, 1) << xDiff, yDiff, 0, 0);
        rotate = agvRotateCurr2Ori * (cv::Mat_<double>(4, 1) << robotPos[0], robotPos[1], robotPos[2], 1);
        dstPosition = translation + IMU_Cur * rotate - diffXY;
    }
    else {
        cv::Mat tmp = (cv::Mat_<double>(4, 1) << robotPos[0], robotPos[1], robotPos[2], 1) -
            agvTransposeCurr2Ori * (cv::Mat_<double>(4, 1) << xDiff, yDiff, 0, 0);
        dstPosition = agvRotateCurr2Ori.inv() * tmp;
        dstPosition = IMU_Cur.inv() * tmp;
        dstPosition -= diffXY;
    }
    //cout << "平移后的坐标为:" << translation.t() << endl;
    //cout << "旋转后的坐标为:" << rotate.t() << endl;
    //cout << "旋转平移后的坐标为:" << dstPosition.t() << endl;
    return { dstPosition.at<double>(0,0), dstPosition.at<double>(1,0), dstPosition.at<double>(2,0) };
}


void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos) {
    // 机械臂末端运动距离为：intervalDistance或旋转角度为：rotateTheta时保留一次轨迹点
    double intervalDistance = 50.; // mm
    double rotateTheta = 10. * CV_PI / 180.;
    if (agvPos.size() <= 1) return;

    //for (int agvIdx = 0; agvIdx < agvPos.size(); agvIdx++) {
    //    cout << agvPos[agvIdx].transpose() << endl;
    //}

    Eigen::Vector3d lastPos = agvPos[0];
    for (int agvIdx = 1; agvIdx < agvPos.size(); agvIdx++) {
        Eigen::Vector3d currPos = agvPos[agvIdx];
        // 计算两点之间夹角，判断是否需要旋转AGV，逆时针旋转为正
        double angle = atan2((currPos[1] - lastPos[1]), (currPos[0] - lastPos[0]));
        //cout << "当前点坐标：" << currPos.transpose() << endl
        //    << "  前一个点坐标：" << lastPos.transpose() << endl
        //    << "    两点间夹角为：" << angle * 180. / CV_PI
        //    << "    夹角差：" << (angle - lastPos[2]) * 180. / CV_PI << endl;
        double agvRotate = 0.;
        //// 旋转
        //if (abs(angle - lastPos[2]) * 180. / CV_PI > 2.) {
        //    cout << "夹角大于设定角度，AGV需要旋转！" << endl;
        //    // 夹角大于x°需要旋转，采样旋转角度上的点
        //    for (int rTime = 0; rTime < floor(abs(angle - lastPos[2] / rotateTheta)); rTime++) {
        //        
        //    }
        //}
        // 平移
        double dist = (Eigen::Vector2d{ lastPos[0], lastPos[1] } - Eigen::Vector2d{ currPos[0], currPos[1] }).norm();
        int totalTTimes = abs(dist / intervalDistance); // floor(abs(dist / rotateTheta));
        //cout << "距离：" << dist << "    插入点次数：" << totalTTimes
        //    << "   当前AGV角度：" << angle << "    需要旋转角度：" << angle - agi._oriAgvPos[2]
        //    << "    AGV初始状态" << agi._oriAgvPos.transpose() << endl;
        for (int tTime = 0; tTime < totalTTimes; tTime++) {
            double xPos = 0., yPos = 0., angleDif = 0.;
            xPos = (tTime + 1) * intervalDistance * cos(angle) + lastPos[0] - agi._oriAgvPos[0];
            yPos = (tTime + 1) * intervalDistance * sin(angle) + lastPos[1] - agi._oriAgvPos[1];
            xPos = (tTime + 1) * intervalDistance * cos(angle) + lastPos[0];
            yPos = (tTime + 1) * intervalDistance * sin(angle) + lastPos[1];
            //Eigen::Vector3d xyDif{ intervalDistance * cos(angle), intervalDistance * sin(angle), 0. };
            //Eigen::Vector3d goalPos{ xPos, yPos, 0. };
            angleDif = angle - agi._oriAgvPos[2];
            angleDif = angleDif > CV_PI ? angleDif - 2 * CV_PI : angleDif;
            angleDif = angleDif < -CV_PI ? angleDif + 2 * CV_PI : angleDif;
            Eigen::Vector3d xyDif{ xPos - agi._oriAgvPos[0], yPos - agi._oriAgvPos[1], angleDif };
            //Eigen::Vector3d goalPos{ 0., 0., agi._oriArmPos[2] };
            Eigen::Vector3d goalPos{ agi._oriArmPos };
            Eigen::Vector3d goalPosRotated = AGVMove2ArmPos(xyDif, goalPos, true, agi._oriAgvPos[2]);
            cout << "旋转、平移为：" << xPos << "  " << yPos << "   插入点坐标：" << agi._oriArmPos.transpose() << "  转换后机械臂中心轨迹：" << goalPosRotated.transpose() << endl;
            vArmMovePosition.emplace_back(goalPosRotated);
        }
        vTest.push_back(vArmMovePosition.size());

        lastPos = currPos; // 更新上一个点位
        lastPos[2] = angle;
        cout << endl << endl << endl;
    }
    //cout << atan2(-1, -1) * 180. / CV_PI << endl;
    return;
}

void ConvertOriMovePos2Curr(const vector<Eigen::Vector3d>& oriPos, vector<Eigen::Vector3d>& currPos) {
    currPos = vector<Eigen::Vector3d>(oriPos);
    for (int i = 0; i < oriPos.size(); i++) {
        Eigen::Vector3d diff = agi._currAgvPos - agi._oriAgvPos;
        Eigen::Vector3d posRT = AGVMove2ArmPos(diff, oriPos[i], false, agi._oriAgvPos[2]);
        currPos[i] = posRT;
    }
}

cv::Mat arm2agv(const double x, const double y, const double theta) {
    return (cv::Mat_<double>(4, 4) <<
        cos(theta), -sin(theta), 0, x,
        sin(theta), cos(theta), 0, y,
        0, 0, 1, 0,
        0, 0, 0, 1
        );
}

cv::Mat arm2agv(const Eigen::Vector3d agv) {
    return (cv::Mat_<double>(4, 4) <<
        cos(agv[2]), -sin(agv[2]), 0, agv[0],
        sin(agv[2]), cos(agv[2]), 0, agv[1],
        0, 0, 1, 0,
        0, 0, 0, 1
        );
}

Eigen::Vector3d arm2agv(const Eigen::Vector3d& agvInfo, const Eigen::Vector3d& armPoint) {
    cv::Mat convertMat = arm2agv(agvInfo);
    //cv::Mat temp = convertMat * (cv::Mat_<double>(4, 1) << armPoint[0], armPoint[1] - 0., armPoint[2], 1);
    cv::Mat temp = convertMat * (cv::Mat_<double>(4, 1) << armPoint[0], armPoint[1] - 100., armPoint[2], 1);
    return Eigen::Vector3d{ temp.at<double>(0,0), temp.at<double>(1,0), temp.at<double>(2,0) };
}

cv::Mat arm2agv(const Eigen::Vector3d& agvInfo, const cv::Mat& armPoint) {
    //cout << "输入坐标点通道、大小：" << armPoint.channels() << "    " << armPoint.size() << endl;
    cv::Mat temp = (cv::Mat_<double>(4, 1) << 0., 0., 0., 1.);
    if (armPoint.size() == cv::Size{ 3, 1 } || armPoint.size() == cv::Size{ 4, 1 }) {
        temp.at<double>(0, 0) = armPoint.at<double>(0, 0);
        temp.at<double>(1, 0) = armPoint.at<double>(1, 0);
        temp.at<double>(2, 0) = armPoint.at<double>(2, 0);
    }
    else if (armPoint.size() == cv::Size{ 4, 4 }) {
        temp.at<double>(0, 0) = armPoint.at<double>(0, 3);
        temp.at<double>(1, 0) = armPoint.at<double>(1, 3);
        temp.at<double>(2, 0) = armPoint.at<double>(2, 3);
    }
    else {
        cout << "通道数不为4*1、4*4或3*1!" << endl;
        return armPoint;
    }
    cv::Mat convertMat = (cv::Mat_<double>(4, 4) <<
        cos(agvInfo[2]), -sin(agvInfo[2]), 0, agvInfo[0],
        sin(agvInfo[2]), cos(agvInfo[2]), 0, agvInfo[1],
        0, 0, 1, 0,
        0, 0, 0, 1
        );
    temp.at<double>(1, 0) -= 100.;
    //temp.at<double>(1, 0) -= 100.;
    return convertMat * temp;
}

/**
    2D坐标点，绕z轴旋转，顺时针为－
    @para 
*/
Eigen::Vector3d PointRzRotate(const Eigen::Vector3d& point, const double& theta, const Eigen::Vector3d& center) {
    Eigen::Vector3d dif = point - center;
    cv::Mat pdMat = (cv::Mat_<double>(3, 1) << dif[0], dif[1], 0);
    cv::Mat rz = (cv::Mat_<double>(3, 3) <<
        cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1
    );
    cv::Mat pdRotMat = rz * pdMat;
    return Eigen::Vector3d{ pdRotMat.at<double>(0,0) + center[0], pdRotMat.at<double>(1,0) + center[1], 0 };
}

vector<Eigen::Vector3d> CalcAgvPosForRGBD(const Eigen::Vector3d& g, double radius, double safedis, double theta, bool display) {
    vector<Eigen::Vector3d> vRes;
    radius += safedis;
    Eigen::Vector2d goal{ g[0], g[1] };
    Eigen::Vector2d centerDir = goal.normalized();
    // 第一个点：起始点和目标点连线与圆上的交点
    Eigen::Vector2d firstPos2D = goal - centerDir * radius;
    Eigen::Vector3d firstPos{ firstPos2D[0], firstPos2D[1], 0 };
    vRes.push_back(firstPos);
    // 绕z轴旋转theta角度以此得到2*theta 3*theta...n*theta
    int times = 360. / theta;
    for (int i = 1; i < times; ++i) {
        Eigen::Vector3d rotated = PointRzRotate(firstPos, (i * theta) * ROBOT_PI / 180., g);
        vRes.push_back(rotated);
    }
 
    if (display) {
        // 在三维空间中显示结果扫查位置
        vector<Eigen::Vector3d> vPoint;
        vector<Eigen::Vector2i> vIdx;
        vPoint.push_back(Eigen::Vector3d{ g[0], g[1], 0 });
        for (int i = 0; i < vRes.size(); ++i) {
            vPoint.emplace_back(vRes[i]);
            vIdx.emplace_back(Eigen::Vector2i{ 0, i + 1 });
        }
        cout << vPoint.size() << "  " << vIdx.size() << endl;
        std::shared_ptr<open3d::geometry::LineSet> lineSet = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vPoint, vIdx));
        //drawAxisInfo dai;
        //dai._center = Eigen::Vector3d{ g[0], g[1], 0 };
        //auto axis = DrawXYZAxisAtOrient(dai);
        open3d::visualization::DrawGeometries({ lineSet, /*axis*/ }, "agv position");
    }

    return vRes;
}

//vector<Eigen::Vector3d> CalcAgvPosForRGBD(const Eigen::Vector3d& g, double radius, double safedis, double theta) {
//    theta = theta * CV_PI / 180.;
//    radius += safedis;
//    Eigen::Vector2d goal{ g[0], g[1] };
//    Eigen::Vector2d centerDir = goal.normalized();
//    //std::cout << "方向向量：" << centerDir.transpose() << endl;
//    Eigen::Vector2d centerPos = goal - centerDir * radius;
//    Eigen::Vector2d centerPos_l1, centerPos_r1;
//    if (centerDir[0] != 0) {
//        double A = -centerDir[0], B = -centerDir[1];
//        double a = 1 + B * B / A / A;
//        double b = -2 * B * cos(theta) / A / A;
//        double c = cos(theta) * cos(theta) / A / A - 1;
//        double y1, y2 = 0.;
//        if ((b * b - 4 * a * c) > 0) {
//            y1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
//            y2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
//            //std::cout << "方程有两个不等的实数根" << endl << "x1=" << y1 << ",x2=" << y2 << endl;
//        }
//        else if ((b * b - 4 * a * c) >= 0) {
//            y1 = -b / (2 * a);
//            //std::cout << "方程有两个相等的实数根" << endl << "x1=x2=" << y1 << endl;
//        }
//        else {
//            //std::cout << "方程无实数根" << endl;
//        }
//        centerPos_l1 = Eigen::Vector2d((cos(theta) - B * y1) / A, y1);
//        centerPos_r1 = Eigen::Vector2d((cos(theta) - B * y2) / A, y2);
//    }
//    else {
//        centerPos_l1 = Eigen::Vector2d(sin(theta), cos(theta));
//        centerPos_r1 = Eigen::Vector2d(-sin(theta), cos(theta));
//    }
//    centerPos_l1 = goal + centerPos_l1 * radius;
//    centerPos_r1 = goal + centerPos_r1 * radius;
//    //std::cout << "中心点为：" << centerPos.transpose() << endl;
//    //std::cout << "左1点为：" << centerPos_l1.transpose() << endl;
//    //std::cout << "右1点为：" << centerPos_r1.transpose() << endl;
//
//    AMRLocalInfo currAmrli;
//    // 读取当前agv相对原始位置
//    RequestAMRLocal(currAmrli);
//    currAmrli._x *= 1000;
//    currAmrli._y *= 1000;
//
//    cv::Mat pos = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos[0], centerPos[1] - 100., agi._worldGoalPos[2], 1.);
//    cv::Mat posl1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_l1[0], centerPos_l1[1] - 100., agi._worldGoalPos[2], 1.);
//    cv::Mat posr1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_r1[0], centerPos_r1[1] - 100., agi._worldGoalPos[2], 1.);
//
//    std::cout << "agv世界坐标中心点为：" << pos.t() << endl;
//    std::cout << "agv世界坐标左1点为：" << posl1.t() << endl;
//    std::cout << "agv世界坐标右1点为：" << posr1.t() << endl;
//    vector<Eigen::Vector3d> vRes;
//    //vRes.emplace_back(Eigen::Vector3d{ centerPos_l1[0], centerPos_l1[1], goal[2] });
//    //vRes.emplace_back(Eigen::Vector3d{ centerPos[0], centerPos[1], goal[2] });
//    //vRes.emplace_back(Eigen::Vector3d{ centerPos_r1[0], centerPos_r1[1], goal[2] });
//    vRes.emplace_back(Eigen::Vector3d{ posl1.at<double>(0, 0), posl1.at<double>(1, 0), posl1.at<double>(2, 0) });
//    vRes.emplace_back(Eigen::Vector3d{ pos.at<double>(0, 0), pos.at<double>(1, 0), pos.at<double>(2, 0) });
//    vRes.emplace_back(Eigen::Vector3d{ posr1.at<double>(0, 0), posr1.at<double>(1, 0), posr1.at<double>(2, 0) });
//
//    return vRes;
//}
