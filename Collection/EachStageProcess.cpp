#include "EachStageProcess.h"
#include "AgvControl.h"
#include "GlobalVariable.h"
#include "RobotAlgorithm.h"
#include "CoordinateTransformation.h"
#include <direct.h>
#include "IOFile.h"
#include "timer.hpp"
#include "OperateImage.h"
#include "algorithm"
#include "MindVision.h"

/**
    交互获得目标信息
    @para goal:目标位置
    @para length:目标长
    @para width:目标宽
*/
void InteractObtainGoalInfo(Eigen::Vector3d& pos, double& length, double& width) {
    // 获取实时RGBD、点云数据，人机交互获取点击目标点
    // 定时器
    Timer* ti = new Timer();
    ti->start(TIMER_TIMING_TIME, TimerInterpute); // 20ms
    cameraImgInfo currImgInfo;
    cv::Point clickPoint = CollectImgAndInteraction(currImgInfo); // 显示、人机交互
    ti->stop();
    ti->~Timer();
    // 推理在算尺寸位置
#if 0
    // 调用python VILD脚本进行推理
    bool ret = CallPythonScriptInference(false);

    // 选择最优bbox
    // 根据鼠标点击点坐标选择最近2Dbbox（VILD）
    vector<int> vPixelInBBoxIndex;
    int validPixelInBBox = -1;
    for (int i = 0; i < vildBBoxImageVec.size(); i++) {
        bool ret = PixelInBBox(v4DBoxPosRect[i], cv::Point{ mouseClickCol, mouseClickRow });
        if (ret) {
            //cout << "第" << i << "个bbox在点击范围内" << endl;
            vPixelInBBoxIndex.emplace_back(i);
        }
        if (vildBBoxImageVec[i].at<uchar>(mouseClickRow, mouseClickCol) == 255) validPixelInBBox = i;
    }
    cout << "有" << vPixelInBBoxIndex.size() << "个BBox有效，最接近bbox为：" << validPixelInBBox << endl;
    if (validPixelInBBox == -1 && vPixelInBBoxIndex.empty()) {
        cout << "数据为空..." << endl;
        return 0;
    }
    else if (validPixelInBBox == -1 && !vPixelInBBoxIndex.empty()) {
        validPixelInBBox = vPixelInBBoxIndex[0];
    }
    cout << "最接近bbox的长宽信息：" << v4DBoxPosRect[validPixelInBBox].width << "   " << v4DBoxPosRect[validPixelInBBox].height << endl;

    // 计算平均深度
    cv::Mat vildDetphImg = cv::imread(vildDepthPath, cv::IMREAD_ANYDEPTH);
    double agvDepth = CountImageAgvDepth(vildDetphImg, vildBBoxImageVec[validPixelInBBox]);

    // 三维重构确定目标大致范围
    std::shared_ptr<open3d::geometry::PointCloud> curr_ptr = std::make_shared<open3d::geometry::PointCloud>();
    *curr_ptr = *ReconstructFromOneImg(currImgInfo, vildBBoxImageVec[validPixelInBBox]);
    open3d::visualization::DrawGeometries({ curr_ptr }, "point cloud");
    open3d::geometry::AxisAlignedBoundingBox bbox = curr_ptr->GetAxisAlignedBoundingBox();
    cout << "bbox info: " << bbox.GetExtent().transpose() << endl;
    Eigen::Vector3d objInfoLWH = bbox.GetExtent();
    Eigen::Vector3d objInfoCenter = bbox.GetCenter();
    //return 0;
#endif // VLID_INFERENCE
    // 直接根据交互位置
    // 计算平均深度
    double agvDepth = currImgInfo._DepthImg.at<ushort>(clickPoint.y, clickPoint.x);
    // AGV导航
    pos = ConvertPixel2World(clickPoint.y, clickPoint.x, currImgInfo, agvDepth);
    //cout << "当前三维重构得到的目标点信息：" << goal[0] << "    " << goal[1] << "    " << goal[2] << endl;
    if (agvDepth < MIN_DISTANCE || agvDepth > MAX_DISTANCE) {
        std::cout << "当前深度值无效!" << endl;
        return;
    }
}

/// <summary>
/// 保存RGBD数据到root下的rgb、depth
/// </summary>
/// <param name="root">目标点</param>
/// <param name="imgIdx">图片保存索引</param>
/// <returns>保留，保存是否成功</returns>
bool SaveRGBDInfo(const string& root, int imgIdx, cameraImgInfo& cii) {
    // 保存RGBD数据
    cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);//640x480
    cv::Mat colorImageDup, depthImageDup;
    if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
        //cout << "读取深度相机图片成功！" << endl;
        flip(colorImage, colorImageDup, 1);
        flip(depthImage, depthImageDup, 1);
        cv::imwrite(root + "rgb/" + to_string(imgIdx) + ".jpg", colorImageDup);
        cv::imwrite(root + "depth/" + to_string(imgIdx) + ".png", depthImageDup);
        cv::Mat pseudoImg = Convert2PseudoColor(depthImageDup);
        cv::imwrite(root + "depth/" + to_string(imgIdx) + "_pseudo.png", pseudoImg);
    }
    // 当前RGBD对应机械臂坐标
    string currStrPose = robotArmHandle->ReadRobotArmPosString();
    fstream fs(root + "rgbd.txt", ios::out | ios::app);
    fs << imgIdx << endl;
    fs << currStrPose << endl;
    // 当前RGBD对应AGV位姿
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    Eigen::Vector3d currImu = robot_status_imu_req(m_SockClient05);
    Eigen::Vector3d currPose{ 1000. * currAmrli._x , 1000. * currAmrli._y, currAmrli._angle };
    fs << currPose[0] << "," << currPose[1] << "," << currPose[2] << "," << currImu[0] << "," << currImu[1] << "," << currImu[2] << endl;

    fs.close();

    // 更新输出数据
    IntegrateImgInfo(colorImageDup, depthImageDup, currStrPose, 1, cii, currPose, currImu);

    return true;
}
bool SaveHDInfo(const string& root, int imgIdx, cameraImgInfo& cii) {
    // 保存HD数据
    string fileName = root + "hdrgb/" + to_string(imgIdx) + ".bmp";
    save_mindvision(const_cast<char*>(fileName.c_str()));
    // 当前RGBD对应机械臂坐标
    string currStrPose = robotArmHandle->ReadRobotArmPosString();
    fstream fs(root + "hdrgb.txt", ios::out | ios::app);
    fs << imgIdx << endl;
    fs << currStrPose << endl;
    // 当前RGBD对应AGV位姿
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    Eigen::Vector3d currImu = robot_status_imu_req(m_SockClient05);
    Eigen::Vector3d currPose{ 1000. * currAmrli._x , 1000. * currAmrli._y, currAmrli._angle };
    fs << currPose[0] << "," << currPose[1] << "," << currPose[2] << "," << currImu[0] << "," << currImu[1] << "," << currImu[2] << endl;

    fs.close();
    // hd rgb不更新cii

    return true;
}

void ReadStage1TxtInfo(const string& root, vector<cameraImgInfo>& vCameraInfo, vector<Eigen::Vector3d>& vAgvInfo, int type = 0) {
    fstream fsread;
    if (0 == type) fsread = fstream(root + "rgbd.txt", ios::in);
    else if (1 == type) fsread = fstream(root + "thermal.txt", ios::in);
    else fsread = fstream(root + "hd.txt", ios::in);
    string currLine;
    vector<double> currAgvInfo; // 当前AGV位置
    vector<Eigen::Vector3d> currImuInfo;
    string currArmInfo; // 当前机械臂位置
    int infoIdx = 0;
    while (getline(fsread, currLine)) {
        //cout << currLine << endl;
        if (currLine.length() <= 2) {
            // 读取标号
            infoIdx = atoi(currLine.c_str());
            continue;
        }
        if (currLine[0] == 'A' && currLine[40] == 'B' && currLine[41] == 'C') {
            // 读取图片对应机械臂位置
            currArmInfo = currLine;
            continue;
        }
        // 读取图片对应AGV位置
        currAgvInfo.clear();
        while (1) {
            int idx = currLine.find(',');
            if (idx == -1) break;
            currAgvInfo.emplace_back(atof(currLine.substr(0, idx).c_str()));
            currLine = currLine.substr(idx + 1);
        }
        currAgvInfo.emplace_back(atof(currLine.c_str()));
        //cout << currAgvInfo[0] << "    " << currAgvInfo[1] << "    " << currAgvInfo[2] << endl;
        if (infoIdx == 0) {
            // agv初始位置
            agi._oriAgvPos = Eigen::Vector3d{ currAgvInfo[0], currAgvInfo[1], currAgvInfo[2] };
        }
        else {
            // 图片对应位置
            vAgvInfo.emplace_back(Eigen::Vector3d{ currAgvInfo[0], currAgvInfo[1], currAgvInfo[2] });
            currImuInfo.emplace_back(Eigen::Vector3d{ currAgvInfo[3], currAgvInfo[4], currAgvInfo[5] });
            cameraImgInfo cii;
            if (0 == type) {
                cv::Mat rgb = cv::imread(root + "rgb/" + to_string(infoIdx) + ".jpg");
                cv::Mat depth = cv::imread(root + "depth/" + to_string(infoIdx) + ".png", cv::IMREAD_ANYDEPTH);
                IntegrateImgInfo(rgb, depth, currArmInfo, 1, cii, vAgvInfo.back(), currImuInfo.back());
            }
            else if (1 == type) {
                cv::Mat thermal = cv::imread(root + "thermal/" + to_string(infoIdx) + ".jpg");
                IntegrateImgInfo(thermal, thermal, currArmInfo, 2, cii, vAgvInfo.back(), currImuInfo.back());
            }
            else {
                cv::Mat hd = cv::imread(root + "hd/" + to_string(infoIdx) + ".jpg");
                IntegrateImgInfo(hd, hd, currArmInfo, 0, cii, vAgvInfo.back(), currImuInfo.back());
            }
            vCameraInfo.emplace_back(cii);
        }
    }
    fsread.close();

#if 0
    for (int i = 0; i < vCameraInfo.size(); i++) {
        cout << vCameraInfo[i]._poseStr << endl;
        cout << vAgvInfo[i].transpose() << endl;
    }
#endif
}

void ReadStage1SLVildInfo(const string& root, const vector<cameraImgInfo>& vCameraInfo, const vector<Eigen::Vector3d>& vAgvInfo, vector<cameraImgInfo>& vRGBDInfoSL, vector<bboxImgInfo>& vRGBDVildInfoSL) {
    // 读取RGBD信息
    for (int i = 0; i < vRGBDInfoSL.size(); i++) {
        //char fileName[10];
        //sprintf_s(fileName, "%04d", i);
        // depth数据
        string depthImagePath = root + "depth/" + to_string(i + 1) + ".png"; // ".png"
        cv::Mat srcDepth = cv::imread(depthImagePath, cv::IMREAD_ANYDEPTH);
        // rgb数据
        string colorImagePath = root + "rgb/" + to_string(i + 1) + ".jpg"; // ".jpg"
        cv::Mat srcColor = cv::imread(colorImagePath);
        // 整合
        cameraImgInfo currCamera;
        IntegrateImgInfo(srcColor, srcDepth, vCameraInfo[i]._poseStr, 1, currCamera);
        vRGBDInfoSL[i] = currCamera;
    }

    // 读取RGBD CLIP推理结果BBox
    for (int i = 0; i < vRGBDVildInfoSL.size(); i++) {
        char fileName[10];
        sprintf_s(fileName, "%04d", i);
        // 目标疑似区域（迭代限制区域），VILD推理结果
        string vildBboxPath = root + "rgb_vild/" + to_string(i + 1) + "/";
        bboxImgInfo bboxInfo;
        IntegrateVildBBoxInfo(vildBboxPath, bboxInfo, 0);
        vRGBDVildInfoSL[i] = bboxInfo;
    }

#if 0

    cout << "***image info:" << endl;
    for (int i = 0; i < vRGBDVildInfoSL.size(); ++i) {
        cout << "pose str:" << vRGBDInfoSL[i]._poseStr << endl;
        cout << "gripper:" << vRGBDInfoSL[i]._Gripper2Base << endl;
        cout << "camera:" << vRGBDInfoSL[i]._CameraPose << endl;
        cout << "agv pose:" << vRGBDInfoSL[i]._agvPose.transpose() << endl;
        cout << "image type:" << vRGBDVildInfoSL[i]._imgType << endl;
        for (int mask = 0; mask < vRGBDVildInfoSL[i]._img.size(); ++mask) {
            cout << "image path:" << vRGBDVildInfoSL[i]._imgPath[mask] << endl;
            cv::imshow("mask", vRGBDVildInfoSL[i]._img[mask]);
            cv::waitKey(0);
        }
    }
    cout << "**********" << endl;

#endif
}

cv::Mat SaveDispersePointImage(const vector<cv::Point>& vPoint) {
    // 将离散点集画在图像上
    cv::Mat dst = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
    // 遍历所有点集中的点
    for (auto point : vPoint) {
        cv::circle(dst, point, 3, cv::Scalar(255), 2);
    }
    return dst.clone();
}

std::shared_ptr<open3d::geometry::PointCloud> SelfLearning(const string& root, const vector<cameraImgInfo>& vCameraInfo, const vector<Eigen::Vector3d>& vAgvInfo, vector<cameraImgInfo>& vRGBDInfoSL, vector<bboxImgInfo>& vRGBDVildInfoSL) {
    /////////////////////////////
    // 自学习迭代起始选取：交互确定bbox在三维世界坐标中位置，并投影到当前视角最近的目标框 
    if (vCameraInfo.size() == 3) {
        vRGBDVildInfoSL[0]._selectmask = vRGBDVildInfoSL[0]._img[4]; // pose_3
        vRGBDVildInfoSL[1]._selectmask = vRGBDVildInfoSL[1]._img[1];
        vRGBDVildInfoSL[2]._selectmask = vRGBDVildInfoSL[2]._img[4];
        //vRGBDVildInfoSL[0]._selectmask = cv::Mat(640, 480, CV_8UC1, cv::Scalar(0)); // pose_3
        //vRGBDVildInfoSL[1]._selectmask = cv::Mat(640, 480, CV_8UC1, cv::Scalar(0.));
        //vRGBDVildInfoSL[2]._selectmask = cv::Mat(640, 480, CV_8UC1, cv::Scalar(0));
    }
    else if (vCameraInfo.size() == 6) {
        ////vRGBDVildInfoSL[0]._selectmask = vRGBDVildInfoSL[0]._img[0];
        ////vRGBDVildInfoSL[1]._selectmask = vRGBDVildInfoSL[1]._img[0];
        ////vRGBDVildInfoSL[2]._selectmask = cv::Mat(640, 480, CV_8UC1, cv::Scalar(0));
        ////vRGBDVildInfoSL[3]._selectmask = cv::Mat(640, 480, CV_8UC1, cv::Scalar(0));
        ////vRGBDVildInfoSL[4]._selectmask = vRGBDVildInfoSL[4]._img[2];
        ////vRGBDVildInfoSL[5]._selectmask = vRGBDVildInfoSL[5]._img[1];
        //vRGBDVildInfoSL[0]._selectmask = vRGBDVildInfoSL[0]._img[0];
        //vRGBDVildInfoSL[1]._selectmask = vRGBDVildInfoSL[1]._img[0];
        //vRGBDVildInfoSL[2]._selectmask = vRGBDVildInfoSL[2]._img[0];
        //vRGBDVildInfoSL[3]._selectmask = vRGBDVildInfoSL[3]._img[1];
        //vRGBDVildInfoSL[4]._selectmask = cv::Mat(640, 480, CV_8UC1, cv::Scalar(0));
        //vRGBDVildInfoSL[5]._selectmask = vRGBDVildInfoSL[5]._img[0];
        vRGBDVildInfoSL[0]._selectmask = vRGBDVildInfoSL[0]._img[0];
        vRGBDVildInfoSL[1]._selectmask = vRGBDVildInfoSL[1]._img[0];
        vRGBDVildInfoSL[2]._selectmask = vRGBDVildInfoSL[2]._img[0];
        vRGBDVildInfoSL[3]._selectmask = vRGBDVildInfoSL[3]._img[0];
        vRGBDVildInfoSL[4]._selectmask = vRGBDVildInfoSL[4]._img[0];
        vRGBDVildInfoSL[5]._selectmask = vRGBDVildInfoSL[5]._img[0];
    }

    // 重构
    vector<shared_ptr<open3d::geometry::PointCloud>> vLastPointCloud(vRGBDInfoSL.size(), nullptr); // 每个视角的局部点云
    std::shared_ptr<open3d::geometry::PointCloud> pclPtrSL = std::make_shared<open3d::geometry::PointCloud>(); // 当前所有局部点云的总和
    for (int i = 0; i < vRGBDInfoSL.size(); i++) {
        reconstructInfo ri;
        //ri._2whichbase = 0x02 | 0x08 | 0x80; // 转换到AGV世界坐标系
        //ri._height = -940.; // 针对性优化
        //ri._2whichbase = 0x02; // 转换到AGV世界坐标系
        ri._2whichbase = 0x02 | 0x08; // 转换到AGV世界坐标系
        ri._mask = vRGBDVildInfoSL[i]._selectmask;
        ri._agvCurrInfo = vAgvInfo[i];
        vLastPointCloud[i] = ReconstructFromOneImg(vRGBDInfoSL[i], ri); // 返回局部点云
        *pclPtrSL += *vLastPointCloud[i]; // 点云拼接
        //open3d::visualization::DrawGeometries({ vLastPointCloud[i] }, "IteratedPointCloud");
    }

    string pcSavePath = CreateDirUseDate(root + "iteration/");

    open3d::io::WritePointCloudToPCD(pcSavePath + "\\beforeSL.pcd", *pclPtrSL);
    open3d::io::WritePointCloudToPLY(pcSavePath + "\\beforeSL.ply", *pclPtrSL);
    open3d::visualization::DrawGeometries({ pclPtrSL }, "IteratedPointCloud");
    //迭代
    std::cout << "当前处理完成点云存放的路径为：" << pcSavePath << endl;
    int iterateNum = 7;
    if (0) // *************
    for (int itr = 0; itr < iterateNum; itr++) {
        std::cout << "***第：" << itr << "次迭代开始（共" << iterateNum << "次迭代）。" << endl;
        for (int i = 0; i < vRGBDInfoSL.size(); i++) {
            // 创建文件夹保存图片
            char dirPath[50];
            int _ret = _mkdir((pcSavePath + "/iteration").c_str());
            sprintf_s(dirPath, "/iteration/%d/", i);
            _ret = _mkdir((pcSavePath + string(dirPath)).c_str());
            // 获取当前相机位置、朝向
            cv::Mat nowPose = vRGBDInfoSL[i]._CameraPose.clone(); // 第i张图片的相机位姿，直接赋值为浅拷贝
            nowPose.at<double>(1, 3) -= 100.;
            nowPose = arm2agv(vAgvInfo[i]) * nowPose;
            // 从当前相机视角将点云投影到2D图像上
            vector<cv::Mat> imageVec = PerspectivePointCloudFromSpecialPose(pclPtrSL, nowPose);
            // 更新原先的掩模maskImageVec
            // 根据深度图去除异常点
            //DealMaskRegion(imageVec);
            vRGBDVildInfoSL[i]._selectmask = imageVec[3].clone(); // 深拷贝，maskImageVec[vIndex[i]] = imageVec[3]：浅拷贝
            vector<vector<cv::Point>> maskContours; // 伪深度图对应掩膜
            //cv::imshow("mask", vRGBDVildInfoSL[i]._selectmask);
            //cv::waitKey(0);
            // 寻找图片掩膜轮廓
            maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 0);
            //if (itr == iterateNum - 1) maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 1);
            //else maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 0);
            // ***保存rgb图片的所有轮廓
            vector<vector<cv::Point>> baseContours = FindContours(vRGBDInfoSL[i]._RgbImg, 0);
            cv::Mat binRgbImgSave(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
            for (int cont = 0; cont < baseContours.size(); ++cont) {
                cv::drawContours(binRgbImgSave, baseContours, cont, cv::Scalar(0, 0, 255), 2);
            }
            // ***保存mask图片的所有轮廓
            cv::Mat binMaskImgSave(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
            for (int cont = 0; cont < maskContours.size(); ++cont) {
                cv::drawContours(binMaskImgSave, maskContours, cont, cv::Scalar(0, 0, 255), 2);
            }
            vector<cv::Point> optimumContourSrc, optimumContour;
            // 求掩膜图片轮廓在原始图片轮廓中最近的一条
            double err = FindOptimumContours(baseContours, maskContours, optimumContourSrc, optimumContour);
            vector<vector<cv::Point>> vOptimumContours;
            vOptimumContours.emplace_back(optimumContour);
            /*20231008保存数据使用*/
            cv::Mat disperseImg = SaveDispersePointImage(optimumContour);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "disperse.jpg", disperseImg);
            //vOptimumContours.emplace_back(optimumContourSrc);
            cv::Mat maskNew(vRGBDInfoSL[i]._RgbImg.rows, vRGBDInfoSL[i]._RgbImg.cols, CV_8UC1, cv::Scalar(0));
            if (vOptimumContours[0].size() != 0) {
                //cv::drawContours(maskNew, vOptimumContours, 0, Scalar(255), -1, 8);
                maskNew = FillImageBasePoint(vOptimumContours[0]);
            }
            cv::Mat maskUse(vRGBDInfoSL[i]._RgbImg.rows, vRGBDInfoSL[i]._RgbImg.cols, CV_8U, cv::Scalar(0));
            vRGBDVildInfoSL[i]._selectmask = maskNew; // 3、全部用理论边界
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "optimum.jpg", maskNew);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "RGBcontour.jpg", binRgbImgSave);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "Maskcontour.jpg", binMaskImgSave);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + ".jpg", imageVec[3]);
            // 重构更新点云 --- 删除上一次结果，保留最新一次结果
            if (vLastPointCloud[i] != nullptr) {
                // 将上一次的点云结果移除
                pclPtrSL = RemoveSmallerPointCloud(pclPtrSL, vLastPointCloud[i]); // 两个点云的差，在一个大点云中去除小的点云
            }
            reconstructInfo ri;
            ri._2whichbase = 0x02 | 0x08; // 转换到AGV世界坐标系、并按掩膜抠图
            ri._mask = vRGBDVildInfoSL[i]._selectmask;
            ri._agvCurrInfo = vAgvInfo[i];
            vLastPointCloud[i] = ReconstructFromOneImg(vRGBDInfoSL[i], ri); // 返回局部点云
            //vLastPointCloud[i] = cloud_ICP(vLastPointCloud[i], vLastPointCloud[0]);
            *pclPtrSL += *vLastPointCloud[i]; // 点云拼接
        }
    }
    // 以第一个视角（点云）进行对齐ICP
    vector<cv::Mat> vAmendM(vRGBDInfoSL.size(), cv::Mat::eye(4, 4, CV_64F));
    pclPtrSL = vLastPointCloud[0];
    for (int i = 1; i < vRGBDInfoSL.size(); i++) {
        vAmendM[i] = PointCloudICP(vLastPointCloud[i], vLastPointCloud[0]);
        //if (3 == i) continue; // 
        *pclPtrSL += *vLastPointCloud[i]; // 点云拼接
    }
    open3d::visualization::DrawGeometries({ pclPtrSL }, "IteratedPointCloud");
    //// rgbd融合
    //for (int i = 0; i < (*pclPtrSL).points_.size(); ++i) {
    //    Eigen::Vector3d pAgv = (*pclPtrSL).points_[i];
    //    Eigen::Vector3d colorCamera = GetColorFromImageVec(pAgv, vCameraInfo, vAmendM, true);
    //    (*pclPtrSL).colors_[i] = colorCamera;
    //}
    //open3d::visualization::DrawGeometries({ pclPtrSL }, "ronghe");

    // 保存数据
    fstream fs(pcSavePath + "\\readme.txt", ios::out | ios::app);
    fs << endl << "*迭代完成!" << endl;
    fs.close();
    open3d::io::WritePointCloudToPCD(pcSavePath + "\\shipIteratedICP.pcd", *pclPtrSL);
    open3d::io::WritePointCloudToPLY(pcSavePath + "\\shipIteratedICP.ply", *pclPtrSL);
    open3d::io::WritePointCloudToPCD(root + "\\shipIteratedICP.pcd", *pclPtrSL);
    open3d::io::WritePointCloudToPLY(root + "\\shipIteratedICP.ply", *pclPtrSL);
    open3d::visualization::DrawGeometries({ pclPtrSL }, "IteratedPointCloud");

    return pclPtrSL;
}

vector<cv::Mat> CalcAmendMatrixEachAgv(shared_ptr<open3d::geometry::PointCloud>& pclPtr, const vector<cameraImgInfo>& vRgbdInfo, const vector<Eigen::Vector3d>& vAgvInfoRgbd, const vector<bboxImgInfo>& vRGBDVildInfoSL) {
    int useNum = vRgbdInfo.size();
    cout << "total image number:" << useNum << endl;
    vector<cv::Mat> vAmendAgv(useNum);
    ///*******************************************/
    vector<cv::Mat> vMask(useNum);
    if (8 == vRgbdInfo.size()) {
        vMask[0] = vRGBDVildInfoSL[0]._img[0];
        vMask[1] = vRGBDVildInfoSL[1]._img[0];
        vMask[2] = vRGBDVildInfoSL[2]._img[0];
        vMask[3] = vRGBDVildInfoSL[3]._img[1];
        vMask[4] = vRGBDVildInfoSL[4]._img[0];
        vMask[5] = vRGBDVildInfoSL[5]._img[0];
        vMask[6] = vRGBDVildInfoSL[6]._img[0];
        vMask[7] = vRGBDVildInfoSL[7]._img[0];
    }
    else {
        //vMask[0] = vRGBDVildInfoSL[0]._img[0];
        //vMask[1] = vRGBDVildInfoSL[1]._img[1];
        //vMask[2] = vRGBDVildInfoSL[2]._img[0];
        //vMask[3] = vRGBDVildInfoSL[3]._img[0];
        //vMask[4] = vRGBDVildInfoSL[4]._img[0];
        //vMask[5] = vRGBDVildInfoSL[5]._img[0];
        vMask[0] = vRGBDVildInfoSL[0]._img[0];
        vMask[1] = vRGBDVildInfoSL[1]._img[0];
        //vMask[0] = vRGBDVildInfoSL[0]._img[0];
        //vMask[1] = vRGBDVildInfoSL[1]._img[0];
        //vMask[2] = vRGBDVildInfoSL[2]._img[1];
        //vMask[3] = vRGBDVildInfoSL[3]._img[0];
        //vMask[4] = vRGBDVildInfoSL[4]._img[0];
        //vMask[5] = vRGBDVildInfoSL[5]._img[0];
        //vMask[6] = vRGBDVildInfoSL[6]._img[0];
        //////vMask[0] = vRGBDVildInfoSL[0]._img[0];
        //////vMask[1] = vRGBDVildInfoSL[1]._img[0];
        //////vMask[2] = vRGBDVildInfoSL[2]._img[1];
        //////vMask[3] = vRGBDVildInfoSL[3]._img[0];
        //////vMask[4] = vRGBDVildInfoSL[4]._img[0];
        //////vMask[5] = vRGBDVildInfoSL[5]._img[0];
        //////vMask[6] = vRGBDVildInfoSL[6]._img[0];
        ////vMask[0] = vRGBDVildInfoSL[0]._img[0];
        ////vMask[1] = vRGBDVildInfoSL[1]._img[1];
        ////vMask[2] = vRGBDVildInfoSL[2]._img[0];
        ////vMask[3] = vRGBDVildInfoSL[3]._img[0];
        ////vMask[4] = vRGBDVildInfoSL[4]._img[0];
        //cv::imshow("mask", vMask[0]);
        //cv::waitKey(0);
        //cout << "-----------------" << endl;
    }
    for (int i = 0; i < useNum; ++i) {
        cout << "******curr pic:" << i << endl;
        cv::Mat converted = arm2agv(vAgvInfoRgbd[i]) * vRgbdInfo[i]._CameraPose;
        vector<cv::Mat> pesudoMask = PerspectivePointCloudFromSpecialPose(pclPtr, converted);
        //cv::imshow("src", pesudoMask[3]);
        //cv::waitKey(0);
        // 局部点云直接和全局点云匹配精度不高->全局投影到局部生成掩膜进行抠图
        // 按伪标签掩膜进行选择
        reconstructInfo ri;
        //ri._2whichbase |= 0x02;
        //ri._2whichbase |= 0x02 | 0x08;
        //ri._mask = pesudoMask[3];
        ri._2whichbase |= 0x02 | 0x08;
        ri._mask = vMask[i];
        ri._agvCurrInfo = vAgvInfoRgbd[i];
        shared_ptr<open3d::geometry::PointCloud> localPtr = ReconstructFromOneImg(vRgbdInfo[i], ri);
        //localPtr = RemoveButtonPoints(localPtr);
        localPtr->PaintUniformColor(Eigen::Vector3d{ 0.,1.,0. });
        open3d::visualization::DrawGeometries({ pclPtr, localPtr }, "src");

        // ICP将局部点云匹配到全局点云->修正AGV误差
        //cloud_ICP(localPtr, pclPtr);
        vAmendAgv[i] = PointCloudICP(localPtr, pclPtr);
        open3d::visualization::DrawGeometries({ pclPtr, localPtr }, "src");
    }
    return vAmendAgv;
}

/// <summary>
/// 从点云获取设备扫查信息ThermalScanInfo
/// </summary>
/// <param name="ptr">只包含目标的点云</param>
/// <param name="vScanInfo"></param>
/// <param name="display"></param>
void ObtainPartitionFromPointCloud(const shared_ptr<open3d::geometry::PointCloud>& ptr, vector<ThermalScanInfo>& vScanInfo, bool display) {
    //// 整体点云分块
    PointCloudInfo currPcli;
    InitPointCloudInfo(ptr, currPcli);
    vector<std::shared_ptr<open3d::geometry::PointCloud>> vCropedCloudForPCA = PointCloudPartition(ptr); // 存放所有裁剪过的点云
    vector<std::shared_ptr<open3d::geometry::LineSet>> vNormOrient = PointCloudPartitionPCA(currPcli, vCropedCloudForPCA, vScanInfo);
    //// 点云合并、显示，直接+=也可以
    //int pointNum = 0;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> vTotalCloud;
    ////shared_ptr<open3d::geometry::PointCloud> downsampledTotal = totalPointCloud->VoxelDownSample(5);//对点云进行下采样
    shared_ptr<open3d::geometry::PointCloud> downsampledTotal = ptr->VoxelDownSample(5);//对点云进行下采样
    for (int i = 0; i < vNormOrient.size(); i++) {
        vTotalCloud.emplace_back(vNormOrient[i]); // 点云整合一起显示
        // 获取设备3D轮廓
        std::shared_ptr<open3d::geometry::LineSet> device_pointcloud = DrawDeviceLinePointCloud(vScanInfo[i]._cameraPosition, vScanInfo[i]._cameraRx, vScanInfo[i]._cameraRy, vScanInfo[i]._cameraRz);
        vTotalCloud.emplace_back(device_pointcloud);
    }
    vTotalCloud.emplace_back(ptr);
    if (display) open3d::visualization::DrawGeometries({ vTotalCloud }, "CloudWithDir");
}

void RobotAdjustThermal2Goal(cv::Mat gripperPoseMat, int goalPos, const string& root) {
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    cv::Mat arm2agvM = arm2agv(Eigen::Vector3d{ 1000. * currAmrli._x, 1000. * currAmrli._y, currAmrli._angle });
    ////AGV当前位置使用机械臂扫查对应的一个分块
    //基座位姿
    cv::Mat goalarm = arm2agvM.inv() * gripperPoseMat;
    goalarm.at<double>(1, 3) = goalarm.at<double>(1, 3) + 100;
    cout << "goalarm info：" << goalarm << endl;
    string cmdStr;
    ConvertMat2String(goalarm, cmdStr);
    std::cout << "调整后末端位姿：" << cmdStr << endl;
    robotArmHandle->Move2OnePoint(cmdStr, 1);
    std::cout << "***[3]请激励热像仪进行采集：" << endl;
    string str;
    cin >> str;

    // 读取当前agv位姿及IMU
    AMRLocalInfo currAmrliAdjusted;
    RequestAMRLocal(currAmrliAdjusted);
    currAmrliAdjusted._x *= 1000.;
    currAmrliAdjusted._y *= 1000.;
    Eigen::Vector3d currImu = robot_status_imu_req(m_SockClient05);
    // 读取当前机械臂坐标Thermal
    string currStrPose = robotArmHandle->ReadRobotArmPosString();

    //保存末端位姿，imu数据
    fstream fs(root + "thermal.txt", std::ios::out | std::ios::app);
    fs << currStrPose << endl;
    fs << currAmrliAdjusted._x << "," << currAmrliAdjusted._y << "," << currAmrliAdjusted._angle << "," << currImu[0] << "," << currImu[1] << "," << currImu[2] << endl;
    fs.close();
}

void CalcThermalPoseAndScan(const shared_ptr<open3d::geometry::PointCloud>& ptr, const vector<ThermalScanInfo>& vScanInfo, const string& root) {
    open3d::geometry::OrientedBoundingBox obb = ptr->GetOrientedBoundingBox();
    Eigen::Vector3d obbCenter = obb.GetCenter();
    fstream fs(root + "thermal.txt", std::ios::out | std::ios::trunc);// 清空之前数据
    fs.close();
    string robotArmPoseMid1 = "A+015000-127299-000008+15020-08999-06020BC"; // 机械臂中间复位点->零点
    string robotArmPoseMid2 = "A+015000-106278+017920+17999-00584-08999BC"; // 机械臂中间复位点->垂直向下
    string robotArmPoseOri = "A+073125-018311+025299-07470+08569-07860BC"; // AGV运动机械臂复位点
    //for (int goalPos = 3; goalPos < vScanInfo.size(); goalPos++) {

    for (int goalPos = 1; goalPos < vScanInfo.size(); goalPos++) {
        //寻找每一个分块的最佳AGV位置vScanInfo
        Eigen::Vector3d centerTgoal = (vScanInfo[goalPos]._gripperPosition - obbCenter);
        centerTgoal[2] = 0;//忽略高度
        centerTgoal = centerTgoal.normalized();
        Eigen::Vector3d currAgvGoal;
        currAgvGoal = vScanInfo[goalPos]._gripperPosition + centerTgoal * 1200;//AGV原离目标点1.2m
        cout << "当前AGV目标点：" << currAgvGoal.transpose() << endl;
        cout << "保存信息机械臂位姿为：" << vScanInfo[goalPos]._cmdStr << endl;
        cout << "保存信息agv位姿为：" << obbCenter.transpose() << endl;
        //continue;
        //Move2Goal(Eigen::Vector2d{ obbCenter[0], obbCenter[1] }, Eigen::Vector2d{ currAgvGoal[0], currAgvGoal[1] });
        MoveAGV2OneGoal(currAgvGoal, true, obbCenter);
        AMRRotation(PI / 2., 1., m_SockClient06); // 旋转90度右臂朝向物体
        AMRLibration(0.2, -0.2, 0.0, m_SockClient06, 1); // x倒退0.2m
        //continue;
        std::cout << "***[1]AGV以就位，输入调整机械臂位姿：" << endl;
        string str;
        cin >> str;

        //移动机械臂到达指定点位。方便到达PCA指定位置
        robotArmHandle->Move2OnePoint(robotArmPoseMid1, 1);
        robotArmHandle->Move2OnePoint(robotArmPoseMid2, 1);
        // *******调整rgbd末端位姿采集局部点云，用于匹配到全局点云修正AGV运动误差
        //RobotArm2Goal(agvGoalEig); //******
        // 读取当前agv位姿
        cameraImgInfo cii;
        SaveRGBDInfo(root, goalPos + 1, cii);
        std::cout << "***[2]已采集RGBD数据输入调整热像仪位姿：" << endl;
        cin >> str;
        // AGV到达目标点，调整机械臂使thermal指向目标
        //if (goalPos != 3 && goalPos != 4 && goalPos != 5) RobotAdjustThermal2Goal(vScanInfo[goalPos]._gripperPoseMat, goalPos, root);
        //回到指定点位
        robotArmHandle->Move2OnePoint(robotArmPoseMid2, 1);
        robotArmHandle->Move2OnePoint(robotArmPoseMid1, 1); // 热像仪采集复位点
        cout << "-----------------------\n\n\n";
        robotArmHandle->Move2OnePoint(robotArmPoseOri, 1);//AGV运动机械臂复位点
    }
}

std::shared_ptr<open3d::geometry::PointCloud> RemoveButtonPoints(const std::shared_ptr<open3d::geometry::PointCloud>& pcl) {
    std::shared_ptr<open3d::geometry::PointCloud> pclRet = std::make_shared<open3d::geometry::PointCloud>();
    //cout << "min bound pos:" << pcl->GetMinBound().transpose() << endl;
    //cout << "max bound pos:" << pcl->GetMaxBound().transpose() << endl;
    auto bbox = pcl->GetOrientedBoundingBox();
    //cout << "max extent pos:" << bbox.extent_.transpose() << endl;
    Eigen::Vector3d minBound = pcl->GetMinBound();
    for (int idx = 0; idx < pcl->points_.size(); ++idx) {
        Eigen::Vector3d pd = pcl->points_[idx];
        if (pd[2] > minBound[2] + 180) {
            pclRet->points_.push_back(pcl->points_[idx]);
            pclRet->colors_.push_back(pcl->colors_[idx]);
        }
    }
    
    return pclRet;
}

cv::Mat Convert2PseudoColor(const cv::Mat& depthImage) {
    //cv::Mat pseudoColorImage;
    //// 将16位深度图像转换为8位灰度图像
    //cv::Mat grayImage;
    //depthImage.convertTo(grayImage, CV_8U, 1.0 / 256.);
    //// 应用伪彩色映射
    //cv::applyColorMap(grayImage, pseudoColorImage, cv::COLORMAP_JET);

    cv::Mat depth;
    depthImage.convertTo(depth, CV_64F);
    // 读取16位深度图（像素范围0～65535），并将其转化为8位（像素范围0～255）
    double minEle1 = *min_element(depth.begin<double>(), depth.end<double>());
    depth -= minEle1;
    double minEle = *min_element(depth.begin<double>(), depth.end<double>());
    double maxEle = *max_element(depth.begin<double>(), depth.end<double>());
    depth = depth / (double)(maxEle - minEle);
    depth *= 255.;
    // 使得越近的地方深度值越大，越远的地方深度值越小，以达到伪彩色图近蓝远红的目的。
    depth = 255. - depth;
    cv::Mat gary;
    depth.convertTo(gary, CV_8UC1);
    //cout << gary.type() << endl;
    cv::Mat pseudoColorImage;
    cv::applyColorMap(gary, pseudoColorImage, cv::COLORMAP_JET);

    return pseudoColorImage;
}

void DisplayRealTimeRGBD() {
    Timer* ti = new Timer();
    ti->start(TIMER_TIMING_TIME, TimerInterpute); // 20ms
    extern bool refreshImgFlag;
    cv::Mat colorImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
    cv::Mat depthImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);
    while (1) {
        if (refreshImgFlag) {
            refreshImgFlag = false;
            // 保存实时RGBD图片并显示两个视图
            if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
                cv::flip(depthImage, depthImage, 1);
                cv::Mat pseudoImg = Convert2PseudoColor(depthImage);
                cv::imshow("depth image", pseudoImg); // 更新显示
            }
        }        
        // 按下Esc键退出循环
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    ti->stop();
    ti->~Timer();
}

/// <summary>
/// 交互结果->计算采集点位->控制AGV运动到采集点位->调整机械臂指向目标->采集->保存
/// </summary>
/// <param name="goal">机械臂目标点</param>
void Stage1Process(const Eigen::Vector3d& armGoal) {
    // 读取当前agv位姿，转换机械臂下目标到AGV世界点
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    //agi._oriAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y,currAmrli._angle };
    cv::Mat agvGoal = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << armGoal[0], armGoal[1] - 100., armGoal[2], 1);
    Eigen::Vector3d agvGoalEig{ agvGoal.at<double>(0, 0),agvGoal.at<double>(1, 0),agvGoal.at<double>(2, 0) };
    // 计算采集点位：规划RGBD测量场、显示
    vector<Eigen::Vector3d> vGoal = CalcAgvPosForRGBD(agvGoalEig, 300, 1200, 60, false);
    // 机械臂位姿复位
    RobotArmReset();
    //thread reconThread(UpdateRealTimePointCloud); // 子线程实时判断当前RGBD是否和路径冲突
    //reconThread.detach();
    // 控制AGV运动到采集点位
    string root = "./results/stage_1/"; // 数据路径
    // *********清空之前数据，保存初始位姿（交互时的位姿）
    fstream fs(root + "rgbd.txt", ios::out | ios::trunc);
    fs << 0 << endl;
    fs << currAmrli._x << "," << currAmrli._y << "," << currAmrli._angle << endl;
    fs.close();
    std::shared_ptr<open3d::geometry::PointCloud> pcl = std::make_shared<open3d::geometry::PointCloud>(); // 全局点云
    for (int goalIdx = 0; goalIdx < vGoal.size(); goalIdx++) {
        cout << "arm goal:" << armGoal.transpose() << endl;
        cout << "ture goal:" << agvGoalEig.transpose() << endl;
        cout << "set goal:" << vGoal[goalIdx].transpose() << endl;
        //continue;
        //continue;
        // 移动到第i个目标
        MoveAGV2OneGoal(vGoal[goalIdx], true, agvGoalEig);
        //Move2Goal(Eigen::Vector2d{ agvGoalEig[0], agvGoalEig[1] }, Eigen::Vector2d{ vGoal[goalIdx][0], vGoal[goalIdx][1] });
        // 相机位姿指向目标
        //RobotArm2Goal(agvGoalEig);
        RobotArm2GoalUpdate(agvGoalEig);
        //DisplayRealTimeRGBD();
        //string str;
        //cin >> str;
        // 保存RGBD数据
        cameraImgInfo cii;
        SaveRGBDInfo(root, goalIdx + 1, cii);
        // hd相机位姿指向目标
        //cout << "below infomation is adjust hd camera!" << endl;
        RobotArm2GoalUpdate(agvGoalEig, 1);
        string str;
        cin >> str;
        SaveHDInfo(root, goalIdx + 1, cii);
        // 拼接点云        
        reconstructInfo ri;
        ri._2whichbase |= 0x02; // 转换到AGV世界坐标系
        ri._agvCurrInfo = cii._agvPose;
        *pcl += *ReconstructFromOneImg(cii, ri);
    }

    open3d::io::WritePointCloudToPCD(root + "pointcloud.pcd", *pcl);
    open3d::io::WritePointCloudToPLY(root + "pointcloud.ply", *pcl);
    open3d::visualization::DrawGeometries({ pcl }, "pcl");
}

void Stage2Process() {
    // 读取txt数据
    string dataPath = "./results/stage_1/";
    vector<cameraImgInfo> vCameraInfo;
    vector<Eigen::Vector3d> vAgvInfo;
    ReadStage1TxtInfo(dataPath, vCameraInfo, vAgvInfo);
    // 读取vild推理信息
    vector<cameraImgInfo> vRGBDInfoSL(vCameraInfo.size());
    vector<bboxImgInfo> vRGBDVildInfoSL(vCameraInfo.size());
    ReadStage1SLVildInfo(dataPath, vCameraInfo, vAgvInfo, vRGBDInfoSL, vRGBDVildInfoSL);
    // SL 
    string savePath = "./results/stage_2/";
    std::shared_ptr<open3d::geometry::PointCloud> pclPtrSL;
    if (1) {
        pclPtrSL = SelfLearning(savePath, vCameraInfo, vAgvInfo, vRGBDInfoSL, vRGBDVildInfoSL);
    }
    else {
        open3d::geometry::PointCloud pd;
        open3d::io::ReadPointCloudFromPLY(savePath + "shipIteratedICP.ply", pd);
        pclPtrSL = std::make_shared<open3d::geometry::PointCloud>(pd);
    }
    // 去除最底层地面
    //pclPtrSL = RemoveButtonPoints(pclPtrSL);
    open3d::io::WritePointCloudToPLY(savePath + "1.ply", *pclPtrSL);
    // 点云分快划分末端扫查信息
    vector<ThermalScanInfo> vScanInfo;
    ObtainPartitionFromPointCloud(pclPtrSL, vScanInfo);
    //return;
    // 遍历每一个点寻找最佳AGV位姿，然后反解机械臂末端位姿
    CalcThermalPoseAndScan(pclPtrSL, vScanInfo, savePath);
}

void Stage3Process() {
    string inputPath = "./results/stage_2/";
    string outputPath = "./results/stage_3/";
    // 读取全局点云stage2 SL结果
    open3d::geometry::PointCloud pcl;
    open3d::io::ReadPointCloudFromPLY(inputPath + "shipIteratedICP.ply", pcl);
    std::shared_ptr<open3d::geometry::PointCloud> pclPtr = make_shared<open3d::geometry::PointCloud>(pcl);
    //open3d::visualization::DrawGeometries({ pclPtr }, "src");
    // 读取局部点云rgbd
    vector<cameraImgInfo> vRgbdInfo;
    vector<Eigen::Vector3d> vAgvInfoRgbd;
    ReadStage1TxtInfo(inputPath, vRgbdInfo, vAgvInfoRgbd);
    // 读取Thermal
    vector<cameraImgInfo> vThermalInfo;
    vector<Eigen::Vector3d> vAgvInfoThermal;
    ReadStage1TxtInfo(inputPath, vThermalInfo, vAgvInfoThermal, 1);
    // 读取HD数据// 读取txt数据
    // 读取vild推理信息
    int useNum = vRgbdInfo.size();
    vector<cameraImgInfo> vRGBDInfoSL(useNum);
    vector<bboxImgInfo> vRGBDVildInfoSL(useNum);
    ReadStage1SLVildInfo(inputPath, vRgbdInfo, vAgvInfoRgbd, vRGBDInfoSL, vRGBDVildInfoSL);
    // 从工控机读取的热像仪图片是镜像的
    cout << "total visual num: " << useNum << endl;
    for (int i = 0; i < useNum; ++i) {
        cv::flip(vThermalInfo[i]._RgbImg, vThermalInfo[i]._RgbImg, -1);
    }
    cout << "vThermalInfo.size:" << vThermalInfo.size() << "    " << vAgvInfoRgbd.size() << endl;
#if 0
    for (int i = 0; i < vThermalInfo.size(); i++) {
        //cout << vAgvInfoThermal[i].transpose() << endl;
        cout << vThermalInfo[i]._poseStr << endl;
        cout << vThermalInfo[i]._agvPose.transpose() << "   " << vThermalInfo[i]._agvIMU.transpose() << endl;
        //cout << vThermalInfo[i]._CameraPose << endl;
        //cout << vThermalInfo[i]._Gripper2Base << endl;
        //cout << vAgvInfoRgbd[i].transpose() << endl;
        cout << vRgbdInfo[i]._poseStr << endl;
        cout << vRgbdInfo[i]._agvPose.transpose() << "  " << vRgbdInfo[i]._agvIMU.transpose() << endl;
        //cv::imshow("rgbd_" + to_string(i), vRgbdInfo[i]._RgbImg);
        //cv::imshow("thermal_" + to_string(i), vThermalInfo[i]._RgbImg);
        //cv::waitKey(0);
    }
    return;
#endif
    //pclPtr = RemoveButtonPoints(pclPtr);
    open3d::io::WritePointCloudToPLY(outputPath + "rgbd_color.ply", *pclPtr);
    //// 计算每个thermal下Agv位姿修正后的旋转平移矩阵
    vector<cv::Mat> vAmendM = CalcAmendMatrixEachAgv(pclPtr, vRgbdInfo, vAgvInfoRgbd, vRGBDVildInfoSL);
    //vector<cv::Mat> vAmendM(useNum);
    //vThermalInfo.resize(1);
    // 将修正AGV位姿后进行融合上色
    for (int i = 0; i < (*pclPtr).points_.size(); ++i) {
        Eigen::Vector3d pAgv = (*pclPtr).points_[i];
        //Eigen::Vector3d colorCamera = GetColorFromImageVec(pAgv, vRgbdInfo, vAmendM);
        //Eigen::Vector3d colorCamera = GetColorFromImageVec(pAgv, vRgbdInfo, vAmendM, true);
        Eigen::Vector3d colorCamera = GetColorFromImageVec(pAgv, vThermalInfo, vAmendM, true);
        (*pclPtr).colors_[i] = colorCamera;
    }
    open3d::io::WritePointCloudToPLY(outputPath + "shipPesudo.ply", *pclPtr);
    open3d::io::WritePointCloudToPCD(outputPath + "shipPesudo.pcd", *pclPtr);
    open3d::visualization::DrawGeometries({ pclPtr }, "src");
}