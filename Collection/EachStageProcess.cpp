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
    �������Ŀ����Ϣ
    @para goal:Ŀ��λ��
    @para length:Ŀ�곤
    @para width:Ŀ���
*/
void InteractObtainGoalInfo(Eigen::Vector3d& pos, double& length, double& width) {
    // ��ȡʵʱRGBD���������ݣ��˻�������ȡ���Ŀ���
    // ��ʱ��
    Timer* ti = new Timer();
    ti->start(TIMER_TIMING_TIME, TimerInterpute); // 20ms
    cameraImgInfo currImgInfo;
    cv::Point clickPoint = CollectImgAndInteraction(currImgInfo); // ��ʾ���˻�����
    ti->stop();
    ti->~Timer();
    // ��������ߴ�λ��
#if 0
    // ����python VILD�ű���������
    bool ret = CallPythonScriptInference(false);

    // ѡ������bbox
    // ���������������ѡ�����2Dbbox��VILD��
    vector<int> vPixelInBBoxIndex;
    int validPixelInBBox = -1;
    for (int i = 0; i < vildBBoxImageVec.size(); i++) {
        bool ret = PixelInBBox(v4DBoxPosRect[i], cv::Point{ mouseClickCol, mouseClickRow });
        if (ret) {
            //cout << "��" << i << "��bbox�ڵ����Χ��" << endl;
            vPixelInBBoxIndex.emplace_back(i);
        }
        if (vildBBoxImageVec[i].at<uchar>(mouseClickRow, mouseClickCol) == 255) validPixelInBBox = i;
    }
    cout << "��" << vPixelInBBoxIndex.size() << "��BBox��Ч����ӽ�bboxΪ��" << validPixelInBBox << endl;
    if (validPixelInBBox == -1 && vPixelInBBoxIndex.empty()) {
        cout << "����Ϊ��..." << endl;
        return 0;
    }
    else if (validPixelInBBox == -1 && !vPixelInBBoxIndex.empty()) {
        validPixelInBBox = vPixelInBBoxIndex[0];
    }
    cout << "��ӽ�bbox�ĳ�����Ϣ��" << v4DBoxPosRect[validPixelInBBox].width << "   " << v4DBoxPosRect[validPixelInBBox].height << endl;

    // ����ƽ�����
    cv::Mat vildDetphImg = cv::imread(vildDepthPath, cv::IMREAD_ANYDEPTH);
    double agvDepth = CountImageAgvDepth(vildDetphImg, vildBBoxImageVec[validPixelInBBox]);

    // ��ά�ع�ȷ��Ŀ����·�Χ
    std::shared_ptr<open3d::geometry::PointCloud> curr_ptr = std::make_shared<open3d::geometry::PointCloud>();
    *curr_ptr = *ReconstructFromOneImg(currImgInfo, vildBBoxImageVec[validPixelInBBox]);
    open3d::visualization::DrawGeometries({ curr_ptr }, "point cloud");
    open3d::geometry::AxisAlignedBoundingBox bbox = curr_ptr->GetAxisAlignedBoundingBox();
    cout << "bbox info: " << bbox.GetExtent().transpose() << endl;
    Eigen::Vector3d objInfoLWH = bbox.GetExtent();
    Eigen::Vector3d objInfoCenter = bbox.GetCenter();
    //return 0;
#endif // VLID_INFERENCE
    // ֱ�Ӹ��ݽ���λ��
    // ����ƽ�����
    double agvDepth = currImgInfo._DepthImg.at<ushort>(clickPoint.y, clickPoint.x);
    // AGV����
    pos = ConvertPixel2World(clickPoint.y, clickPoint.x, currImgInfo, agvDepth);
    //cout << "��ǰ��ά�ع��õ���Ŀ�����Ϣ��" << goal[0] << "    " << goal[1] << "    " << goal[2] << endl;
    if (agvDepth < MIN_DISTANCE || agvDepth > MAX_DISTANCE) {
        std::cout << "��ǰ���ֵ��Ч!" << endl;
        return;
    }
}

/// <summary>
/// ����RGBD���ݵ�root�µ�rgb��depth
/// </summary>
/// <param name="root">Ŀ���</param>
/// <param name="imgIdx">ͼƬ��������</param>
/// <returns>�����������Ƿ�ɹ�</returns>
bool SaveRGBDInfo(const string& root, int imgIdx, cameraImgInfo& cii) {
    // ����RGBD����
    cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);//640x480
    cv::Mat colorImageDup, depthImageDup;
    if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
        //cout << "��ȡ������ͼƬ�ɹ���" << endl;
        flip(colorImage, colorImageDup, 1);
        flip(depthImage, depthImageDup, 1);
        cv::imwrite(root + "rgb/" + to_string(imgIdx) + ".jpg", colorImageDup);
        cv::imwrite(root + "depth/" + to_string(imgIdx) + ".png", depthImageDup);
        cv::Mat pseudoImg = Convert2PseudoColor(depthImageDup);
        cv::imwrite(root + "depth/" + to_string(imgIdx) + "_pseudo.png", pseudoImg);
    }
    // ��ǰRGBD��Ӧ��е������
    string currStrPose = robotArmHandle->ReadRobotArmPosString();
    fstream fs(root + "rgbd.txt", ios::out | ios::app);
    fs << imgIdx << endl;
    fs << currStrPose << endl;
    // ��ǰRGBD��ӦAGVλ��
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    Eigen::Vector3d currImu = robot_status_imu_req(m_SockClient05);
    Eigen::Vector3d currPose{ 1000. * currAmrli._x , 1000. * currAmrli._y, currAmrli._angle };
    fs << currPose[0] << "," << currPose[1] << "," << currPose[2] << "," << currImu[0] << "," << currImu[1] << "," << currImu[2] << endl;

    fs.close();

    // �����������
    IntegrateImgInfo(colorImageDup, depthImageDup, currStrPose, 1, cii, currPose, currImu);

    return true;
}
bool SaveHDInfo(const string& root, int imgIdx, cameraImgInfo& cii) {
    // ����HD����
    string fileName = root + "hdrgb/" + to_string(imgIdx) + ".bmp";
    save_mindvision(const_cast<char*>(fileName.c_str()));
    // ��ǰRGBD��Ӧ��е������
    string currStrPose = robotArmHandle->ReadRobotArmPosString();
    fstream fs(root + "hdrgb.txt", ios::out | ios::app);
    fs << imgIdx << endl;
    fs << currStrPose << endl;
    // ��ǰRGBD��ӦAGVλ��
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    Eigen::Vector3d currImu = robot_status_imu_req(m_SockClient05);
    Eigen::Vector3d currPose{ 1000. * currAmrli._x , 1000. * currAmrli._y, currAmrli._angle };
    fs << currPose[0] << "," << currPose[1] << "," << currPose[2] << "," << currImu[0] << "," << currImu[1] << "," << currImu[2] << endl;

    fs.close();
    // hd rgb������cii

    return true;
}

void ReadStage1TxtInfo(const string& root, vector<cameraImgInfo>& vCameraInfo, vector<Eigen::Vector3d>& vAgvInfo, int type = 0) {
    fstream fsread;
    if (0 == type) fsread = fstream(root + "rgbd.txt", ios::in);
    else if (1 == type) fsread = fstream(root + "thermal.txt", ios::in);
    else fsread = fstream(root + "hd.txt", ios::in);
    string currLine;
    vector<double> currAgvInfo; // ��ǰAGVλ��
    vector<Eigen::Vector3d> currImuInfo;
    string currArmInfo; // ��ǰ��е��λ��
    int infoIdx = 0;
    while (getline(fsread, currLine)) {
        //cout << currLine << endl;
        if (currLine.length() <= 2) {
            // ��ȡ���
            infoIdx = atoi(currLine.c_str());
            continue;
        }
        if (currLine[0] == 'A' && currLine[40] == 'B' && currLine[41] == 'C') {
            // ��ȡͼƬ��Ӧ��е��λ��
            currArmInfo = currLine;
            continue;
        }
        // ��ȡͼƬ��ӦAGVλ��
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
            // agv��ʼλ��
            agi._oriAgvPos = Eigen::Vector3d{ currAgvInfo[0], currAgvInfo[1], currAgvInfo[2] };
        }
        else {
            // ͼƬ��Ӧλ��
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
    // ��ȡRGBD��Ϣ
    for (int i = 0; i < vRGBDInfoSL.size(); i++) {
        //char fileName[10];
        //sprintf_s(fileName, "%04d", i);
        // depth����
        string depthImagePath = root + "depth/" + to_string(i + 1) + ".png"; // ".png"
        cv::Mat srcDepth = cv::imread(depthImagePath, cv::IMREAD_ANYDEPTH);
        // rgb����
        string colorImagePath = root + "rgb/" + to_string(i + 1) + ".jpg"; // ".jpg"
        cv::Mat srcColor = cv::imread(colorImagePath);
        // ����
        cameraImgInfo currCamera;
        IntegrateImgInfo(srcColor, srcDepth, vCameraInfo[i]._poseStr, 1, currCamera);
        vRGBDInfoSL[i] = currCamera;
    }

    // ��ȡRGBD CLIP������BBox
    for (int i = 0; i < vRGBDVildInfoSL.size(); i++) {
        char fileName[10];
        sprintf_s(fileName, "%04d", i);
        // Ŀ���������򣨵����������򣩣�VILD������
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
    // ����ɢ�㼯����ͼ����
    cv::Mat dst = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
    // �������е㼯�еĵ�
    for (auto point : vPoint) {
        cv::circle(dst, point, 3, cv::Scalar(255), 2);
    }
    return dst.clone();
}

std::shared_ptr<open3d::geometry::PointCloud> SelfLearning(const string& root, const vector<cameraImgInfo>& vCameraInfo, const vector<Eigen::Vector3d>& vAgvInfo, vector<cameraImgInfo>& vRGBDInfoSL, vector<bboxImgInfo>& vRGBDVildInfoSL) {
    /////////////////////////////
    // ��ѧϰ������ʼѡȡ������ȷ��bbox����ά����������λ�ã���ͶӰ����ǰ�ӽ������Ŀ��� 
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

    // �ع�
    vector<shared_ptr<open3d::geometry::PointCloud>> vLastPointCloud(vRGBDInfoSL.size(), nullptr); // ÿ���ӽǵľֲ�����
    std::shared_ptr<open3d::geometry::PointCloud> pclPtrSL = std::make_shared<open3d::geometry::PointCloud>(); // ��ǰ���оֲ����Ƶ��ܺ�
    for (int i = 0; i < vRGBDInfoSL.size(); i++) {
        reconstructInfo ri;
        //ri._2whichbase = 0x02 | 0x08 | 0x80; // ת����AGV��������ϵ
        //ri._height = -940.; // ������Ż�
        //ri._2whichbase = 0x02; // ת����AGV��������ϵ
        ri._2whichbase = 0x02 | 0x08; // ת����AGV��������ϵ
        ri._mask = vRGBDVildInfoSL[i]._selectmask;
        ri._agvCurrInfo = vAgvInfo[i];
        vLastPointCloud[i] = ReconstructFromOneImg(vRGBDInfoSL[i], ri); // ���ؾֲ�����
        *pclPtrSL += *vLastPointCloud[i]; // ����ƴ��
        //open3d::visualization::DrawGeometries({ vLastPointCloud[i] }, "IteratedPointCloud");
    }

    string pcSavePath = CreateDirUseDate(root + "iteration/");

    open3d::io::WritePointCloudToPCD(pcSavePath + "\\beforeSL.pcd", *pclPtrSL);
    open3d::io::WritePointCloudToPLY(pcSavePath + "\\beforeSL.ply", *pclPtrSL);
    open3d::visualization::DrawGeometries({ pclPtrSL }, "IteratedPointCloud");
    //����
    std::cout << "��ǰ������ɵ��ƴ�ŵ�·��Ϊ��" << pcSavePath << endl;
    int iterateNum = 7;
    if (0) // *************
    for (int itr = 0; itr < iterateNum; itr++) {
        std::cout << "***�ڣ�" << itr << "�ε�����ʼ����" << iterateNum << "�ε�������" << endl;
        for (int i = 0; i < vRGBDInfoSL.size(); i++) {
            // �����ļ��б���ͼƬ
            char dirPath[50];
            int _ret = _mkdir((pcSavePath + "/iteration").c_str());
            sprintf_s(dirPath, "/iteration/%d/", i);
            _ret = _mkdir((pcSavePath + string(dirPath)).c_str());
            // ��ȡ��ǰ���λ�á�����
            cv::Mat nowPose = vRGBDInfoSL[i]._CameraPose.clone(); // ��i��ͼƬ�����λ�ˣ�ֱ�Ӹ�ֵΪǳ����
            nowPose.at<double>(1, 3) -= 100.;
            nowPose = arm2agv(vAgvInfo[i]) * nowPose;
            // �ӵ�ǰ����ӽǽ�����ͶӰ��2Dͼ����
            vector<cv::Mat> imageVec = PerspectivePointCloudFromSpecialPose(pclPtrSL, nowPose);
            // ����ԭ�ȵ���ģmaskImageVec
            // �������ͼȥ���쳣��
            //DealMaskRegion(imageVec);
            vRGBDVildInfoSL[i]._selectmask = imageVec[3].clone(); // �����maskImageVec[vIndex[i]] = imageVec[3]��ǳ����
            vector<vector<cv::Point>> maskContours; // α���ͼ��Ӧ��Ĥ
            //cv::imshow("mask", vRGBDVildInfoSL[i]._selectmask);
            //cv::waitKey(0);
            // Ѱ��ͼƬ��Ĥ����
            maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 0);
            //if (itr == iterateNum - 1) maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 1);
            //else maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 0);
            // ***����rgbͼƬ����������
            vector<vector<cv::Point>> baseContours = FindContours(vRGBDInfoSL[i]._RgbImg, 0);
            cv::Mat binRgbImgSave(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
            for (int cont = 0; cont < baseContours.size(); ++cont) {
                cv::drawContours(binRgbImgSave, baseContours, cont, cv::Scalar(0, 0, 255), 2);
            }
            // ***����maskͼƬ����������
            cv::Mat binMaskImgSave(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
            for (int cont = 0; cont < maskContours.size(); ++cont) {
                cv::drawContours(binMaskImgSave, maskContours, cont, cv::Scalar(0, 0, 255), 2);
            }
            vector<cv::Point> optimumContourSrc, optimumContour;
            // ����ĤͼƬ������ԭʼͼƬ�����������һ��
            double err = FindOptimumContours(baseContours, maskContours, optimumContourSrc, optimumContour);
            vector<vector<cv::Point>> vOptimumContours;
            vOptimumContours.emplace_back(optimumContour);
            /*20231008��������ʹ��*/
            cv::Mat disperseImg = SaveDispersePointImage(optimumContour);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "disperse.jpg", disperseImg);
            //vOptimumContours.emplace_back(optimumContourSrc);
            cv::Mat maskNew(vRGBDInfoSL[i]._RgbImg.rows, vRGBDInfoSL[i]._RgbImg.cols, CV_8UC1, cv::Scalar(0));
            if (vOptimumContours[0].size() != 0) {
                //cv::drawContours(maskNew, vOptimumContours, 0, Scalar(255), -1, 8);
                maskNew = FillImageBasePoint(vOptimumContours[0]);
            }
            cv::Mat maskUse(vRGBDInfoSL[i]._RgbImg.rows, vRGBDInfoSL[i]._RgbImg.cols, CV_8U, cv::Scalar(0));
            vRGBDVildInfoSL[i]._selectmask = maskNew; // 3��ȫ�������۱߽�
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "optimum.jpg", maskNew);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "RGBcontour.jpg", binRgbImgSave);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "Maskcontour.jpg", binMaskImgSave);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + ".jpg", imageVec[3]);
            // �ع����µ��� --- ɾ����һ�ν������������һ�ν��
            if (vLastPointCloud[i] != nullptr) {
                // ����һ�εĵ��ƽ���Ƴ�
                pclPtrSL = RemoveSmallerPointCloud(pclPtrSL, vLastPointCloud[i]); // �������ƵĲ��һ���������ȥ��С�ĵ���
            }
            reconstructInfo ri;
            ri._2whichbase = 0x02 | 0x08; // ת����AGV��������ϵ��������Ĥ��ͼ
            ri._mask = vRGBDVildInfoSL[i]._selectmask;
            ri._agvCurrInfo = vAgvInfo[i];
            vLastPointCloud[i] = ReconstructFromOneImg(vRGBDInfoSL[i], ri); // ���ؾֲ�����
            //vLastPointCloud[i] = cloud_ICP(vLastPointCloud[i], vLastPointCloud[0]);
            *pclPtrSL += *vLastPointCloud[i]; // ����ƴ��
        }
    }
    // �Ե�һ���ӽǣ����ƣ����ж���ICP
    vector<cv::Mat> vAmendM(vRGBDInfoSL.size(), cv::Mat::eye(4, 4, CV_64F));
    pclPtrSL = vLastPointCloud[0];
    for (int i = 1; i < vRGBDInfoSL.size(); i++) {
        vAmendM[i] = PointCloudICP(vLastPointCloud[i], vLastPointCloud[0]);
        //if (3 == i) continue; // 
        *pclPtrSL += *vLastPointCloud[i]; // ����ƴ��
    }
    open3d::visualization::DrawGeometries({ pclPtrSL }, "IteratedPointCloud");
    //// rgbd�ں�
    //for (int i = 0; i < (*pclPtrSL).points_.size(); ++i) {
    //    Eigen::Vector3d pAgv = (*pclPtrSL).points_[i];
    //    Eigen::Vector3d colorCamera = GetColorFromImageVec(pAgv, vCameraInfo, vAmendM, true);
    //    (*pclPtrSL).colors_[i] = colorCamera;
    //}
    //open3d::visualization::DrawGeometries({ pclPtrSL }, "ronghe");

    // ��������
    fstream fs(pcSavePath + "\\readme.txt", ios::out | ios::app);
    fs << endl << "*�������!" << endl;
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
        // �ֲ�����ֱ�Ӻ�ȫ�ֵ���ƥ�侫�Ȳ���->ȫ��ͶӰ���ֲ�������Ĥ���п�ͼ
        // ��α��ǩ��Ĥ����ѡ��
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

        // ICP���ֲ�����ƥ�䵽ȫ�ֵ���->����AGV���
        //cloud_ICP(localPtr, pclPtr);
        vAmendAgv[i] = PointCloudICP(localPtr, pclPtr);
        open3d::visualization::DrawGeometries({ pclPtr, localPtr }, "src");
    }
    return vAmendAgv;
}

/// <summary>
/// �ӵ��ƻ�ȡ�豸ɨ����ϢThermalScanInfo
/// </summary>
/// <param name="ptr">ֻ����Ŀ��ĵ���</param>
/// <param name="vScanInfo"></param>
/// <param name="display"></param>
void ObtainPartitionFromPointCloud(const shared_ptr<open3d::geometry::PointCloud>& ptr, vector<ThermalScanInfo>& vScanInfo, bool display) {
    //// ������Ʒֿ�
    PointCloudInfo currPcli;
    InitPointCloudInfo(ptr, currPcli);
    vector<std::shared_ptr<open3d::geometry::PointCloud>> vCropedCloudForPCA = PointCloudPartition(ptr); // ������вü����ĵ���
    vector<std::shared_ptr<open3d::geometry::LineSet>> vNormOrient = PointCloudPartitionPCA(currPcli, vCropedCloudForPCA, vScanInfo);
    //// ���ƺϲ�����ʾ��ֱ��+=Ҳ����
    //int pointNum = 0;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> vTotalCloud;
    ////shared_ptr<open3d::geometry::PointCloud> downsampledTotal = totalPointCloud->VoxelDownSample(5);//�Ե��ƽ����²���
    shared_ptr<open3d::geometry::PointCloud> downsampledTotal = ptr->VoxelDownSample(5);//�Ե��ƽ����²���
    for (int i = 0; i < vNormOrient.size(); i++) {
        vTotalCloud.emplace_back(vNormOrient[i]); // ��������һ����ʾ
        // ��ȡ�豸3D����
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
    ////AGV��ǰλ��ʹ�û�е��ɨ���Ӧ��һ���ֿ�
    //����λ��
    cv::Mat goalarm = arm2agvM.inv() * gripperPoseMat;
    goalarm.at<double>(1, 3) = goalarm.at<double>(1, 3) + 100;
    cout << "goalarm info��" << goalarm << endl;
    string cmdStr;
    ConvertMat2String(goalarm, cmdStr);
    std::cout << "������ĩ��λ�ˣ�" << cmdStr << endl;
    robotArmHandle->Move2OnePoint(cmdStr, 1);
    std::cout << "***[3]�뼤�������ǽ��вɼ���" << endl;
    string str;
    cin >> str;

    // ��ȡ��ǰagvλ�˼�IMU
    AMRLocalInfo currAmrliAdjusted;
    RequestAMRLocal(currAmrliAdjusted);
    currAmrliAdjusted._x *= 1000.;
    currAmrliAdjusted._y *= 1000.;
    Eigen::Vector3d currImu = robot_status_imu_req(m_SockClient05);
    // ��ȡ��ǰ��е������Thermal
    string currStrPose = robotArmHandle->ReadRobotArmPosString();

    //����ĩ��λ�ˣ�imu����
    fstream fs(root + "thermal.txt", std::ios::out | std::ios::app);
    fs << currStrPose << endl;
    fs << currAmrliAdjusted._x << "," << currAmrliAdjusted._y << "," << currAmrliAdjusted._angle << "," << currImu[0] << "," << currImu[1] << "," << currImu[2] << endl;
    fs.close();
}

void CalcThermalPoseAndScan(const shared_ptr<open3d::geometry::PointCloud>& ptr, const vector<ThermalScanInfo>& vScanInfo, const string& root) {
    open3d::geometry::OrientedBoundingBox obb = ptr->GetOrientedBoundingBox();
    Eigen::Vector3d obbCenter = obb.GetCenter();
    fstream fs(root + "thermal.txt", std::ios::out | std::ios::trunc);// ���֮ǰ����
    fs.close();
    string robotArmPoseMid1 = "A+015000-127299-000008+15020-08999-06020BC"; // ��е���м临λ��->���
    string robotArmPoseMid2 = "A+015000-106278+017920+17999-00584-08999BC"; // ��е���м临λ��->��ֱ����
    string robotArmPoseOri = "A+073125-018311+025299-07470+08569-07860BC"; // AGV�˶���е�۸�λ��
    //for (int goalPos = 3; goalPos < vScanInfo.size(); goalPos++) {

    for (int goalPos = 1; goalPos < vScanInfo.size(); goalPos++) {
        //Ѱ��ÿһ���ֿ�����AGVλ��vScanInfo
        Eigen::Vector3d centerTgoal = (vScanInfo[goalPos]._gripperPosition - obbCenter);
        centerTgoal[2] = 0;//���Ը߶�
        centerTgoal = centerTgoal.normalized();
        Eigen::Vector3d currAgvGoal;
        currAgvGoal = vScanInfo[goalPos]._gripperPosition + centerTgoal * 1200;//AGVԭ��Ŀ���1.2m
        cout << "��ǰAGVĿ��㣺" << currAgvGoal.transpose() << endl;
        cout << "������Ϣ��е��λ��Ϊ��" << vScanInfo[goalPos]._cmdStr << endl;
        cout << "������Ϣagvλ��Ϊ��" << obbCenter.transpose() << endl;
        //continue;
        //Move2Goal(Eigen::Vector2d{ obbCenter[0], obbCenter[1] }, Eigen::Vector2d{ currAgvGoal[0], currAgvGoal[1] });
        MoveAGV2OneGoal(currAgvGoal, true, obbCenter);
        AMRRotation(PI / 2., 1., m_SockClient06); // ��ת90���ұ۳�������
        AMRLibration(0.2, -0.2, 0.0, m_SockClient06, 1); // x����0.2m
        //continue;
        std::cout << "***[1]AGV�Ծ�λ�����������е��λ�ˣ�" << endl;
        string str;
        cin >> str;

        //�ƶ���е�۵���ָ����λ�����㵽��PCAָ��λ��
        robotArmHandle->Move2OnePoint(robotArmPoseMid1, 1);
        robotArmHandle->Move2OnePoint(robotArmPoseMid2, 1);
        // *******����rgbdĩ��λ�˲ɼ��ֲ����ƣ�����ƥ�䵽ȫ�ֵ�������AGV�˶����
        //RobotArm2Goal(agvGoalEig); //******
        // ��ȡ��ǰagvλ��
        cameraImgInfo cii;
        SaveRGBDInfo(root, goalPos + 1, cii);
        std::cout << "***[2]�Ѳɼ�RGBD�����������������λ�ˣ�" << endl;
        cin >> str;
        // AGV����Ŀ��㣬������е��ʹthermalָ��Ŀ��
        //if (goalPos != 3 && goalPos != 4 && goalPos != 5) RobotAdjustThermal2Goal(vScanInfo[goalPos]._gripperPoseMat, goalPos, root);
        //�ص�ָ����λ
        robotArmHandle->Move2OnePoint(robotArmPoseMid2, 1);
        robotArmHandle->Move2OnePoint(robotArmPoseMid1, 1); // �����ǲɼ���λ��
        cout << "-----------------------\n\n\n";
        robotArmHandle->Move2OnePoint(robotArmPoseOri, 1);//AGV�˶���е�۸�λ��
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
    //// ��16λ���ͼ��ת��Ϊ8λ�Ҷ�ͼ��
    //cv::Mat grayImage;
    //depthImage.convertTo(grayImage, CV_8U, 1.0 / 256.);
    //// Ӧ��α��ɫӳ��
    //cv::applyColorMap(grayImage, pseudoColorImage, cv::COLORMAP_JET);

    cv::Mat depth;
    depthImage.convertTo(depth, CV_64F);
    // ��ȡ16λ���ͼ�����ط�Χ0��65535����������ת��Ϊ8λ�����ط�Χ0��255��
    double minEle1 = *min_element(depth.begin<double>(), depth.end<double>());
    depth -= minEle1;
    double minEle = *min_element(depth.begin<double>(), depth.end<double>());
    double maxEle = *max_element(depth.begin<double>(), depth.end<double>());
    depth = depth / (double)(maxEle - minEle);
    depth *= 255.;
    // ʹ��Խ���ĵط����ֵԽ��ԽԶ�ĵط����ֵԽС���Դﵽα��ɫͼ����Զ���Ŀ�ġ�
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
            // ����ʵʱRGBDͼƬ����ʾ������ͼ
            if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
                cv::flip(depthImage, depthImage, 1);
                cv::Mat pseudoImg = Convert2PseudoColor(depthImage);
                cv::imshow("depth image", pseudoImg); // ������ʾ
            }
        }        
        // ����Esc���˳�ѭ��
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    ti->stop();
    ti->~Timer();
}

/// <summary>
/// �������->����ɼ���λ->����AGV�˶����ɼ���λ->������е��ָ��Ŀ��->�ɼ�->����
/// </summary>
/// <param name="goal">��е��Ŀ���</param>
void Stage1Process(const Eigen::Vector3d& armGoal) {
    // ��ȡ��ǰagvλ�ˣ�ת����е����Ŀ�굽AGV�����
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    //agi._oriAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y,currAmrli._angle };
    cv::Mat agvGoal = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << armGoal[0], armGoal[1] - 100., armGoal[2], 1);
    Eigen::Vector3d agvGoalEig{ agvGoal.at<double>(0, 0),agvGoal.at<double>(1, 0),agvGoal.at<double>(2, 0) };
    // ����ɼ���λ���滮RGBD����������ʾ
    vector<Eigen::Vector3d> vGoal = CalcAgvPosForRGBD(agvGoalEig, 300, 1200, 60, false);
    // ��е��λ�˸�λ
    RobotArmReset();
    //thread reconThread(UpdateRealTimePointCloud); // ���߳�ʵʱ�жϵ�ǰRGBD�Ƿ��·����ͻ
    //reconThread.detach();
    // ����AGV�˶����ɼ���λ
    string root = "./results/stage_1/"; // ����·��
    // *********���֮ǰ���ݣ������ʼλ�ˣ�����ʱ��λ�ˣ�
    fstream fs(root + "rgbd.txt", ios::out | ios::trunc);
    fs << 0 << endl;
    fs << currAmrli._x << "," << currAmrli._y << "," << currAmrli._angle << endl;
    fs.close();
    std::shared_ptr<open3d::geometry::PointCloud> pcl = std::make_shared<open3d::geometry::PointCloud>(); // ȫ�ֵ���
    for (int goalIdx = 0; goalIdx < vGoal.size(); goalIdx++) {
        cout << "arm goal:" << armGoal.transpose() << endl;
        cout << "ture goal:" << agvGoalEig.transpose() << endl;
        cout << "set goal:" << vGoal[goalIdx].transpose() << endl;
        //continue;
        //continue;
        // �ƶ�����i��Ŀ��
        MoveAGV2OneGoal(vGoal[goalIdx], true, agvGoalEig);
        //Move2Goal(Eigen::Vector2d{ agvGoalEig[0], agvGoalEig[1] }, Eigen::Vector2d{ vGoal[goalIdx][0], vGoal[goalIdx][1] });
        // ���λ��ָ��Ŀ��
        //RobotArm2Goal(agvGoalEig);
        RobotArm2GoalUpdate(agvGoalEig);
        //DisplayRealTimeRGBD();
        //string str;
        //cin >> str;
        // ����RGBD����
        cameraImgInfo cii;
        SaveRGBDInfo(root, goalIdx + 1, cii);
        // hd���λ��ָ��Ŀ��
        //cout << "below infomation is adjust hd camera!" << endl;
        RobotArm2GoalUpdate(agvGoalEig, 1);
        string str;
        cin >> str;
        SaveHDInfo(root, goalIdx + 1, cii);
        // ƴ�ӵ���        
        reconstructInfo ri;
        ri._2whichbase |= 0x02; // ת����AGV��������ϵ
        ri._agvCurrInfo = cii._agvPose;
        *pcl += *ReconstructFromOneImg(cii, ri);
    }

    open3d::io::WritePointCloudToPCD(root + "pointcloud.pcd", *pcl);
    open3d::io::WritePointCloudToPLY(root + "pointcloud.ply", *pcl);
    open3d::visualization::DrawGeometries({ pcl }, "pcl");
}

void Stage2Process() {
    // ��ȡtxt����
    string dataPath = "./results/stage_1/";
    vector<cameraImgInfo> vCameraInfo;
    vector<Eigen::Vector3d> vAgvInfo;
    ReadStage1TxtInfo(dataPath, vCameraInfo, vAgvInfo);
    // ��ȡvild������Ϣ
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
    // ȥ����ײ����
    //pclPtrSL = RemoveButtonPoints(pclPtrSL);
    open3d::io::WritePointCloudToPLY(savePath + "1.ply", *pclPtrSL);
    // ���Ʒֿ컮��ĩ��ɨ����Ϣ
    vector<ThermalScanInfo> vScanInfo;
    ObtainPartitionFromPointCloud(pclPtrSL, vScanInfo);
    //return;
    // ����ÿһ����Ѱ�����AGVλ�ˣ�Ȼ�󷴽��е��ĩ��λ��
    CalcThermalPoseAndScan(pclPtrSL, vScanInfo, savePath);
}

void Stage3Process() {
    string inputPath = "./results/stage_2/";
    string outputPath = "./results/stage_3/";
    // ��ȡȫ�ֵ���stage2 SL���
    open3d::geometry::PointCloud pcl;
    open3d::io::ReadPointCloudFromPLY(inputPath + "shipIteratedICP.ply", pcl);
    std::shared_ptr<open3d::geometry::PointCloud> pclPtr = make_shared<open3d::geometry::PointCloud>(pcl);
    //open3d::visualization::DrawGeometries({ pclPtr }, "src");
    // ��ȡ�ֲ�����rgbd
    vector<cameraImgInfo> vRgbdInfo;
    vector<Eigen::Vector3d> vAgvInfoRgbd;
    ReadStage1TxtInfo(inputPath, vRgbdInfo, vAgvInfoRgbd);
    // ��ȡThermal
    vector<cameraImgInfo> vThermalInfo;
    vector<Eigen::Vector3d> vAgvInfoThermal;
    ReadStage1TxtInfo(inputPath, vThermalInfo, vAgvInfoThermal, 1);
    // ��ȡHD����// ��ȡtxt����
    // ��ȡvild������Ϣ
    int useNum = vRgbdInfo.size();
    vector<cameraImgInfo> vRGBDInfoSL(useNum);
    vector<bboxImgInfo> vRGBDVildInfoSL(useNum);
    ReadStage1SLVildInfo(inputPath, vRgbdInfo, vAgvInfoRgbd, vRGBDInfoSL, vRGBDVildInfoSL);
    // �ӹ��ػ���ȡ��������ͼƬ�Ǿ����
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
    //// ����ÿ��thermal��Agvλ�����������תƽ�ƾ���
    vector<cv::Mat> vAmendM = CalcAmendMatrixEachAgv(pclPtr, vRgbdInfo, vAgvInfoRgbd, vRGBDVildInfoSL);
    //vector<cv::Mat> vAmendM(useNum);
    //vThermalInfo.resize(1);
    // ������AGVλ�˺�����ں���ɫ
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