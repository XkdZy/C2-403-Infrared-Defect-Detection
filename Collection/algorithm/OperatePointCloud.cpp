#include "OperatePointCloud.h"
#include "RobotAlgorithm.h"
#include "GlobalVariable.h"
#include "CoordinateTransformation.h"
#include "AstraCameraD2C.h"

struct Image {
    Image(int _imageIndex, int _u, int _v, int _distance) {
        imageIndex = _imageIndex;
        distance = _distance;
        u = _u;
        v = _v;
    }
    int imageIndex;
    int distance;
    int u;
    int v;
};

void GetColorFromImageVec(Eigen::Vector3d& colorTmp, const cv::Mat& p_base, string thermalOrCamera) {
    //vector<cameraImgInfo> vThermalInfo;//***********************************
    //colorTmp << 0.0, 0.0, 0.0;
    //Eigen::Vector3d pointTemp(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
    //vector<Image> imageVec;
    //if (thermalOrCamera == "thermal") {
    //    for (int j = 0; j < vThermalInfo.size(); j++) {
    //        int distanceTmp = (vThermalInfo[j]._CameraPosition - pointTemp).norm();
    //        //�����������µ�������Ϣ
    //        cv::Mat p_thermal = H_Thermal2Gripper.inv() * vThermalInfo[j]._Gripper2Base * p_base;
    //        //���������Ƕ�Ӧ����������
    //        p_thermal = ThermalMatrix * p_thermal;

    //        int u = p_thermal.at<double>(0, 0) / p_thermal.at<double>(2, 0);
    //        int v = p_thermal.at<double>(1, 0) / p_thermal.at<double>(2, 0);
    //        if (u > 0 && u < 640 && v > 0 && v < 480) {
    //            imageVec.push_back(Image(j, u, v, distanceTmp));
    //        }
    //    }
    //}
    //else if (thermalOrCamera == "camera") {
    //    for (int j = 0; j < vRGBDInfo.size(); j++) {
    //        int distanceTmp = (vRGBDInfo[j]._CameraPosition - pointTemp).norm();
    //        //�����������µ�������Ϣ
    //        cv::Mat p_camera = H_Camera2Gripper.inv() * vRGBDInfo[j]._Gripper2Base * p_base;
    //        //���������Ƕ�Ӧ����������
    //        p_camera = CameraMatrix * p_camera;

    //        int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
    //        int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
    //        if (u > 0 && u < 640 && v > 0 && v < 480) {
    //            imageVec.push_back(Image(j, u, v, distanceTmp));
    //        }
    //    }
    //}

    ////�鿴�õ����Ƿ���ƥ�������������ͼ
    //if (imageVec.empty()) {
    //    //���û��ƥ�������ͼ��������Ϊ��ɫ
    //    colorTmp << 1, 1, 1;
    //}
    //else {
    //    //�����ƥ�������ͼ��������ͼ��Ȩ�ں�
    //    sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
    //        return img1.distance < img2.distance; });

    //    int differencValue = 30;
    //    while (imageVec.back().distance - imageVec.front().distance > differencValue) {
    //        imageVec.pop_back();
    //    }
    //    int distanceSum = differencValue;
    //    for (auto image_ : imageVec) {
    //        distanceSum += image_.distance;
    //    }
    //    int meanDistance = distanceSum / imageVec.size();

    //    for (auto image_ : imageVec) {
    //        double weight = (meanDistance - image_.distance) / (double)differencValue;
    //        Mat img;
    //        if (thermalOrCamera == "thermal") {
    //            img = vThermalInfo[image_.imageIndex]._RgbImg;
    //        }
    //        else if (thermalOrCamera == "camera") {
    //            img = vRGBDInfo[image_.imageIndex]._RgbImg;
    //        }
    //        int u = image_.u;
    //        int v = image_.v;
    //        Eigen::Vector3d colorTemp_(img.at<Vec3b>(v, u)[2] / 255.0, img.at<Vec3b>(v, u)[1] / 255.0, img.at<Vec3b>(v, u)[0] / 255.0);

    //        colorTmp += colorTemp_ * weight;
    //    }
    //}
}

Eigen::Vector3d GetColorFromImageVec(const Eigen::Vector3d& pAgv, const vector<cameraImgInfo>& vImgInfos, vector<cv::Mat>& vAmend, bool amend) {
    vector<Image> imageVec;
    cv::Mat p_agv = (cv::Mat_<double>(4, 1) <<pAgv [0], pAgv[1], pAgv[2], 1);

    for (int j = 0; j < vImgInfos.size(); j++) {
        if (!amend) vAmend[j] = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat agvAmend = vAmend[j].inv() * p_agv; // ����AGVλ�����
        cv::Mat Harm2agv = arm2agv(vImgInfos[j]._agvPose);
        cv::Mat p_base = Harm2agv.inv() * agvAmend;
        p_base.at<double>(1, 0) += 100; // y����ƫ��100mm
        if (2 == vImgInfos[j]._type) {
            cv::Mat rot = RotateRxMat(vImgInfos[j]._agvIMU[1]); // �ɼ�Thermal����IMU���
            p_base = rot * p_base;
        } 
        cv::Mat p_camera;
        int distanceTmp;
        // �����������ϵ�µ�������Ϣ
        if (1 == vImgInfos[j]._type) {
            p_camera = H_Camera2Gripper.inv() * vImgInfos[j]._Gripper2Base.inv() * p_base;
            distanceTmp = (Eigen::Vector3d(p_camera.at<double>(0, 0), p_camera.at<double>(1, 0), p_camera.at<double>(2, 0))).norm();
            p_camera = CameraMatrix * p_camera;
        }
        else if (2 == vImgInfos[j]._type) {
            p_camera = H_Thermal2Gripper.inv() * vImgInfos[j]._Gripper2Base.inv() * p_base;
            distanceTmp = (Eigen::Vector3d(p_camera.at<double>(0, 0), p_camera.at<double>(1, 0), p_camera.at<double>(2, 0))).norm();
            p_camera = ThermalMatrix * p_camera;
        }
        else {
            p_camera = H_Camera2Gripper_HD.inv() * vImgInfos[j]._Gripper2Base.inv() * p_base;
            distanceTmp = (Eigen::Vector3d(p_camera.at<double>(0, 0), p_camera.at<double>(1, 0), p_camera.at<double>(2, 0))).norm();
            p_camera = CameraMatrix_HD * p_camera;
        }

        int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
        int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
        //cout << "u:" << u << "  v:" << v << "   distanceTemp:" << distanceTmp << endl;
        if (u > 0 && u < 640 && v > 0 && v < 480) {
            //cout << "u:" << u << "  v:" << v << "   distanceTemp:" << distanceTmp << endl;
            imageVec.push_back(Image(j, u, v, distanceTmp));
        }
    }

    Eigen::Vector3d colorTmp{ 0.0, 0.0, 0.0 };
    //�鿴�õ����Ƿ���ƥ�������������ͼ
    if (imageVec.empty()) {
        //���û��ƥ�������ͼ��������Ϊ��ɫ
        colorTmp << 1, 1, 1;
    }
    else {
        //�����ƥ�������ͼ��������ͼ��Ȩ�ں�
        sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
            return img1.distance < img2.distance; });

        int differencValue = 30;
        while (imageVec.back().distance - imageVec.front().distance > differencValue) {
            imageVec.pop_back();
        }
        int distanceSum = differencValue;
        for (auto image_ : imageVec) {
            distanceSum += image_.distance;
        }
        int meanDistance = distanceSum / imageVec.size();

        for (auto image_ : imageVec) {
            double weight = (meanDistance - image_.distance) / (double)differencValue;
            cv::Mat img;
            img = vImgInfos[image_.imageIndex]._RgbImg;
            int u = image_.u;
            int v = image_.v;
            Eigen::Vector3d colorTemp_(img.at<cv::Vec3b>(v, u)[2] / 255.0, img.at<cv::Vec3b>(v, u)[1] / 255.0, img.at<cv::Vec3b>(v, u)[0] / 255.0);

            colorTmp += colorTemp_ * weight;
        }
    }

    return colorTmp;
}

Eigen::Vector3d GetColorFromImageVec(const cv::Mat& p_base, const Eigen::Vector3d& refPos, const vector<cameraImgInfo>& vImg) {
    Eigen::Vector3d colorTmp{ 0.0, 0.0, 0.0 };
    Eigen::Vector3d baseEig(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
    if (vImg.empty()) {
        cout << "ͼƬ����Ϊ�գ�" << endl;
        return colorTmp;
    }
    vector<Image> imageVec;
    //int j = 1;
    for (int j = 0; j < vImg.size(); j++) {
        // ������ϵͳһ��idx��Ӧ��е��λ��
        //cout << "��ǰͼƬAGVλ�ˣ�" << refPos.transpose() << " ThermalͼƬAGVλ�ˣ�" << vImg[0]._agvPose.transpose() << endl;
        Eigen::Vector3d baseEigTmp = AGVMove2ArmPos(refPos - vImg[j]._agvPose, baseEig, true, vImg[j]._agvPose[2]);
        cv::Mat p_baseDup = (cv::Mat_<double>(4, 1) << baseEigTmp[0], baseEigTmp[1], baseEigTmp[2], 1);

        int distanceTmp = (vImg[j]._CameraPosition - baseEigTmp).norm();
        //�����������µ�������Ϣ
        cv::Mat p_camera;
        if (vImg[j]._type == 2) {
            // thermal
            cv::Mat p_thermal = H_Thermal2Gripper.inv() * vImg[j]._Gripper2Base.inv() * p_baseDup;
            p_camera = ThermalMatrix * p_thermal;
        }
        else if (vImg[j]._type == 1) {
            // rgbd
            cv::Mat p_rgbd = H_Camera2Gripper.inv() * vImg[j]._Gripper2Base.inv() * p_baseDup;
            p_camera = CameraMatrix * p_rgbd;
        }
        //cout << p_camera.t() << endl;
        //���������Ƕ�Ӧ����������
        int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
        int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
        if (u >= 0 && u < 640 && v >= 0 && v < 480) {
            //cout << p_camera.t() << "   " << u << "  " << v << endl;
            imageVec.push_back(Image(j, u, v, distanceTmp));
        }
    }

    //�鿴�õ����Ƿ���ƥ�������������ͼ
    if (imageVec.empty()) {
        //���û��ƥ�������ͼ��������Ϊ��ɫ
        colorTmp << 1, 1, 1;
        //cout << "�ܵ�ͼƬ�ߴ�Ϊ��" << vImg.size() << endl;
    }
    else {
        //cout << "�ܵ�ͼƬ�ߴ�Ϊ��" << vImg.size() << "  " << imageVec.size() << endl;
        //�����ƥ�������ͼ��������ͼ��Ȩ�ں�
        sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
            return img1.distance < img2.distance;
            });

        //// ���������
        //int u = imageVec[0].u;
        //int v = imageVec[0].v;
        //if (vImg[0]._type == 2) {
        //    u = 639 - u;
        //    v = 479 - v;
        //}
        //return Eigen::Vector3d{
        //    vImg[0]._RgbImg.at<Vec3b>(v, u)[2] / 255.0,
        //    vImg[0]._RgbImg.at<Vec3b>(v, u)[1] / 255.0,
        //    vImg[0]._RgbImg.at<Vec3b>(v, u)[0] / 255.0
        //};

        // ��Ȩ
        //int differencValue = 30;
        int differencValue = 100;
        while (imageVec.back().distance - imageVec.front().distance > differencValue) {
            imageVec.pop_back();
        }
        //cout << "�ܵ�ͼƬ�ߴ�Ϊ��" << vImg.size() << "  " << imageVec.size() << endl;
        int distanceSum = differencValue;
        for (auto image_ : imageVec) {
            distanceSum += image_.distance;
        }
        int meanDistance = distanceSum / imageVec.size();

        for (auto image_ : imageVec) {
            double weight = (meanDistance - image_.distance) / (double)differencValue;
            cv::Mat img = vImg[image_.imageIndex]._RgbImg;
            int u = image_.u;
            int v = image_.v;
            if (vImg[0]._type == 2) {
                u = 639 - u; // ˮƽ����
                v = 479 - v; // ��ֱ����
            }
            Eigen::Vector3d colorTemp_(img.at<cv::Vec3b>(v, u)[2] / 255.0, img.at<cv::Vec3b>(v, u)[1] / 255.0, img.at<cv::Vec3b>(v, u)[0] / 255.0);
            //Eigen::Vector3d colorTemp_(img.at<Vec3b>(v, u)[0] / 255.0, img.at<Vec3b>(v, u)[1] / 255.0, img.at<Vec3b>(v, u)[2] / 255.0);

            colorTmp += colorTemp_ * weight;
        }
    }

    //cout << "color:" << colorTmp.transpose() << endl;
    return colorTmp;
}

//void GetColorFromImageVec(Eigen::Vector3d& colorTmp, const cv::Mat& p_base, string thermalOrCamera) {
//    colorTmp << 0.0, 0.0, 0.0;
//    Eigen::Vector3d pointTemp(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
//    vector<Image> imageVec;
//    if (thermalOrCamera == "thermal") {
//        for (int j = 0; j < thermalPositionVec.size(); j++) {
//            int distanceTmp = (thermalPositionVec[j] - pointTemp).norm();
//            //�����������µ�������Ϣ
//            cv::Mat p_thermal = H_Thermal2Gripper.inv() * H_Base2Gripper_thermalVec[j] * p_base;
//            //���������Ƕ�Ӧ����������
//            p_thermal = ThermalMatrix * p_thermal;
//
//            int u = p_thermal.at<double>(0, 0) / p_thermal.at<double>(2, 0);
//            int v = p_thermal.at<double>(1, 0) / p_thermal.at<double>(2, 0);
//            if (u > 0 && u < 640 && v > 0 && v < 480) {
//                imageVec.push_back(Image(j, u, v, distanceTmp));
//            }
//        }
//    }
//    else if (thermalOrCamera == "camera") {
//        for (int j = 0; j < cameraPositionVec.size(); j++) {
//            int distanceTmp = (cameraPositionVec[j] - pointTemp).norm();
//            //�����������µ�������Ϣ
//            cv::Mat p_camera = H_Camera2Gripper.inv() * H_Base2Gripper_cameraVec[j] * p_base;
//            //���������Ƕ�Ӧ����������
//            p_camera = CameraMatrix * p_camera;
//
//            int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
//            int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
//            if (u > 0 && u < 640 && v > 0 && v < 480) {
//                imageVec.push_back(Image(j, u, v, distanceTmp));
//            }
//        }
//    }
//
//    //�鿴�õ����Ƿ���ƥ�������������ͼ
//    if (imageVec.empty()) {
//        //���û��ƥ�������ͼ��������Ϊ��ɫ
//        colorTmp << 1, 1, 1;
//    }
//    else {
//        //�����ƥ�������ͼ��������ͼ��Ȩ�ں�
//        sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
//            return img1.distance < img2.distance; });
//
//        int differencValue = 30;
//        while (imageVec.back().distance - imageVec.front().distance > differencValue) {
//            imageVec.pop_back();
//        }
//        int distanceSum = differencValue;
//        for (auto image_ : imageVec) {
//            distanceSum += image_.distance;
//        }
//        int meanDistance = distanceSum / imageVec.size();
//
//        for (auto image_ : imageVec) {
//            double weight = (meanDistance - image_.distance) / (double)differencValue;
//            Mat img;
//            if (thermalOrCamera == "thermal") {
//                img = thermalImageVec[image_.imageIndex];
//            }
//            else if (thermalOrCamera == "camera") {
//                img = colorImageVec[image_.imageIndex];
//            }
//            int u = image_.u;
//            int v = image_.v;
//            Eigen::Vector3d colorTemp_(img.at<Vec3b>(v, u)[2] / 255.0, img.at<Vec3b>(v, u)[1] / 255.0, img.at<Vec3b>(v, u)[0] / 255.0);
//
//            colorTmp += colorTemp_ * weight;
//        }
//    }
//}

Eigen::Vector3d ConvertPixel2World(int row, int col, const cameraImgInfo& cii, ushort depth) {
    ushort depthNum = 0;
    if (depth == 0) depthNum = cii._DepthImg.at<ushort>(row, col);
    else depthNum = depth;
    cout << "depth num:" << depthNum << endl;
    Eigen::Vector3d position{ 0., 0., 0. };
    if (depthNum > MIN_DISTANCE && depthNum < MAX_DISTANCE) {
        //cout << depthNum << endl;
        //�����������µ�����
        double x_camera, y_camera, z_camera;
        Eigen::Vector3d camera_xyz = ConvertPixel2Camera(col, row, depthNum, 0);
        x_camera = camera_xyz[0];
        y_camera = camera_xyz[1];
        z_camera = camera_xyz[2];
        //astraCameraD2C->convertDepthToWorld(col, row, depthNum, x_camera, y_camera, z_camera);
        cv::Mat p_camera{ x_camera, y_camera, z_camera, 1.0 };
        //cout << "p_camera:" << p_camera.t() << endl;
        //�����е���µ�������Ϣ
        cv::Mat p_gripper = H_Camera2Gripper * p_camera;

        //��������µ�������Ϣ
        cv::Mat p_base = cii._Gripper2Base * p_gripper;
        //cout << "p_base:" << p_base.t() << endl;
        //cout << "_Gripper2Base:" << cii._Gripper2Base << endl;

        position = Eigen::Vector3d(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
    }

    return position;
}

/**
    �����ͼ�������
    @para depthImg:����ͼƬ
*/
void FillDepthGaps(cv::Mat& depthImage) {
    //cv::imshow("depth", depthImage);
    //cv::waitKey(0);
    cv::Mat mask;
    cv::threshold(depthImage, mask, 0, 65536, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(mask, mask, kernel);

    cv::Mat filledDepthImage;
    bitwise_and(depthImage, mask, depthImage);
    //depthImage.copyTo(filledDepthImage, mask);
    //bitwise_and(filledDepthImage, mask, depthImage);
}

std::shared_ptr<open3d::geometry::PointCloud> ReconstructFromOneImg(const cameraImgInfo& cii, reconstructInfo& ri) {
    std::shared_ptr<open3d::geometry::PointCloud> pcl = std::make_shared<open3d::geometry::PointCloud>();
    cv::Mat depthImage = cii._DepthImg.clone();
    //cv::imshow("depth", depthImage);
    //cv::waitKey(0);
    FillDepthGaps(depthImage);
    // ƴ�����ͼΪ��ά�������ݣ�ƥ���������ͼ����ά����
    for (int row = 0; row < depthImage.rows; row++) {
        for (int col = 0; col < depthImage.cols; col++) {
            ushort depthNum = depthImage.at<ushort>(row, col); // ��������ǰ���ص����ֵ
            if (depthNum < MIN_DISTANCE) continue;
            if ((0x08 & ri._2whichbase) && ri._mask.at<uchar>(row, col) == 0) continue;

            //�����������µ�����
            double x_camera, y_camera, z_camera;
            Eigen::Vector3d camera_xyz = ConvertPixel2Camera(col, row, depthNum, 0);
            x_camera = camera_xyz[0];
            y_camera = camera_xyz[1];
            z_camera = camera_xyz[2];
            cv::Mat p_camera{ x_camera, y_camera, z_camera, 1.0 };

            //�����е���µ�������Ϣ
            cv::Mat p_gripper = H_Camera2Gripper * p_camera;

            //��������µ�������Ϣ
            cv::Mat p_base = cii._Gripper2Base * p_gripper;

            Eigen::Vector3d pointTemp{ p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2) };
            Eigen::Vector3d colorThermal(0.0, 0.0, 0.0);
            Eigen::Vector3d colorCamera(0.0, 0.0, 0.0);
            Eigen::Vector3d colorTemp(0.0, 0.0, 0.0);

            if ((0x01 & ri._2whichbase) && ri._agvRT) {
                // AGV��תƽ�ƣ���е�����������Ӧ�仯
                pointTemp = AGVMove2ArmPos(ri._agvRTInfo, pointTemp, true, ri._agvOriDir);
            }
            else if (0x02 & ri._2whichbase) {
                // ��е���������굽AGV��������
                pointTemp = arm2agv(ri._agvCurrInfo, pointTemp);
            }
            if (0x04 & ri._2whichbase) {
                // �޷�
                if ((pointTemp - ri._setPoint).norm() > ri._dmm) continue;
            }
            if (0x10 & ri._2whichbase) {
                // thermalͼƬ��ɫ
                colorThermal = GetColorFromImageVec(p_base, ri._rgbds[ri._rgbdIdx]._agvPose, ri._thermals);
            }
            if (0x40 & ri._2whichbase) {
                // RGBDͼƬ��ɫ
                colorCamera = GetColorFromImageVec(p_base, ri._rgbds[ri._rgbdIdx]._agvPose, ri._rgbds);
            }
            if (0x20 & ri._2whichbase) {
                // �жϸõ���ȫ�ֵ������Ƿ���ڣ������������Ƿ��иõ㣩����������򲻱���
                vector<Eigen::Vector3d> v{ pointTemp };
                if (ri._totalVoxel->CheckIfIncluded(v)[0]) continue;
            }
            if (0x80 & ri._2whichbase) {
                // z�������޷�
                if (ri._height > pointTemp[2]) continue;
            }

            // �õ�ǰͼƬ��ɫ��ɫ
            //double weightCamera = 0.2, weightThermal = 0.8;
            //double weightCamera = 0.5, weightThermal = 0.5;
            double weightCamera = 1., weightThermal = 0.;
            if ((0x40 & ri._2whichbase) == 0) {
                cv::Vec3b nowImg = cii._RgbImg.at<cv::Vec3b>(row, col);
                colorCamera = Eigen::Vector3d(nowImg[2] / 255.0, nowImg[1] / 255.0, nowImg[0] / 255.0);
            }
            colorTemp = weightCamera * colorCamera + weightThermal * colorThermal;
            //cout << colorThermal.transpose() << "   " << colorCamera.transpose() << endl;

            pcl->points_.push_back(pointTemp);
            pcl->colors_.push_back(colorTemp);
        }
    }

    return pcl;
}

std::shared_ptr<open3d::geometry::PointCloud> RemoveSmallerPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& bigPC, const std::shared_ptr<open3d::geometry::PointCloud>& smallPC) {
    std::shared_ptr<open3d::geometry::PointCloud> ret = bigPC;

    vector<double> dis = bigPC->ComputePointCloudDistance(open3d::geometry::PointCloud(smallPC->points_));
    vector<size_t> vIndex;
    for (int i = 0; i < dis.size(); i++) {
        if (dis[i] >= 0.1) {
            vIndex.emplace_back(i);
        }
    }
    if (vIndex.size() != 0) {
        ret = bigPC->SelectByIndex(vIndex);
        //open3d::visualization::DrawGeometries({ pclWithoutlastPointCloud }, "diff");
    }

    return ret;
}

void CombinePartialPointCloud(std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC1,
                            std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC2,
                            std::vector<std::shared_ptr<const open3d::geometry::Geometry>>& totalPC)
{
    for (int i = 0; i < vPartialPC1.size(); i++)
    {
        totalPC.emplace_back(vPartialPC1[i]); // ��������һ����ʾ
        totalPC.emplace_back(vPartialPC2[i]);
    }
}

std::shared_ptr<open3d::geometry::PointCloud> RotatePointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& srcPcl, const PointCloudRotateInfo pcri) {
    // ����ԭʼ����
    std::shared_ptr<open3d::geometry::PointCloud> pclUse = std::make_shared<open3d::geometry::PointCloud>(*srcPcl);

    //cout << "ŷ���ǣ�" << pcri._rotateXYZ.transpose() << endl;
    //// ���ư�ĳ����ת
    //cv::Mat rotMat = RotateRzMat(pcri._rotateXYZ[2]);
    ////cout << "��ת����Ϊ��" << rotMat << endl;
    //auto pclPtrSLRotate = make_shared<open3d::geometry::PointCloud>();
    //for (int pd = 0; pd < pclUse->points_.size(); pd++) {
    //    Eigen::Vector3d currPosition = pclUse->points_[pd];
    //    //cout << "before rotate:" << currPosition.transpose() << endl;
    //    Eigen::Vector3d cmeraPosition = currPosition - pcri._rotateCenter;
    //    cv::Mat cameraRotatedMat = rotMat * (cv::Mat_<double>(4, 1) << cmeraPosition[0], cmeraPosition[1], cmeraPosition[2], 1);
    //    Eigen::Vector3d rotatedPosition{ pcri._rotateCenter + Eigen::Vector3d{ cameraRotatedMat.at<double>(0,0), cameraRotatedMat.at<double>(1,0),cameraRotatedMat.at<double>(2,0)} };
    //    pclPtrSLRotate->points_.emplace_back(rotatedPosition);
    //    //cout << "after rotate:" << rotatedPosition.transpose() << endl;
    //    pclPtrSLRotate->colors_.emplace_back(pclUse->colors_[pd]);
    //}
    //return pclPtrSLRotate;

    // ������������ת��open3d˳ʱ��Ϊ������ʱ��Ϊ��
    Eigen::Matrix3d rotMat = pclUse->GetRotationMatrixFromXYZ(pcri._rotateXYZ);
    auto rotatedPointCloudTemp = pclUse->Rotate(rotMat, pcri._rotateCenter);
    return std::make_shared<open3d::geometry::PointCloud>(rotatedPointCloudTemp);
}

void GenerateBoundingBox(std::shared_ptr<open3d::geometry::PointCloud> srcPointCloud, vector<Eigen::Vector3d>& vEightPoint) {
    // ������С��Ӿ���
    open3d::geometry::AxisAlignedBoundingBox axisBox = srcPointCloud->GetAxisAlignedBoundingBox(); // ���ư�Χ��
    axisBox.color_ = Eigen::Vector3d(1, 0, 0);
    std::shared_ptr<geometry::AxisAlignedBoundingBox> pcl_ptr4 = std::make_shared<geometry::AxisAlignedBoundingBox>(axisBox); // ��Χ���Ӧ����
    //open3d::visualization::DrawGeometries({ inlier, pcl_ptr4 }, "BoundingBox");
    auto boxPoint = pcl_ptr4->GetBoxPoints();
    vector<Eigen::Vector3d> allPoint = srcPointCloud->points_;
    vector<Eigen::Vector3d> allPointColor = srcPointCloud->colors_;
    //for (int i = 0; i < boxPoint.size(); i++)
    //{
    //    cout << boxPoint[i].transpose() << endl;
    //}
    double BILI = 0.999999;
    double limitXLow = boxPoint[0][0]; // ����1
    limitXLow = limitXLow * (sign(limitXLow) > 0 ? 2 - BILI : BILI);
    double limitYLow = boxPoint[0][1]; // ����2
    limitYLow = limitYLow * (sign(limitYLow) > 0 ? 2 - BILI : BILI);
    double limitZLow = boxPoint[0][2];
    limitZLow = limitZLow * (sign(limitZLow) > 0 ? 2 - BILI : BILI);
    double limitXUpp = boxPoint[4][0]; // ����3
    limitXUpp = limitXUpp * (sign(limitXUpp) < 0 ? 2 - BILI : BILI);
    double limitYUpp = boxPoint[4][1]; // ����4
    limitYUpp = limitYUpp * (sign(limitYUpp) < 0 ? 2 - BILI : BILI);
    double limitZUpp = boxPoint[4][2];
    limitZUpp = limitZUpp * (sign(limitZUpp) < 0 ? 2 - BILI : BILI);
    Eigen::Vector3d vP1, vP2, vP3, vP4, vP5, vP6;
    for (int i = 0; i < allPoint.size(); i++)
    {
        if (allPoint[i][0] < limitXLow)
            vP1 = allPoint[i];
        if (allPoint[i][1] < limitYLow)
            vP2 = allPoint[i];
        if (allPoint[i][2] < limitZLow)
            vP3 = allPoint[i];
        if (allPoint[i][0] > limitXUpp)
            vP4 = allPoint[i];
        if (allPoint[i][1] > limitYUpp)
            vP5 = allPoint[i];
        if (allPoint[i][2] > limitZUpp)
            vP6 = allPoint[i];
    }
    ///*****************************open3d�����Χ�еķ�ʽ�ͷǽ��ܽӴ�ʱ�Լ�����Ķ��㷽ʽ��myBoundingBox���Ļ��߱Ƚ�
    /// ��Χ��8���㶨��                    ��Χ��(1')�ǽ������ʱ����ͼ
    ///      0 ------------------- 1    0 --------------1
    ///       /|                /|      |       /\      |
    ///      / |               / |      |      /  \     |
    ///     /  |              /  |      |     /    \    |
    ///    /   |             /   |      |    /      \   |
    /// 2 ------------------- 7  |      |   /        \  |
    ///   |    |____________|____| 6    |  /          \ |
    ///   |   /3            |   /       | /            \|7'
    ///   |  /              |  /        |/             /|
    ///   | /               | /       0'|\            / |
    ///   |/                |/          | \          /  |
    /// 5 ------------------- 4         2 --------------3
    ///                                         2'
    //cout << Eigen::Vector3d(vP1[0], vP1[1], limitZLow).transpose() << endl;
    //cout << Eigen::Vector3d(vP4[0], vP4[1], limitZLow).transpose() << endl;
    //cout << Eigen::Vector3d(vP5[0], vP5[1], limitZLow).transpose() << endl;
    //cout << Eigen::Vector3d(vP1[0], vP1[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP2[0], vP2[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP5[0], vP5[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP4[0], vP4[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP2[0], vP2[1], limitZLow).transpose() << endl;
    vEightPoint.emplace_back(Eigen::Vector3d(vP1[0], vP1[1], limitZLow)); // P0
    vEightPoint.emplace_back(Eigen::Vector3d(vP2[0], vP2[1], limitZLow)); // P1
    vEightPoint.emplace_back(Eigen::Vector3d(vP5[0], vP5[1], limitZLow)); // P2
    vEightPoint.emplace_back(Eigen::Vector3d(vP1[0], vP1[1], limitZUpp)); // P3
    vEightPoint.emplace_back(Eigen::Vector3d(vP4[0], vP4[1], limitZUpp)); // P4
    vEightPoint.emplace_back(Eigen::Vector3d(vP5[0], vP5[1], limitZUpp)); // P5
    vEightPoint.emplace_back(Eigen::Vector3d(vP2[0], vP2[1], limitZUpp)); // P6
    vEightPoint.emplace_back(Eigen::Vector3d(vP4[0], vP4[1], limitZLow)); // P7
    vector<Eigen::Vector2i> vLineIndexTemp; // ֱ�ߵ��������ж�Ӧ������
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 1));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 2));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 3));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(1, 6));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(1, 7));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(2, 5));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(2, 7));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(3, 5));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(3, 6));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(4, 5));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(4, 6));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(4, 7));
    vector<Eigen::Vector2i> vLineIndexBoxTemp; // ֱ�ߵ��������ж�Ӧ������
    vLineIndexBoxTemp.emplace_back(Eigen::Vector2i(0, 1));
    vLineIndexBoxTemp.emplace_back(Eigen::Vector2i(0, 2));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vEightPoint, vLineIndexTemp));
    std::shared_ptr<open3d::geometry::LineSet> lineSetBoxTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(boxPoint, vLineIndexBoxTemp));
    //open3d::visualization::DrawGeometries({ srcPointCloud, lineSetTemp_cloud,lineSetBoxTemp_cloud, pcl_ptr4 }, "BoundingBox");
}

void ProjectPointCloudFromZOrient(std::shared_ptr<open3d::geometry::PointCloud>& srcPointCloud, vector<Eigen::Vector3d>& vBoundingBox, cv::Mat& img, int& validPixel, uchar outImgType, bool& dir)
{
    // ����AxisBoundingBox����z�����¿��Ķ�άͼ��
    // ���Ƶ������190000��2Dͼ��ֱ��ʲ���
    vector<Eigen::Vector3d> vAllPoint = srcPointCloud->points_;
    vector<Eigen::Vector3d> vAllColor = srcPointCloud->colors_;
    // �̶�x�ĳ���360��ȷ��y�ĳ���
    int rowNum = 240;
    double rowNumTemp = (vBoundingBox[0] - vBoundingBox[2]).norm(); // y����
    double colNumTemp = (vBoundingBox[0] - vBoundingBox[1]).norm(); // x����
    double zValNumTemp = (vBoundingBox[0] - vBoundingBox[3]).norm(); // z����
    int colNum = (int)((double)(colNumTemp * rowNum) / rowNumTemp);
    // ��¼��Χ��
    //Eigen::MatrixXd outImgDouble(rowNum, colNum); // �����ع�һ����rowNum*colNum��ͼƬ��
    cv::Mat binImgInv(rowNum, colNum, CV_8U); // ��ɫ���޵����ڸ�λ���ϣ�����ɫ���е����ڸ�λ���ϣ�
    cv::Mat garyImgInv(rowNum, colNum, CV_8U); // ͼƬ��ɫ�͵�ǰ�㵽��ߵ���Ȳ����
    cv::Mat rgbImgInv(rowNum, colNum, CV_8UC3); // �õ����е����ɫ��ͼƬ��ɫ
    validPixel = 0;
    for (int i = 0; i < vAllPoint.size(); i++)
    {
        double disCol = abs(vAllPoint[i][0] - vBoundingBox[0][0]); // xƫ�� - col
        double disRow = abs(vAllPoint[i][1] - vBoundingBox[0][1]); // yƫ�� - row
        double disZ = abs(vAllPoint[i][2] - vBoundingBox[4][2]); // zƫ�� - z
        // ��i�����ӦͼƬ�ϵ����꣨nowRow��nowCol��
        int nowRow = (disRow * rowNum) / rowNumTemp - 1;
        int nowCol = (disCol * colNum) / colNumTemp - 1;
        nowRow = nowRow < 0 ? 0 : nowRow;
        nowCol = nowCol < 0 ? 0 : nowCol;
        int nowZVal = 255 - (disZ * 255.) / zValNumTemp; // ��һ����255������ƽ��Խ������ֵԽ��
        if (binImgInv.at<unsigned char>(nowRow, nowCol) == 255)
        {
            // ��ǰz��֮ǰ������Ϸ�������֮ǰ�ĵ�
            if (nowZVal > garyImgInv.at<uchar>(nowRow, nowCol))
            {
                garyImgInv.at<uchar>(nowRow, nowCol) = nowZVal;
                rgbImgInv.at<cv::Vec3b>(nowRow, nowCol) = cv::Vec3b(255 * vAllColor[i][0], 255 * vAllColor[i][1], 255 * vAllColor[i][2]);
            }
            if ((nowRow <= rowNum / 2) && (nowCol <= colNum / 2))
                validPixel -= 1;
        }
        else
        {
            garyImgInv.at<uchar>(nowRow, nowCol) = nowZVal;
            rgbImgInv.at<cv::Vec3b>(nowRow, nowCol) = cv::Vec3b(255 * vAllColor[i][0], 255 * vAllColor[i][1], 255 * vAllColor[i][2]);
        }
        binImgInv.at<unsigned char>(nowRow, nowCol) = 255;
        if((nowRow <= rowNum / 2) && (nowCol <= colNum / 2))
            validPixel += 1;
    }
    cv::Mat binImg, garyImg, rgbImg;
    // >0ˮƽ =0��ֱ <0ˮƽ����ֱ
    //flip(binImgInv, binImg, 0);
    //flip(binImgInv, garyImg, 0);
    //flip(binImgInv, rgbImg, 0);
    cv::flip(binImgInv, binImg, -1);
    cv::flip(binImgInv, garyImg, -1);
    cv::flip(binImgInv, rgbImg, -1);
    if (outImgType == 0)
        img = cv::Mat(binImg);
    else if (outImgType == 1)
        img = cv::Mat(garyImg);
    else
        img = cv::Mat(rgbImg);

    cv::Mat blurImg, binary, dst;
    cv::GaussianBlur(binImg, blurImg, cv::Size(5, 5), 1); // ��˹ģ��
    cv::threshold(blurImg, binary, 128, 255, cv::THRESH_BINARY); // ��ֵ��
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, dst, cv::MORPH_CLOSE, kernel); // �ղ��� �������ڸ�ʴ

    // ��ȡ1/4��2/4��3/4�У�1/4��2/4��3/4�������жϺڰ�����ʱ��λ�ã��ɴ��жϳ���
    dir = 0;
    vector<int> jumpRow(3, 0), jumpCol(3, 0);
    for (int j = 0; j < binImg.cols/2; j++) // ������
    {
        if ((binary.at<uchar>(rowNum / 4, j) == 0) && (binary.at<uchar>(rowNum / 4, j + 1) == 255)
            && (binary.at<uchar>(rowNum / 4, j + 2) == 255) && (binary.at<uchar>(rowNum / 4, j + 3) == 255)) 
        {
            jumpRow[0] = j;
        }
        if ((binary.at<uchar>(rowNum * 2 / 4, j) == 0) && (binary.at<uchar>(rowNum * 2 / 4, j + 1) == 255)
            && (binary.at<uchar>(rowNum * 2 / 4, j + 2) == 255) && (binary.at<uchar>(rowNum * 2 / 4, j + 3) == 255))
        {
            jumpRow[1] = j;
        }
        if ((binary.at<uchar>(rowNum * 3 / 4, j) == 0) && (binary.at<uchar>(rowNum * 3 / 4, j + 1) == 255)
            && (binary.at<uchar>(rowNum * 3 / 4, j + 2) == 255) && (binary.at<uchar>(rowNum * 3 / 4, j + 3) == 255))
        {
            jumpRow[2] = j;
        }
    }
    for (int i = 0; i < binImg.rows / 2; i++) // ������
    {
        if ((binary.at<uchar>(i, colNum / 4) == 0) && (binary.at<uchar>(i + 1, colNum / 4) == 255)
            && (binary.at<uchar>(i + 2, colNum / 4) == 255) && (binary.at<uchar>(i + 3, colNum / 4) == 255))
        {
            jumpCol[0] = i;
        }
        if ((binary.at<uchar>(i, colNum * 2 / 4) == 0) && (binary.at<uchar>(i + 1, colNum * 2 / 4) == 255)
            && (binary.at<uchar>(i + 2, colNum * 2 / 4) == 255) && (binary.at<uchar>(i + 3, colNum * 2 / 4) == 255))
        {
            jumpCol[1] = i;
        }
        if ((binary.at<uchar>(i, colNum * 3 / 4) == 0) && (binary.at<uchar>(i + 1, colNum * 3 / 4) == 255)
            && (binary.at<uchar>(i + 2, colNum * 3 / 4) == 255) && (binary.at<uchar>(i + 3, colNum * 3 / 4) == 255))
        {
            jumpCol[2] = i;
        }
    }
    if ((jumpRow[2] > jumpRow[1]) && (jumpRow[1] >= jumpRow[0]))
    {
        dir = false;
        return;
    }
    if ((jumpRow[2] <= jumpRow[1]) && (jumpRow[1] < jumpRow[0]))
    {
        dir = true;
        return;
    }
    if ((jumpCol[2] > jumpCol[1]) && (jumpCol[1] >= jumpCol[0]))
    {
        dir = false;
        return;
    }
    if ((jumpCol[2] <= jumpCol[1]) && (jumpCol[1] < jumpCol[0]))
    {
        dir = true;
        return;
    }

    //cout << "***********************************�����x��yƽ��ͶӰ�Լ�ͶӰ��ͼƬ��б������������" << endl;
    //cout << jumpRow[2] << "   " << jumpRow[1] << "   " << jumpRow[0] << endl;
    //cout << jumpCol[2] << "   " << jumpCol[1] << "   " << jumpCol[0] << endl;
    //cout << "dir��" << dir << endl;
    //cout << "rowNum��" << rowNum << endl;
    //cout << "colNum��" << colNum << endl;
    //cout << "outImg.size()��" << binImg.size() << endl;
    //cout << "vAllPoint.size()��" << vAllPoint.size() << endl;
    //cout << "validPixel��" << validPixel << endl;
    //cout << "**********************************************************************" << endl;
    ////cout << "nowRow��" << nowRow << "nowCol��" << nowCol << endl;
    //cv::imshow("binImg", binImg);
    //cv::imshow("garyImg", garyImg);
    //cv::imshow("rgbImg", rgbImg);
    //cv::waitKey(0);
}

void FromImgGetXYOrient(cv::Mat& src, int validCnt, bool leanDir, Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient, int lengthMoreWidth)
{
    // �ǶȷŴ��˴�Լ4���������û����2��������û����2��
    // �ڱ������recImg.size() - validCnt
    int rowNum = src.rows;
    int colNum = src.cols;
    int blackArea = src.rows * src.cols / 4 - validCnt;
    // ��������ȣ���/����+�У�**2
    double areaRatio = ((double)src.rows) / (src.cols + src.rows);    //double areaRatio = pow((recImg.rows / (recImg.cols + recImg.rows)), 2);
    areaRatio = 0.75;//1.��0.125;
    int rowValidCnt = blackArea * areaRatio;    //int rowValidCnt = blackArea * areaRatio / 2;
    int colValidCnt = blackArea * areaRatio;    //int colValidCnt = blackArea * (1 - areaRatio) / 2;
    double xAngle = atan((double)rowValidCnt / rowNum / rowNum) / CV_PI * 180;
    double yAngle = atan((double)colValidCnt / colNum / colNum) / CV_PI * 180;
    if (leanDir == true)
    {
        yOrient = Eigen::Vector3d(100 * tan(xAngle / 180. * CV_PI), 100, 0);
        yOrient = yOrient.normalized();
        xOrient = Eigen::Vector3d(100, -100 * tan(yAngle / 180. * CV_PI), 0);
        xOrient = xOrient.normalized();
    }
    else
    {
        yOrient = Eigen::Vector3d(-tan(xAngle / 180. * CV_PI) * 100, 100, 0);
        yOrient = yOrient.normalized();
        xOrient = Eigen::Vector3d(100, 100 * tan(yAngle / 180. * CV_PI), 0);
        xOrient = xOrient.normalized();
    }
    //if (!lengthMoreWidth)
    {
        // Ĭ��Ҫ����
        Eigen::Vector3d tempVec(yOrient); // ��������߶�Ӧ��x
        yOrient = xOrient;
        xOrient = tempVec;
    }
    
    cout << "***********************************�����x��y�������������" << endl;
    cout << "blackArea��" << blackArea << endl;
    cout << "validCnt��" << validCnt << "   rowValidCnt��" << rowValidCnt << "   colValidCnt��" << colValidCnt << endl;
    cout << "xAngle��" << xAngle << "   yAngle��" << yAngle << endl;
    cout << "**********************************************************************" << endl;

    // ������ʾ
    cv::Point p1(100, 100), p2(tan(xAngle / 180. * CV_PI) * 100, 100), p3(100, -tan(yAngle / 180. * CV_PI) * 100);
    cv::line(src, p1, p2 + p1, cv::Scalar(0, 0, 255));
    cv::line(src, p1, p3 + p1, cv::Scalar(0, 0, 255));
    //cv::imshow("src", src);
    //cv::waitKey(0);

    /// ������С��Ӿ��Σ�ȷ����б�ǣ�Ч��������
    //vector<cv::Mat> contours; 
    //Mat hierarcy;
    //cv::findContours(binary, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    //cout << "num=" << contours.size() << endl;
    //vector<Rect> boundRect(contours.size());  //������Ӿ��μ���
    //vector<RotatedRect> box(contours.size()); //������С��Ӿ��μ���
    //Point2f rect[4];
    //int maxIndex = 0, maxArea = 0;
    //for (int i = 0; i < contours.size(); i++)
    //{
    //    int nowArea = cv::contourArea(contours[i]);
    //    if (maxArea < nowArea)
    //    {
    //        maxIndex = i;
    //        maxArea = nowArea;
    //    }
    //}
    //cout << "maxArea��" << maxArea << "������" << dst.size() <<  endl;
    //int i = maxIndex;
    //box[i] = minAreaRect(Mat(contours[i]));  //����ÿ��������С��Ӿ���
    //boundRect[i] = boundingRect(Mat(contours[i]));
    ////������С��Ӿ��ε����ĵ�
    //box[i].points(rect);  //����С��Ӿ����ĸ��˵㸴�Ƹ�rect����
    //rectangle(dst, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x +
    //    boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
    //for (int j = 0; j < 4; j++)
    //{
    //    line(dst, rect[j], rect[(j + 1) % 4], Scalar(255, 0, 0), 2, 8);  //������С��Ӿ���ÿ����
    //}
    //Eigen::Vector2d xOrient((rect[0] - rect[1]).x, (rect[0] - rect[1]).y);
    //Eigen::Vector2d yOrient((rect[0] - rect[3]).x, (rect[0] - rect[3]).y);
    //cout << "rect[0]��" << rect[0] << "   rect[1]��" << rect[1] << "rect[2]��" << rect[2] << "   rect[3]��" << rect[3] << endl;
    //cout << "xOrient��" << xOrient.normalized().transpose() << "   yOrient��" << yOrient.normalized().transpose() << endl;
    //cv::imshow("2DImage", dst);
    //cv::waitKey(0);
}

std::shared_ptr<open3d::geometry::LineSet> DrawXYZAxisAtOrient(const drawAxisInfo dai) {
    //std::shared_ptr<open3d::geometry::LineSet> ls = std::make_shared<open3d::geometry::LineSet>();
    // ����������λ��
    vector<Eigen::Vector3d> vAxisPoint;
    vAxisPoint.emplace_back(dai._center); // ��Ҫ����ֱ�����ڵ�
    vAxisPoint.emplace_back(dai._center + dai._xLen * Eigen::Vector3d(1., 0, 0));
    vAxisPoint.emplace_back(dai._center + dai._yLen * Eigen::Vector3d(0, 1., 0));
    vAxisPoint.emplace_back(dai._center + dai._zLen * Eigen::Vector3d(0, 0, 1.));
    vector<Eigen::Vector2i> vAxisIndex; // ֱ�ߵ��������ж�Ӧ������
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 1));
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 2));
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 3));

    return std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vAxisPoint, vAxisIndex));
}

std::shared_ptr<open3d::geometry::LineSet> DrawDeviceLinePointCloud(Eigen::Vector3d position, Eigen::Vector3d xOrient, Eigen::Vector3d yOrient, Eigen::Vector3d zOrient) {
    double uppPlainLength = DEVICE_LENGTH / 4, uppPlainWidth = DEVICE_WIDTH / 4;
    //double uppPlainLength = 0., uppPlainWidth = 0.;
    vector<Eigen::Vector3d> vAxisPoint;
    //vAxisPoint.emplace_back(position); // ��Ҫ����ֱ�����ڵ�
    vAxisPoint.emplace_back(position + xOrient * uppPlainLength + yOrient * uppPlainWidth);
    vAxisPoint.emplace_back(position + xOrient * uppPlainLength - yOrient * uppPlainWidth);
    vAxisPoint.emplace_back(position - xOrient * uppPlainLength + yOrient * uppPlainWidth);
    vAxisPoint.emplace_back(position - xOrient * uppPlainLength - yOrient * uppPlainWidth);
    //vAxisPoint.emplace_back(position + zOrient * plain2cam);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) + xOrient * DEVICE_LENGTH /2 + yOrient * DEVICE_WIDTH /2);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) + xOrient * DEVICE_LENGTH/2 - yOrient * DEVICE_WIDTH/2);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) - xOrient * DEVICE_LENGTH /2 + yOrient * DEVICE_WIDTH /2);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) - xOrient * DEVICE_LENGTH /2 - yOrient * DEVICE_WIDTH /2);
    vector<Eigen::Vector2i> vAxisIndex; // ֱ�ߵ��������ж�Ӧ������
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 1)); // ��ƽ��
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 2));
    vAxisIndex.emplace_back(Eigen::Vector2i(1, 3));
    vAxisIndex.emplace_back(Eigen::Vector2i(2, 3));
    vAxisIndex.emplace_back(Eigen::Vector2i(4, 5)); // ��ƽ��
    vAxisIndex.emplace_back(Eigen::Vector2i(4, 6));
    vAxisIndex.emplace_back(Eigen::Vector2i(5, 7));
    vAxisIndex.emplace_back(Eigen::Vector2i(6, 7)); 
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 4)); // �ϡ���ƽ��������
    vAxisIndex.emplace_back(Eigen::Vector2i(1, 5));
    vAxisIndex.emplace_back(Eigen::Vector2i(2, 6));
    vAxisIndex.emplace_back(Eigen::Vector2i(3, 7));
    //std::shared_ptr<open3d::geometry::LineSet> device_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vAxisPoint, vAxisIndex));
    //return device_cloud;
    return std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vAxisPoint, vAxisIndex));
}

vector<Eigen::Vector3d> GetAngleBounding(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, bool hd) {
    /*
    * x -> ���z���� z -> ���x����
    * 1��A0x + B0y + C0z = cos�� postrueCamZ
    * 2��A1x + B1y + C1z = sin�� postrueCamY
    * 3��A2x + B2y + C2z = 0   postrueCamX
    * 123 --> (A2B0 - A0B2)y + (A2C0 - A0C2)z = A2cos��
    *         (A2B1 - A1B2)y + (A2C1 - A1C2)z = A2sin�� 
    *         (B2A1 - B1A2)x + (B2C1 - B1C2)z = B2sin�� 
    *  �ǣ�b0 = A2B0 - A0B2��c0 = A2C0 - A0C2��d0 = A2cos��
    *      b1 = A2B1 - A1B2��c1 = A2C1 - A1C2��d1 = A2sin��
    *      a2 = B2A1 - B1A2��c2 = B2C1 - B1C2��d2 = B2sin��
    * 4��      b0y + c0z = d0
    * 5��      b1y + c1z = d1
    * 6��      a2x + c2z = d2
    * 456 ---> (c1b0 - c0b1)y = c1d0 - c0d1
    *          (b1c0 - b0c1)z = b1d0 - b0d1
    *  �ǣ�tempY = (c1b0 - c0b1)��tempD0 = c1d0 - c0d1
    *      tempZ = (b1c0 - b0c1)��tempD1 = b1d0 - b0d1
    * 7��      tempYy = tmepD0
    * 8��      tempZz = tempD1
    *          a2x = d2 - c2z
    */
    vector<Eigen::Vector3d> v; // ǰ�ĸ���FOV���������������ĸ���FOV�ĸ�����λ��
    Eigen::Vector3d temp = y;
    y = z;
    z = temp;
    Eigen::Vector3d tempX = x;
    Eigen::Vector3d tempY = y;
    Eigen::Vector3d tempZ = z;
    double angle;
    if (hd) angle = HORIZON_ANGLE_HD;
    else angle = HORIZON_ANGLE;
    for (int i = 0; i < 4; i++) {
        if (i == 1) {
            tempY = -y;
            tempZ = z;
            if (hd) angle = HORIZON_ANGLE_HD;
            else angle = HORIZON_ANGLE;
        }
        else if (i == 2) {
            tempY = z;
            tempZ = y;
            if (hd) angle = VERTICAL_ANGLE_HD;
            else angle = VERTICAL_ANGLE;
        }
        else if(i == 3) {
            tempY = -z;
            tempZ = y;
            if (hd) angle = VERTICAL_ANGLE_HD;
            else angle = VERTICAL_ANGLE;
        }
        double A0 = tempX[0], B0 = tempX[1], C0 = tempX[2];
        double A1 = tempY[0], B1 = tempY[1], C1 = tempY[2];
        double A2 = tempZ[0], B2 = tempZ[1], C2 = tempZ[2];
        double b0 = A2 * B0 - A0 * B2, c0 = A2 * C0 - A0 * C2;
        double b1 = A2 * B1 - A1 * B2, c1 = A2 * C1 - A1 * C2;
        double a2 = B2 * A1 - B1 * A2, c2 = B2 * C1 - B1 * C2;
        double d0 = A2 * cos(angle * CV_PI / 180), d1 = A2 * sin(angle * CV_PI / 180), d2 = B2 * sin(angle * CV_PI / 180);
        double tempY = c1 * b0 - c0 * b1;
        double tempZ = b1 * c0 - b0 * c1;
        double tempD0 = c1 * d0 - c0 * d1;
        double tempD1 = b1 * d0 - b0 * d1;
        // double y, z, x;
        //double yComp = abs(tempY) < 1e-3 ? 0 : tempD0 / tempY;
        //double zComp = abs(tempZ) < 1e-3 ? 0 : tempD1 / tempZ;
        double zComp = tempD1 / tempZ;
        //double yComp = tempD0 / tempY;
        double yComp = (d1 - c1 * zComp) / b1;
        double xComp = (d2 - c2 * zComp) / a2;
        v.emplace_back(Eigen::Vector3d(xComp, yComp, zComp).normalized());

#if 0 // 1

    cout << "hor��" << HORIZON_ANGLE << "  ver��" << VERTICAL_ANGLE << endl;
    cout << "x��" << x.transpose() << "  y��" << y.transpose() << "   z��" << z.transpose() << endl;
    cout << "b0��" << b0 << "  c0��" << c0 << "  d0��" << d0 << endl;
    cout << "b1��" << b1 << "  c1��" << c1 << "  d1��" << d1 << endl;
    cout << "a2��" << a2 << "  c2��" << c2 << "  d2��" << d2 << endl;
    cout << "tempY��" << tempY << "  tempD0��" << tempD0 << endl;
    cout << "tempZ��" << tempZ << "  tempD1��" << tempD1 << endl;
    cout << "xComp��" << xComp << "  yComp��" << yComp << "  zComp��" << zComp << endl;

#endif

    }

    // ����GetAngleBounding�ĸ�����������ͼƬ�ĸ��������λ��
    Eigen::Vector3d xFovOrient; // v[0] / cos(hor * CV_PI / 180)��б�߳���xFovOrient������ƽ��x����
    Eigen::Vector3d yFovOrient; // v[2] / cos(ver * CV_PI / 180)��б�߳���yFovOrient������ƽ��y����
    if (hd) {
        xFovOrient = x - v[0] / cos(HORIZON_ANGLE_HD * CV_PI / 180);
        yFovOrient = x - v[2] / cos(VERTICAL_ANGLE_HD * CV_PI / 180);
    }
    else {
        xFovOrient = x - v[0] / cos(HORIZON_ANGLE * CV_PI / 180);
        yFovOrient = x - v[2] / cos(VERTICAL_ANGLE * CV_PI / 180);
    }
    v.emplace_back(x + xFovOrient + yFovOrient);
    v.emplace_back(x + xFovOrient - yFovOrient);
    v.emplace_back(x - xFovOrient + yFovOrient);
    v.emplace_back(x - xFovOrient - yFovOrient);
    v.emplace_back(xFovOrient.normalized());
    v.emplace_back(yFovOrient.normalized());

    //cout << "cos(hor)��" << cos(hor) << "   sin(hor)��" << sin(hor) << endl;
    //cout << "cos(ver)��" << cos(ver) << "   sin(ver)��" << sin(ver) << endl;
    //cout << "cos(45)��" << cos(45) << "   sin(45)��" << sin(45) << endl;
    //cout << "v[0]��" << v[0].transpose() << "  v[1]��" << v[1].transpose() << endl;
        
    return  v;
}

bool VectorInRange(Eigen::Vector3d vec, Eigen::Vector3d camZOrient)
{
    bool ret = true;

    double angle = acos(vec.transpose() * camZOrient) * 180 / CV_PI;
    if (abs(angle) < abs(EDGE_ANGLE))
    {
        return true;
    }
    else
    {
        return false;
    }

    return ret;
}

///*vector<cv::Mat>*/void PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose, vector<cv::Mat>& vMat)
vector<cv::Mat> PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose) {
    vector<cv::Mat> vMat;

    Eigen::Vector3d positionCam(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3)); // λ��
    Eigen::Vector3d postrueCamZ(pose.at<double>(0, 2), pose.at<double>(1, 2), pose.at<double>(2, 2)); // ����
    Eigen::Vector3d postrueCamY(pose.at<double>(0, 1), pose.at<double>(1, 1), pose.at<double>(2, 1)); // ����
    Eigen::Vector3d postrueCamX(pose.at<double>(0, 0), pose.at<double>(1, 0), pose.at<double>(2, 0)); // ����
    // ����������߷��򣬸�������ӳ���ȷ����Ұ��Χ
    // �ӽǷ�ΧvBoundingVector[0]��[1]->���ߣ�vBoundingVector[2]��[3]->�̱�
    vector<Eigen::Vector3d> vBoundingVector = GetAngleBounding(postrueCamZ, postrueCamY, postrueCamX, 0);
    double edgeAngle = EDGE_ANGLE * 180 / CV_PI; // angleEdge�����㵽���z����ļн�
    Eigen::Vector3d planeXAxis = vBoundingVector[8], planeYAxis = vBoundingVector[9];
    //cout << "planeXAxis��" << planeXAxis.transpose() << "planeYAxis��" << planeYAxis.transpose() << endl;
    cv::Mat binImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U, cv::Scalar(0));
    cv::Mat garyImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_64F, cv::Scalar(0.));
    //cv::Mat garyImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U, cv::Scalar(0));
    cv::Mat rgbImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    // ���λ��
    for (int i = 0; i < pd->points_.size(); i++) {
        // �������пռ��
        Eigen::Vector3d nowVec = (pd->points_[i] - positionCam);
        double zDistance = nowVec.norm() * nowVec.normalized().transpose() * postrueCamZ;
        nowVec = nowVec.normalized(); // ��ǰ���ָ����Ƶ�ǰ��
        double angle = acos(nowVec.transpose() * postrueCamZ) * 180 / CV_PI; // ��z��н��ж��Ƿ����ӳ���Χ��
        //if (abs(angle) > abs(EDGE_ANGLE * 180 / CV_PI)) {
        //    // �����ڲ�
        //    continue;
        //}
        //double xResult = nowVec.transpose() * planeXAxis; // ��x��н��ж���x���������� 
        //double yResult = nowVec.transpose() * planeYAxis; // ��y��н��ж���y����������
        Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance;
        //Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance * sin(angle * CV_PI / 180); // ƽ�����Ĵ�dsin(angle)
        Eigen::Vector3d planeVec = pd->points_[i] - planeOri;
        double xDistance = planeVec.norm() * planeVec.normalized().transpose() * planeXAxis;
        double yDistance = planeVec.norm() * planeVec.normalized().transpose() * planeYAxis; // dcos
        double xAllDistance = zDistance * tan(HORIZON_ANGLE * CV_PI / 180);
        double yAllDistance = zDistance * tan(VERTICAL_ANGLE * CV_PI / 180);
        int xPos = ((xDistance / xAllDistance) / 2.) * IMAGE_WIDTH;
        int yPos = ((yDistance / yAllDistance) / 2.) * IMAGE_HEIGHT;
        if ((abs(xPos) >= IMAGE_WIDTH / 2) || (abs(yPos) >= IMAGE_HEIGHT / 2)) {
            // �±�Խ��
            continue;
        }
        //if ((pd->points_[i] - Eigen::Vector3d(628.667, -952.119, -917.092)).norm() < 0.5) {
        //    //cout << "���Ե�����Ϊ��" << xPos + IMAGE_WIDTH_HD / 2 << "  " << yPos + IMAGE_HEIGHT_HD / 2 << endl;
        //    cout << "���Ե�����Ϊ��" << -xPos + IMAGE_WIDTH / 2 << "  " << -yPos + IMAGE_HEIGHT / 2 << endl;
        //}
        if (binImage.at<uchar>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) == 255) {
            float lastDistance = garyImage.at<double>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2);
            if (lastDistance > (pd->points_[i] - positionCam).norm()) {
                // ���ϴεĽ�-���滻 
                garyImage.at<double>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = (pd->points_[i] - positionCam).norm(); // ���
                rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            }
        }
        else {
            rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            garyImage.at<double>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = (pd->points_[i] - positionCam).norm(); // ���
        }
        binImage.at<uchar>(yPos + (IMAGE_HEIGHT / 2), xPos + (IMAGE_WIDTH / 2)) = (uchar)255;
        ////cout << "points_.size()��" << pcl_ptr->points_.size() << endl;
        ////cout << "xDistance/xAllDistance��" << xDistance / xAllDistance << "   yDistance/yAllDistance��" << yDistance / yAllDistance << endl;
        ////cout << "xPos��" << xPos << "   yPos��" << yPos << endl;
    }
    // 1
    //double maxVal1 = *max_element(vDup[1].begin<double>(), vDup[1].end<double>());
    //double minVal1 = *min_element(vDup[1].begin<double>(), vDup[1].end<double>());
    //std::cout << "���ֵ��" << maxVal1 << "��Сֵ��" << minVal1 << std::endl;
    // 2
    //double minValue, maxValue;    // ���ֵ����Сֵ
    //cv::Point  minIdx, maxIdx;    // ��Сֵ���꣬���ֵ����     
    //cv::minMaxLoc(vDup[1], &minValue, &maxValue, &minIdx, &maxIdx);
    //std::cout << "���ֵ��" << maxValue << "��Сֵ��" << minValue << std::endl;
    //std::cout << "���ֵλ�ã�" << maxIdx << "��Сֵλ�ã�" << minIdx;
    // 3
    double maxVal = 0., minVal = INT32_MAX;
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
        for (int col = 0; col < IMAGE_WIDTH; col++) {
            double currVal = garyImage.at<double>(row, col);
            if ((int)garyImage.at<double>(row, col) == 0) continue;
            maxVal = max(maxVal, currVal);
            minVal = min(minVal, currVal);
        }
    }
    cv::Mat gary;
    garyImage.convertTo(gary, CV_8U, 255. / (maxVal - minVal));
    //std::cout << "���ֵ��" << maxVal << "��Сֵ��" << minVal << std::endl;
    garyImage = gary;

    cv::flip(binImage, binImage, -1); // ͼƬˮƽ����ֱ���Ǿ����
    cv::flip(garyImage, garyImage, -1); // ͼƬˮƽ����ֱ���Ǿ����
    cv::flip(rgbImage, rgbImage, -1); // ͼƬˮƽ����ֱ���Ǿ����

    cv::Mat dst, dst1;
    //cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    //cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    //cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::dilate(binImage, dst1, kernel1);
    //cv::erode(dst1, dst, kernel2);

    vMat.emplace_back(binImage);
    vMat.emplace_back(garyImage);
    vMat.emplace_back(rgbImage);
    //vMat.emplace_back(dst);
    vMat.emplace_back(dst1);
    //cv::imshow("rgbImage", rgbImage);
    //cv::waitKey(0);

#if 0 // 1   

    cout << "*******************************************************************����͸�Ӳ��ԣ�" << endl;

    cout << "fourPostrue.size()��" << pose.size() << endl;
    cout << "fourPostrue��" << pose << endl;
    cout << "positionCam��" << positionCam.transpose() << endl;
    cout << "postrueCamZ��" << postrueCamZ.transpose() << endl;
    cout << "vBoundingVector.size()��" << vBoundingVector.size() << endl;
    cout << "vBoundingVector[0] �� x��нǣ�" << acos(postrueCamX.transpose() * vBoundingVector[0]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[0] �� y��нǣ�" << acos(postrueCamY.transpose() * vBoundingVector[0]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[0] �� z��нǣ�" << acos(postrueCamZ.transpose() * vBoundingVector[0]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[2] �� x��нǣ�" << acos(postrueCamX.transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[2] �� y��нǣ�" << acos(postrueCamY.transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[2] �� z��нǣ�" << acos(postrueCamZ.transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[0]��" << vBoundingVector[0].transpose() << endl;
    cout << "vBoundingVector[1]��" << vBoundingVector[1].transpose() << endl;
    cout << "vBoundingVector[2]��" << vBoundingVector[2].transpose() << endl;
    cout << "vBoundingVector[3]��" << vBoundingVector[3].transpose() << endl;
    cout << "HORIZON_ANGLE��" << HORIZON_ANGLE << "  VERTICAL_ANGLE��" << VERTICAL_ANGLE << "   EDGE_ANGLE��" << edgeAngle << endl;
    cout << "************************************************************************************��" << endl;

    vector<Eigen::Vector3d> vDrawLinePoint; // ���������
    vector<Eigen::Vector2i> vDrawLineIndex; // ֱ�ߵ��������ж�Ӧ������
    // �߽��4������
    vDrawLinePoint.clear();
    vDrawLineIndex.clear();
    vDrawLinePoint.emplace_back(positionCam); // 0
    vDrawLinePoint.emplace_back(positionCam + postrueCamX * 50); // 1
    vDrawLinePoint.emplace_back(positionCam + postrueCamY * 100); // 2
    vDrawLinePoint.emplace_back(positionCam + postrueCamZ * 1200);  // 3
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[0] * 1500); // x 4
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[1] * 1500); // -x 5
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[2] * 1000); // y 6
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[3] * 1000); // -y 7
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4)); //
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 5));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 6));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 7));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    // �߽�ķ�������
    vDrawLinePoint.clear();
    vDrawLineIndex.clear();
    double zAxisLength = 800;
    vDrawLinePoint.emplace_back(positionCam); // 0
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[4]); // 1
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[5]); // 2
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[6]); // 3
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[7]); // 4
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1)); //
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 2)); //
    vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 4));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(3, 4));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 3));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud2 = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    open3d::visualization::DrawGeometries({ pd, lineSetTemp_cloud, lineSetTemp_cloud2 }, "CamLine");

#endif


    return vMat;
}

vector<cv::Mat> PerspectivePointCloudFromSpecialPoseHD(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose) {
    vector<cv::Mat> vMat;

    Eigen::Vector3d positionCam(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3)); // λ��
    Eigen::Vector3d postrueCamZ(pose.at<double>(0, 2), pose.at<double>(1, 2), pose.at<double>(2, 2)); // ����
    Eigen::Vector3d postrueCamY(pose.at<double>(0, 1), pose.at<double>(1, 1), pose.at<double>(2, 1)); // ����
    Eigen::Vector3d postrueCamX(pose.at<double>(0, 0), pose.at<double>(1, 0), pose.at<double>(2, 0)); // ����
    // ����������߷��򣬸�������ӳ���ȷ����Ұ��Χ
    // �ӽǷ�ΧvBoundingVector[0]��[1]->���ߣ�vBoundingVector[2]��[3]->�̱�
    vector<Eigen::Vector3d> vBoundingVector = GetAngleBounding(postrueCamZ, postrueCamY, postrueCamX, 1);
    Eigen::Vector3d planeXAxis = vBoundingVector[8], planeYAxis = vBoundingVector[9];
    //cout << "planeXAxis��" << planeXAxis.transpose() << "planeYAxis��" << planeYAxis.transpose() << endl;
    cv::Mat binImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8U, cv::Scalar(0));
    cv::Mat depthImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_64F, cv::Scalar(0.));
    //cv::Mat garyImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8U, cv::Scalar(0));
    cv::Mat rgbImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8UC3, cv::Scalar(0, 0, 0));
    // ���λ��
    for (int i = 0; i < pd->points_.size(); i++) {
        // �������пռ��
        Eigen::Vector3d nowVec = (pd->points_[i] - positionCam);
        double zDistance = 0.8 * nowVec.norm() * nowVec.normalized().transpose() * postrueCamZ;
        nowVec = nowVec.normalized(); // ��ǰ���ָ����Ƶ�ǰ��
        double angle = acos(nowVec.transpose() * postrueCamZ) * 180 / CV_PI; // ��z��н��ж��Ƿ����ӳ���Χ��
        //if (abs(angle) > abs(EDGE_ANGLE_HD * 180 / CV_PI)) {
        //    // �����ڲ� EDGE_ANGLE_HD * 180 / CV_PI�����㵽���z����ļн�
        //    continue;
        //}
        //double xResult = nowVec.transpose() * planeXAxis; // ��x��н��ж���x���������� 
        //double yResult = nowVec.transpose() * planeYAxis; // ��y��н��ж���y����������
        //Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance; // ƽ�����Ĵ�dsin(angle)
        Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance * sin(angle * CV_PI / 180); // ƽ�����Ĵ�dsin(angle)
        Eigen::Vector3d planeVec = pd->points_[i] - planeOri;
        double xDistance = planeVec.norm() * planeVec.normalized().transpose() * planeXAxis;
        double yDistance = planeVec.norm() * planeVec.normalized().transpose() * planeYAxis; // dcos
        double xAllDistance = zDistance * tan(HORIZON_ANGLE_HD * CV_PI / 180);
        double yAllDistance = zDistance * tan(VERTICAL_ANGLE_HD * CV_PI / 180);
        int xPos = ((xDistance / xAllDistance) / 2.) * IMAGE_WIDTH_HD;
        int yPos = ((yDistance / yAllDistance) / 2.) * IMAGE_HEIGHT_HD;
        if ((abs(xPos) >= IMAGE_WIDTH_HD / 2) || (abs(yPos) >= IMAGE_HEIGHT_HD / 2)) {
            // �±�Խ��
            continue;
        }

        //if ((pd->points_[i] - Eigen::Vector3d(628.037, -974.233, -940.113)).norm() < 0.5) {
        //    //cout << "���Ե�����Ϊ��" << xPos + IMAGE_WIDTH_HD / 2 << "  " << yPos + IMAGE_HEIGHT_HD / 2 << endl;
        //    cout << "���Ե���ͼƬ�ϵ�����Ϊ��" << -xPos + IMAGE_WIDTH_HD / 2 << "  " << -yPos + IMAGE_HEIGHT_HD / 2 << endl;
        //    cout << "���Ե���Ϣ1��" << planeVec.transpose() << "  x" << planeXAxis.transpose() << " y" << planeYAxis.transpose() << " " << postrueCamZ.transpose() << " " << angle << endl;
        //    cout << "���Ե���Ϣ2��" << zDistance << "  " << xDistance << " " << yDistance << " " << xAllDistance << " " << yAllDistance << endl;
        //    cout << "���Ե���y�нǣ�" << acos((pd->points_[i] - positionCam).normalized().transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
        //    cout << "���Ե������нǣ�" << acos((pd->points_[i] - positionCam).normalized().transpose() * postrueCamZ) * 180 / CV_PI << endl;
        //    cout << "y�����нǣ�" << acos(vBoundingVector[2].transpose() * postrueCamZ) * 180 / CV_PI << endl;
        //}

        if (binImage.at<uchar>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) == 255) {
            double lastDistance = depthImage.at<double>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2);
            if (lastDistance > (pd->points_[i] - positionCam).norm()) {
                // ���ϴεĽ�-���滻 
                depthImage.at<double>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = (pd->points_[i] - positionCam).norm(); // ���
                rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            }
        }
        else {
            rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            depthImage.at<double>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = (pd->points_[i] - positionCam).norm(); // ���
        }
        binImage.at<uchar>(yPos + (IMAGE_HEIGHT_HD / 2), xPos + (IMAGE_WIDTH_HD / 2)) = (uchar)255;
        ////cout << "points_.size()��" << pcl_ptr->points_.size() << endl;
        ////cout << "xDistance/xAllDistance��" << xDistance / xAllDistance << "   yDistance/yAllDistance��" << yDistance / yAllDistance << endl;
        ////cout << "xPos��" << xPos << "   yPos��" << yPos << endl;
    }
    // 1
    double maxVal1 = *max_element(depthImage.begin<double>(), depthImage.end<double>());
    double minVal1 = *min_element(depthImage.begin<double>(), depthImage.end<double>());
    //std::cout << "���ֵ��" << maxVal1 << "��Сֵ��" << minVal1 << std::endl;
    // 2
    //double minValue, maxValue;    // ���ֵ����Сֵ
    //cv::Point  minIdx, maxIdx;    // ��Сֵ���꣬���ֵ����     
    //cv::minMaxLoc(vDup[1], &minValue, &maxValue, &minIdx, &maxIdx);
    //std::cout << "���ֵ��" << maxValue << "��Сֵ��" << minValue << std::endl;
    //std::cout << "���ֵλ�ã�" << maxIdx << "��Сֵλ�ã�" << minIdx;
    // 3
    double maxVal = 0., minVal = INT32_MAX;
    for (int row = 0; row < IMAGE_HEIGHT_HD; row++) {
        for (int col = 0; col < IMAGE_WIDTH_HD; col++) {
            double currVal = depthImage.at<double>(row, col);
            if ((int)depthImage.at<double>(row, col) == 0) continue;
            maxVal = max(maxVal, currVal);
            minVal = min(minVal, currVal);
        }
    }
    cv::Mat garyImage;
    depthImage.convertTo(garyImage, CV_8U, 255. / (maxVal - minVal));
    //depthImage.convertTo(gary, CV_8U, 255. / (maxVal1 - minVal1));

    cv::flip(binImage, binImage, -1); // ͼƬˮƽ����ֱ���Ǿ����
    cv::flip(depthImage, depthImage, -1); // ͼƬˮƽ����ֱ���Ǿ����
    cv::flip(rgbImage, rgbImage, -1); // ͼƬˮƽ����ֱ���Ǿ����
    cv::flip(garyImage, garyImage, -1); // ͼƬˮƽ����ֱ���Ǿ����

    cv::Mat dst, dst1;
    //cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    //cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    //cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::dilate(binImage, dst1, kernel1);
    cv::erode(dst1, dst, kernel2);

    vMat.emplace_back(binImage);
    vMat.emplace_back(garyImage);
    vMat.emplace_back(rgbImage);
    vMat.emplace_back(dst);
    vMat.emplace_back(depthImage);
    //cv::imshow("rgbImage", rgbImage);
    //cv::waitKey(0);
    
    //vector<Eigen::Vector3d> vDrawLinePoint; // ���������
    //vector<Eigen::Vector2i> vDrawLineIndex; // ֱ�ߵ��������ж�Ӧ������
    //// �߽��4������
    //vDrawLinePoint.clear();
    //vDrawLineIndex.clear();
    //vDrawLinePoint.emplace_back(positionCam); // 0
    //vDrawLinePoint.emplace_back(positionCam + postrueCamX * 50); // 1
    //vDrawLinePoint.emplace_back(positionCam + postrueCamY * 100); // 2
    //vDrawLinePoint.emplace_back(positionCam + postrueCamZ * 500);  // 3
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[0] * 1500); // x 4
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[1] * 1500); // -x 5
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[2] * 1000); // y 6
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[3] * 1000); // -y 7
    //vDrawLinePoint.emplace_back(Eigen::Vector3d(628.037, -974.233, -940.113)); // ���Ե�
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4)); //
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 5));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 6));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 7));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 8)); // ���Ե�
    //std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    //// �߽�ķ�������
    //vDrawLinePoint.clear();
    //vDrawLineIndex.clear();
    //double zAxisLength = 800;
    //vDrawLinePoint.emplace_back(positionCam); // 0
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[4]); // 1
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[5]); // 2
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[6]); // 3
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[7]); // 4
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1)); //
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 2)); //
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(3, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 3));
    //std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud2 = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    //// ����xyz������
    //std::shared_ptr<open3d::geometry::LineSet> originandaxis_cloud;
    //DrawXYZAxisAtOrient(originandaxis_cloud);
    //open3d::visualization::DrawGeometries({ pd, lineSetTemp_cloud, lineSetTemp_cloud2, originandaxis_cloud,  }, "CamLine");

    return vMat;
}

vector<Eigen::Vector3d> SchdimtOrthogonalityForPoseture(const vector<Eigen::Vector3d>& src) {
    vector<Eigen::Vector3d> dst{ 3 };
    if (src.size() != 3) {
        cout << "�������룬ά����Ϊ3" << endl;
        return dst;
    }
    // ��ά˳��rz ry rx
    dst[0] = src[0];
    //double k = (dst[0].transpose() * src[1]).norm() / (dst[0].norm() * dst[0].norm());
    double k = (dst[0].transpose() * src[1]).norm() / (dst[0].transpose() * dst[0]);
    dst[1] = src[1] - k * dst[0];
    //double k1 = (dst[0].transpose() * src[2]).norm() / (dst[0].norm() * dst[0].norm());
    //double k2 = (dst[1].transpose() * src[2]).norm() / (dst[1].norm() * dst[1].norm());
    double k1 = (dst[0].transpose() * src[2]).norm() / (dst[0].transpose() * dst[0]);
    double k2 = (dst[1].transpose() * src[2]).norm() / (dst[1].transpose() * dst[1]);
    dst[2] = src[2] - k1 * dst[0] - k2 * dst[1];
    
    for (int i = 0; i < 3; i++) {
        dst[i] = dst[i].normalized();
    }

    return dst;
}

controlRobotInfo ConvertMask2CameraPose(const cameraImgInfo& cami, const bboxImgInfo& bbi, int bboxIndex, int con2whichcam) {
    Eigen::Vector3d camPosition = cami._CameraPosition;
    // ���ĵ�
    int goalPixelX = bbi._bboxRect[bboxIndex].x + bbi._bboxRect[bboxIndex].width / 2;
    int goalPixelY = bbi._bboxRect[bboxIndex].y + bbi._bboxRect[bboxIndex].height / 2;
    //Eigen::Vector3d cameraGoal = ConvertPixel2Camera(goalPixelX, goalPixelY, depthMean.at<double>(0, 0) / 2, 1);
    Eigen::Vector3d cameraGoal = ConvertPixel2Camera(goalPixelX, goalPixelY, 1, 1);
    cv::Mat pBase = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal[0], cameraGoal[1], cameraGoal[2], 1);
    Eigen::Vector3d pBaseEig{ pBase.at<double>(0,0) ,pBase.at<double>(1,0),pBase.at<double>(2,0) };

    // ���ϵ�
    int goalPixelX0 = bbi._bboxRect[bboxIndex].x;
    int goalPixelY0 = bbi._bboxRect[bboxIndex].y;
    Eigen::Vector3d cameraGoal0 = ConvertPixel2Camera(goalPixelX0, goalPixelY0, 1, 1);
    cv::Mat pBase0 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal0[0], cameraGoal0[1], cameraGoal0[2], 1);
    Eigen::Vector3d pBaseEig0{ pBase0.at<double>(0,0) ,pBase0.at<double>(1,0),pBase0.at<double>(2,0) };

    // ���ϵ�
    int goalPixelX1 = bbi._bboxRect[bboxIndex].x + bbi._bboxRect[bboxIndex].width;
    int goalPixelY1 = bbi._bboxRect[bboxIndex].y;
    Eigen::Vector3d cameraGoal1 = ConvertPixel2Camera(goalPixelX1, goalPixelY1, 1, 1);
    cv::Mat pBase1 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal1[0], cameraGoal1[1], cameraGoal1[2], 1);
    Eigen::Vector3d pBaseEig1{ pBase1.at<double>(0,0) ,pBase1.at<double>(1,0),pBase1.at<double>(2,0) };

    // ���µ�
    int goalPixelX2 = bbi._bboxRect[bboxIndex].x;
    int goalPixelY2 = bbi._bboxRect[bboxIndex].y + bbi._bboxRect[bboxIndex].height;
    Eigen::Vector3d cameraGoal2 = ConvertPixel2Camera(goalPixelX2, goalPixelY2, 1, 1);
    cv::Mat pBase2 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal2[0], cameraGoal2[1], cameraGoal2[2], 1);
    Eigen::Vector3d pBaseEig2{ pBase2.at<double>(0,0) ,pBase2.at<double>(1,0),pBase2.at<double>(2,0) };

    //// ���µ�
    //int goalPixelX3 = bbi._bboxRect[bboxIndex].x + bbi._bboxRect[bboxIndex].width;
    //int goalPixelY3 = bbi._bboxRect[bboxIndex].y + bbi._bboxRect[bboxIndex].height;
    //Eigen::Vector3d cameraGoal3 = ConvertPixel2Camera(goalPixelX3, goalPixelY3, 1, 1);
    //cv::Mat pBase3 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal3[0], cameraGoal3[1], cameraGoal3[2], 1);
    //Eigen::Vector3d pBaseEig3{ pBase3.at<double>(0,0) ,pBase3.at<double>(1,0),pBase3.at<double>(2,0) };
    //cout << "camera position:" << cameraGoal0.transpose() << "  " << cameraGoal1.transpose() << "  " << cameraGoal2.transpose() << endl;

    Eigen::Vector3d positionCam{ cami._CameraPosition };
    vector<Eigen::Vector3d> vDrawLinePoint; // ���������
    vector<Eigen::Vector2i> vDrawLineIndex; // ֱ�ߵ��������ж�Ӧ������
    int drawLenght = 1000.;
    // �߽��4������
    vDrawLinePoint.clear();
    vDrawLineIndex.clear();
    vDrawLinePoint.emplace_back(positionCam); // 0
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig - positionCam).normalized()); // 1
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig0 - positionCam).normalized()); // 2 ���ϵ�
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig1 - positionCam).normalized()); // 3 ���ϵ�
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig2 - positionCam).normalized()); // 4 ���µ�
    //vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig3 - positionCam).normalized()); // 5
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 5));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(3, 5));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(4, 5));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    //open3d::visualization::DrawGeometries({ pcl_ptr, lineSetTemp_cloud }, "goal");

    controlRobotInfo cmdInfo;
    // position
    cmdInfo._campositionEigen = cami._CameraPosition;
    cmdInfo._campositionMat = (cv::Mat_<double>(4, 1) << cami._CameraPosition[0], cami._CameraPosition[1], cami._CameraPosition[2], 1);
    Eigen::Vector3d cameraZ = (pBaseEig - positionCam).normalized();
    Eigen::Vector3d cameraX = ((pBaseEig1 - positionCam).normalized() - (pBaseEig0 - positionCam).normalized()).normalized();
    Eigen::Vector3d cameraY = ((pBaseEig2 - positionCam).normalized() - (pBaseEig0 - positionCam).normalized()).normalized();
    //vector<Eigen::Vector3d> vPosture(3);
    //vPosture[0] = cameraZ; vPosture[1] = cameraY; vPosture[0] = cameraX;
    //vector<Eigen::Vector3d> vPosetureSchdimit = SchdimtOrthogonalityForPoseture(vPosture);
    //cameraZ = vPosetureSchdimit[0];
    //cameraY = vPosetureSchdimit[1];
    //cameraX = vPosetureSchdimit[2];

    // posture
    cv::Mat cameraPosture(3, 3, CV_64F, cv::Scalar(0));
    cameraPosture.at<double>(0, 0) = cameraX[0]; cameraPosture.at<double>(1, 0) = cameraX[1];  cameraPosture.at<double>(2, 0) = cameraX[2];
    cameraPosture.at<double>(0, 1) = cameraY[0]; cameraPosture.at<double>(1, 1) = cameraY[1];  cameraPosture.at<double>(2, 1) = cameraY[2];
    cameraPosture.at<double>(0, 2) = cameraZ[0]; cameraPosture.at<double>(1, 2) = cameraZ[1];  cameraPosture.at<double>(2, 2) = cameraZ[2];
    cmdInfo._camposture = cameraPosture.clone();
    cmdInfo._cameuler = RotationMatrixToEulerAngles(cameraPosture);
    // pose
    cv::Mat cameraPose(4, 4, CV_64F, cv::Scalar(0));
    cameraPosture.copyTo(cameraPose({ 0,0,3,3 }));
    cameraPose.at<double>(0, 3) = cmdInfo._campositionEigen[0];
    cameraPose.at<double>(1, 3) = cmdInfo._campositionEigen[1];
    cameraPose.at<double>(2, 3) = cmdInfo._campositionEigen[2];
    cameraPose.at<double>(3, 0) = 0.;
    cameraPose.at<double>(3, 1) = 0.;
    cameraPose.at<double>(3, 2) = 0.;
    cameraPose.at<double>(3, 3) = 1.;
    cmdInfo._campose = cameraPose.clone();
    //cout << "rx:" << cameraX.transpose() << "   ry:" << cameraY.transpose() << "    rz:" << cameraZ.transpose() << endl;
    //cout << "camera pose adjust:" << cameraPosture << endl;
    //// cmd string
    //cout << "image info:" << endl;
    //cout << "   string:" << cami._poseStr << endl;
    //cout << "   position:" << cami._CameraPosition.transpose() << endl;
    //cout << "   pose:" << cami._CameraPose << endl;
    //cout << "   G2B:" << cami._Gripper2Base << endl;

    //cout << "p_gripper:" << (cami._CameraPose * H_Camera2Gripper_HD.inv() * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1)).t() << endl;
    //cv::Mat gripper = cameraPose * H_Camera2Gripper_HD.inv();
    cv::Mat gripper = cameraPose * H_Camera2Gripper.inv();
    string cmdStr;
    ConvertMat2String(gripper, cmdStr);
    cmdInfo._grippercmdStr = cmdStr;
    ////cout << "position:" << cami._CameraPosition.transpose() << endl;
    //cout << "��ʵ���꣺" << cami._poseStr << endl;
    //cout << "����Ŀ�꣺" << cmdInfo._grippercmdStr << endl;

    //cout << "z:" << cameraZ.transpose() << "    y:" << cameraY.transpose() << "    x:" << cameraX.transpose() 
    //    << "    x*z:" << cameraZ.transpose() * cameraX
    //    << "    y*z:" << cameraZ.transpose() * cameraY 
    //    << "    x*y:" << cameraY.transpose() * cameraX << endl; 
    //cout << "�ǲ�����ת����" << isRotationMatrix(cameraPose) << endl;
    ////cout << "eluer:" << cmdInfo._euler << endl;
    ////Eigen::Matrix3d cameraPostureEigen;
    //////cameraPostureEigen << cameraX[0], cameraX[1], cameraX[2], 
    //////    cameraY[0], cameraY[1],cameraY[2], cameraZ[0], cameraZ[1], cameraZ[2];
    ////cameraPostureEigen << 
    ////    cameraX[0], cameraY[0], cameraZ[0],
    ////    cameraX[1], cameraY[1], cameraZ[1],
    ////    cameraX[2], cameraY[2], cameraZ[2];
    ////cout << "eluer:" << RotationMatrixToEulerAngles(cameraPostureEigen).transpose() << endl;

    return cmdInfo;
}



int OTSU(const vector<int>& vGray) {
    const int nGrayScale = 256;//�Ҷ�
    int nPixelCount[nGrayScale] = { 0 };//�Ҷ�ֱ��ͼ
    //ͳ��ͼƬ�и����Ҷ�ֵ�ĸ���
    for (int i = 0; i < vGray.size(); i++) {
        int val = vGray[i];
        nPixelCount[val]++;		//int nPixelCount[nGrayScale] = { 0 };//�Ҷ�ֱ��ͼ
    }
    //ͳ��ͼƬ�и����Ҷ�ֵ��ռ�ı���
    int nPixelSum = vGray.size();//������ֵ
    float fPixelPct[nGrayScale] = { 0 };//�����Ҷ�ֵռ����ı���
    for (int i = 0; i < nGrayScale; ++i) {
        fPixelPct[i] = 1.0 * nPixelCount[i] / nPixelSum;
    }
    double w0, w1;//����/Ŀ������ռ��
    double u0, u1;//Ŀ��/����ƽ���Ҷ�ֵ
    double fTempVar = 0;//��䷽��
    double fMaxVar = 0;//�����䷽��
    int fBestValue = 0;//������ֵ
    double fTemp0, fTemp1;
    for (int k = 0; k < nGrayScale; ++k) {
        w0 = w1 = u0 = u1 = fTempVar = 0;
        fTemp0 = fTemp1 = 0;
        //ǰ������������ [0-k][k+1-255]
        for (int i = 0; i < nGrayScale; ++i) {
            //�����ǰ����ֵС����ֵk�����ڱ�������֮����Ŀ��
            if (i <= k) {
                //���㱳������ռ��
                w0 += fPixelPct[i];
                //���㵱ǰ�Ҷ�ֵ�����ĸ���:�Ҷ�ֵ*�Ҷ�ֵ�����ĸ���
                fTemp0 += (i * fPixelPct[i]);
            }
            else {
                //���㱳������ռ��
                w1 += fPixelPct[i];
                fTemp1 += (i * fPixelPct[i]);
            }
        }
        //����ƽ���Ҷ�ֵ��p0/w0
        u0 = fTemp0 / w0;
        u1 = fTemp1 / w1;
        //�������ڷ���
        fTempVar = (double)(w0 * w1 * pow((u0 - u1), 2));
        if (fTempVar > fMaxVar) {
            fMaxVar = fTempVar;
            fBestValue = k;
        }
    }

    return fBestValue;
}

//void MaskIntersection(const vector<cv::Mat>& src, const vector<cv::Mat>& refImg, const vector<cv::Rect>& refRect, vector<cv::Mat>& dst) {
void MaskIntersection(const cv::Mat &src, const vector<cv::Mat>&refImg, const vector<cv::Rect>&refRect, cv::Mat & dst) {
    dst = src;
    int area = 0;
    int idx = -1;
    for (int i = 0; i < refRect.size(); i++) {
        // ����ÿ����Ĥ
        if (refRect[i].width * refRect[i].height > area) {
            area = refRect[i].width * refRect[i].height;
            idx = i;
        }
    }
    if (idx != -1) dst = refImg[idx];

    return;
}

void DealMaskRegion(vector<cv::Mat>& src) {
    static int imageCnt = 0;
    cv::imwrite("D:/temp/0506/gary/" + to_string(imageCnt) + ".jpg", src[1]);
    vector<cv::Mat> vDup{ src };
    cv::Mat gary = vDup[1];
    int totalValidNum = 0;
    vector<int> vCnt(256);
    // ͳ�ƻҶȷֲ�
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
        for (int col = 0; col < IMAGE_WIDTH; col++) {
            if ((int)gary.at<uchar>(row, col) == 0) continue;
            vCnt[(int)gary.at<uchar>(row, col)]++;
            totalValidNum++;
        }
    }
    int hisHeight = 400;
    cv::Mat hisImg(hisHeight, 512, CV_8U, cv::Scalar(0));
    // ��������ͼ
    cv::Point lastPoint{0, hisHeight - vCnt[0] > hisHeight ? hisHeight : vCnt[0]};
    for (int i = 0; i < 256; i++) {
        cv::Point currPoint{ i * 2, hisHeight - (vCnt[i] > hisHeight ? hisHeight : vCnt[i]) };
        if (i != 0) cv::line(hisImg, lastPoint, currPoint, cv::Scalar{ 128 });
        lastPoint = currPoint;
    }
    // ����
    double ratio = 0.95;
    int subTotalValidNum = 0;
    int bestThresh = 0;
    for (int i = 256 - 1; i >= 0; i--) {
        subTotalValidNum += vCnt[i];
        bestThresh = i;
        if (subTotalValidNum >= totalValidNum * (1 - ratio)) break;
    }
    cv::imwrite("D:/temp/0506/hist/" + to_string(imageCnt) + ".jpg", hisImg);

    // ��ֵ��
    cv::Mat binary = gary.clone();
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
        for (int col = 0; col < IMAGE_WIDTH; col++) {
            if ((int)gary.at<uchar>(row, col) == 0 || (int)gary.at<uchar>(row, col) > bestThresh) 
                binary.at<uchar>(row, col) = 0;
            else 
                binary.at<uchar>(row, col) = 255;
        }
    }

    //vector<int> vPixel;
    //for (int row = 0; row < IMAGE_HEIGHT; row++) {
    //    for (int col = 0; col < IMAGE_WIDTH; col++) {
    //        //if ((int)gary.at<uchar>(row, col) == 0) continue;
    //        //cout << (int)gary.at<uchar>(row, col) << "  ";
    //        vPixel.emplace_back((int)gary.at<uchar>(row, col));
    //    }
    //}
    //int bestGray = OTSU(vPixel);
    //cout << "��ֵΪ��" << bestGray << endl;

    //cv::threshold(gary, binary, bestThresh, 255., cv::THRESH_BINARY);
    //cv::imshow("binary", binary);
    //cv::waitKey(0);
    cv::imwrite("D:/temp/0506/binary/" + to_string(imageCnt) + ".jpg", binary);
    imageCnt++;
    src[0] = binary;
    src[3] = binary;
}

void ConvertPoint2VoxelPos(const Eigen::Vector3d p, Eigen::Vector3i& pos) {
    pos = Eigen::Vector3i(0, 0, 0);
    open3d::geometry::AxisAlignedBoundingBox axisBox = pcl_ptr->GetAxisAlignedBoundingBox(); // ���ư�Χ��
    vector<Eigen::Vector3d> vBoundingBox = axisBox.GetBoxPoints();
    double x = axisBox.GetExtent()[0]; // x����
    double y = axisBox.GetExtent()[1]; // y����
    double z = axisBox.GetExtent()[2]; // z����
    // ��¼ƫ��
    double disX = abs(p[0] - vBoundingBox[0][0]); // xƫ�� - col
    double disY = abs(p[1] - vBoundingBox[0][1]); // yƫ�� - row
    double disZ = abs(p[2] - vBoundingBox[0][2]); // zƫ�� - z
    int xVoxelNum = x / VOXEL_SIZE;
    int yVoxelNum = y / VOXEL_SIZE;
    int zVoxelNum = z / VOXEL_SIZE;
    // ��i�����ӦͼƬ�ϵ����꣨nowRow��nowCol��
    int voxelX = round((disX * (double)xVoxelNum) / x);
    int voxelY = round((disY * (double)yVoxelNum) / y);
    int voxelZ = round((disZ * (double)zVoxelNum) / z);
    //cout << (disX * (double)xVoxelNum) / x << "������������" << voxelX << endl;
    //int voxelX = (disX * (double)xVoxelNum) / x - 1;
    //int voxelY = (disY * (double)yVoxelNum) / y - 1;
    //int voxelZ = (disZ * (double)zVoxelNum) / z - 1;
    voxelX = voxelX < 0 ? 0 : voxelX;
    voxelY = voxelY < 0 ? 0 : voxelY;
    voxelZ = voxelZ < 0 ? 0 : voxelZ;

    //pos[0] = xVoxelNum - voxelX;
    //pos[1] = yVoxelNum - voxelY;
    pos[0] = voxelX;
    pos[1] = voxelY;
    pos[2] = voxelZ;
    if (p[2] > vBoundingBox[2][2]) {
        // ��������Ͻ�
        cout << "���λ�ô��ڵ������ޣ�" << endl;
        //return;
    }
    else {
    }

    //std::cout << "��ǰ3D������Ϊ��" << p.transpose() << endl;
    //std::cout << "xVoxelNum��" << xVoxelNum << "   yVoxelNum��" << yVoxelNum << " zVoxelNum��" << zVoxelNum << endl;
    //std::cout << "x��" << x << "   y��" << y << " z��" << z << endl;
    //std::cout << "vBoundingBox[0]��" << vBoundingBox[0].transpose() << endl;
    //std::cout << "vBoundingBox[4]��" << vBoundingBox[4].transpose() << endl;
    //std::cout << "disX��" << disX << "   disY��" << disY << " disZ��" << disZ << endl;
    //std::cout << "voxelX��" << voxelX << "   voxelY��" << voxelY << " voxelZ��" << voxelZ << endl;
    //std::cout << "pos��" << pos.transpose() << endl;
    return;
}
