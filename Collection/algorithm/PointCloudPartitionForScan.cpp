#include "CoordinateTransformation.h"
#include "GlobalVariable.h"
#include "PointCloudPartitionForScan.h"
#include "OperatePointCloud.h"

/// <summary>
/// ��ʼ��������Ϣ����õ��Ʋ���
/// </summary>
/// <param name="pcl">���룺����</param>
/// <param name="pcli">��������Ʋ����ṹ��</param>
void InitPointCloudInfo(const std::shared_ptr<open3d::geometry::PointCloud>& pcl, PointCloudInfo& pcli) {
	open3d::geometry::AxisAlignedBoundingBox axisBox = pcl->GetAxisAlignedBoundingBox(); // ���ư�Χ��
	vector<Eigen::Vector3d> axisPoint = axisBox.GetBoxPoints(); // ��ȡ��Χ�����꣨0-1������0-2����0-3���ߣ�
	double pclLength = axisBox.GetExtent()[0];
	double pclWidth = axisBox.GetExtent()[1];
	double pclHeight = axisBox.GetExtent()[2];
	pcli._refRx = (axisPoint[0] - axisPoint[1]).normalized();
	pcli._refRy = (axisPoint[0] - axisPoint[2]).normalized();
	if (pclLength > pclWidth) {
		pcli._refRx = (axisPoint[0] - axisPoint[2]).normalized();
		pcli._refRy = (axisPoint[0] - axisPoint[1]).normalized();
	}
	//cout << pcli._refRx.transpose() << "	" << pcli._refRy.transpose() << endl;
	pcli._minBound = axisBox.GetMinBound();
	pcli._maxBound = axisBox.GetMaxBound();
	pcli._extent = axisBox.GetExtent();
	//���Ʋ���ȷ����
	//    axisBox.GetExtent()����Χ�г�������
	//    inlier->GetMinBound()����Χ��8�����еĵ�0��axisPoint[0]---P0
	//    inlier->GetMaxBound()����Χ��8�����еĵ�4��axisPoint[4]---P4
	return;
}

/// <summary>
/// �㷨ʽ���жϵ�ǰ��������ָ��
/// </summary>
/// <param name="vPoint"></param>
/// <returns></returns>
bool OrientEstimate(Eigen::Vector3d& norm) {
	bool status = true;

	// ���Ԫ�ض�Ӧ�ķ���
	Eigen::Vector3d pointDup = norm.cwiseAbs(); // ȡ����ֵ
	Eigen::MatrixXd::Index maxRow, maxCol;
	int maxComponentInd = pointDup.maxCoeff(&maxRow, &maxCol); // ����ֵ���ֵ��Ӧ����
	if (norm[maxRow] > 0)
		return status;
	else
		return 1 - status;
}

/// <summary>
/// ˹���أ����������z����Ϊ��1��ʹ����x��y�ο�������֮����
/// </summary>
/// <param name="xOrientRec">���룺����x�ο�����</param>
/// <param name="yOrientRec">���룺����y�ο�����</param>
/// <param name="zOrient">���룺������᷽��</param>
/// <param name="xOrient">������������x����</param>
/// <param name="yOrient">������������y����</param>
void CalcPointCloudXYOrient(Eigen::Vector3d xOrientRec, Eigen::Vector3d yOrientRec, Eigen::Vector3d zOrient,
	Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient) {
	// ��̬����ǰ���ư�Χ�У��԰�Χ�е�x��y����������Ϊ����ĺ�ѡ������ֱ��ѡy���򣨿���ѡ��n�����С�ķ���Ϊ���һ������
	// ��x���򣨳����豸����˹����������
	xOrient = (xOrientRec - xOrientRec.transpose() * zOrient * zOrient).normalized();
	bool ret = OrientEstimate(xOrient); // ʹx������ָ���Լ�ͬһ�ࣨ���������������ķ��ţ�
	////cout << "��" << i << "�����Ʒ���ֵ��" << ret << endl;
	//if (!ret) {
	//	xOrient = -1 * xOrient; // ʹ��������ָ���Լ��ⲿ
	//}
	// ��y����˹����������
	yOrient = (yOrientRec - yOrientRec.transpose() * xOrient * xOrient - yOrientRec.transpose() * zOrient * zOrient).normalized();

	return;
}

/// <summary>
/// �����豸�ߴ罫���Ʒֿ�
/// </summary>
/// <param name="pcl">���룺�������</param>
/// <returns>�ֿ����</returns>
vector<std::shared_ptr<open3d::geometry::PointCloud>> PointCloudPartition(const std::shared_ptr<open3d::geometry::PointCloud>& pcl) {
	// �ֿ飺�����豸�ߴ磬�����Ʒָ��С�顣
	// ÿ��С��ͨ��plain segment����ϸ�ֳ��������
	vector<std::shared_ptr<open3d::geometry::PointCloud>> vSubPointCloud;

	open3d::geometry::AxisAlignedBoundingBox axisBox = pcl->GetAxisAlignedBoundingBox(); // ���ư�Χ��
	vector<Eigen::Vector3d> axisPoint = axisBox.GetBoxPoints(); // ��ȡ��Χ�����꣨0-1������0-2����0-3���ߣ�
	// pclLength��һ������pclWidth��
	double pclLength = axisBox.GetExtent()[0];
	double pclWidth = axisBox.GetExtent()[1];
	double pclHeight = axisBox.GetExtent()[2];
	//// MinBound����Ӧ��С����㣻MaxBound���������㣻GetExtent����Ӧ�����
	//cout << "MinBound��" << axisBox.GetMinBound().transpose() << endl;
	//cout << "MaxBound��" << axisBox.GetMaxBound().transpose() << endl;
	//cout << "GetExtent:" << axisBox.GetExtent().transpose() << endl;
	//cout << (axisPoint[0] - axisPoint[1]).norm() << endl;
	//cout << (axisPoint[0] - axisPoint[2]).norm() << endl;
	//cout << (axisPoint[0] - axisPoint[3]).norm() << endl;

	// ���ƶ�/�豸��
	//int lengthScanTimes = ceil(min(pclLength, pclWidth) / DEVICE_LENGTH); // ���ƶ�/�豸��
	//int widthScanTimes = ceil(max(pclLength, pclWidth) / DEVICE_WIDTH); // ���Ƴ�/�豸��
	int lengthScanTimes = ceil(min(pclLength, pclWidth) / DEVICE_LENGTH); // ���ƶ�/�豸��
	int widthScanTimes = ceil(max(pclLength, pclWidth) / DEVICE_WIDTH); // ���Ƴ�/�豸��
	//cout << "max(pclLength, pclWidth) / DEVICE_WIDTH:" << min(pclLength, pclWidth) / DEVICE_LENGTH << endl;
	//cout << "max(pclLength, pclWidth) / DEVICE_WIDTH:" << max(pclLength, pclWidth) / DEVICE_WIDTH << endl;
	cout << "lengthScanTimes:" << lengthScanTimes << "	widthScanTimes:" << widthScanTimes << endl;
	vector<std::shared_ptr<const open3d::geometry::Geometry>> vTotal; // ������ʾ
	for (int col = 0; col < lengthScanTimes; col++) {
		for (int row = 0; row < widthScanTimes; row++) {
			// �����Χ��minBound��maxBound
			double minBoundX = col * pclLength / lengthScanTimes;
			double minBoundY = row * pclWidth / widthScanTimes;
			double maxBoundX = (col + 1) * pclLength / lengthScanTimes;
			double maxBoundY = (row + 1) * pclWidth / widthScanTimes;
			if (pclLength > pclWidth) {
				// ����Ե�
				////cout << "length > width" << endl;
				minBoundX = row * pclLength / widthScanTimes;
				minBoundY = col * pclWidth / lengthScanTimes;
				maxBoundX = (row + 1) * pclLength / widthScanTimes;
				maxBoundY = (col + 1) * pclWidth / lengthScanTimes;
			}
			Eigen::Vector3d currMinBound = axisBox.GetMinBound() + Eigen::Vector3d(minBoundX, minBoundY, 0);
			Eigen::Vector3d currMaxBound = axisBox.GetMinBound() + Eigen::Vector3d(maxBoundX, maxBoundY, axisBox.GetMaxBound()[2] - axisBox.GetMinBound()[2]);
			open3d::geometry::AxisAlignedBoundingBox cropBox = open3d::geometry::AxisAlignedBoundingBox(currMinBound, currMaxBound);
			// ��bbox�ָ��������
			std::shared_ptr<open3d::geometry::PointCloud> croped_cloud = pcl->Crop(cropBox);
			shared_ptr<open3d::geometry::PointCloud> temp_pointcloud = croped_cloud;
			//open3d::visualization::DrawGeometries({ croped_cloud }, "croped_cloud");
			//vector<std::shared_ptr<open3d::geometry::Geometry>> vPlainSegment;
			vector<std::shared_ptr<open3d::geometry::PointCloud>> vPlainSegment;
			// ƽ��ָ����ֳ�3��
			for (int itr = 0; itr < 3; itr++) {
				// 1��ƽ��ָ�
				tuple< Eigen::Vector4d, vector<size_t>> retPlane = temp_pointcloud->SegmentPlane(12, 5, 100); // �����ƽ��ģ�͵�������
				Eigen::Vector4d planeFormula = std::get<0>(retPlane); // ƽ�淽�̣�ax + by + cz + d = 0 
				vector<size_t> planeIndex = std::get<1>(retPlane); // �ڵ�����
				//std::cout << "planeFormula��" << planeFormula.transpose() << endl;
				// 2��ƽ��
				shared_ptr<open3d::geometry::PointCloud> in_pointcloud = temp_pointcloud->SelectByIndex(planeIndex);
				open3d::geometry::AxisAlignedBoundingBox localPlaneSegment = in_pointcloud->GetAxisAlignedBoundingBox(); // ��ʾƽ��ָ��İ�Χ��
				shared_ptr<open3d::geometry::PointCloud> outlier_pointcloud = temp_pointcloud->SelectByIndex(planeIndex, true);
				//      ȥ����Ⱥ��������Ⱥ���޳�������ָ����һ������Χѡ�����ɸ��㣬�������Ǿ����ͳ�Ʋ�����
				//                  ���ĳ����ƫ��ƽ��ֵ����stdio_ratio���ķ�������Ϊ����Ⱥ�㣬������ɾ����
				shared_ptr<open3d::geometry::PointCloud> out_withoutOutlier = std::get<0>(outlier_pointcloud->RemoveStatisticalOutliers(30, 1)); //0��������ֵ����
				//open3d::visualization::DrawGeometries({ in_pointcloud }, "Segment Plane");
				vPlainSegment.emplace_back(in_pointcloud); // ƽ��ָ���
				// 3��1/nԭ����>ʣ���->ֹͣ
				if (croped_cloud->points_.size() > 6 * out_withoutOutlier->points_.size()) { // ȡn=5
					break;
				}
				// <= ���µ���
				temp_pointcloud = out_withoutOutlier;
			}
			std::shared_ptr<open3d::geometry::PointCloud> currPointCloud = std::make_shared<open3d::geometry::PointCloud>();
			for (int i = 0; i < vPlainSegment.size(); i++) {
				*currPointCloud += *vPlainSegment[i];
			}
			vTotal.emplace_back(currPointCloud);
			vSubPointCloud.emplace_back(currPointCloud); // ���浱ǰƽ��
			//open3d::visualization::DrawGeometries({ currPointCloud }, "Segment Plane");
		}
	}
	//open3d::visualization::DrawGeometries({ vTotal }, "total");

	return vSubPointCloud;
}

/// <summary>
/// ���ڵ�ǰ���������λ�á���̬������������Ϣ
/// </summary>
/// <param name="tsi"></param>
void IntergateScanInfo(ThermalScanInfo& tsi) {
	cv::Mat cmaR = tsi._cameraPosetrue;
	cv::Mat camT = (cv::Mat_<double>(3, 1) << tsi._cameraPosition[0], tsi._cameraPosition[1], tsi._cameraPosition[2]);
	// ��R��t��Ϊ�����ʽ
	cv::Mat hThermal2Base = R_T2RT(cmaR, camT);
	// ��е��ĩ����̬��ȡ�����۱궨����B * ���λ��(R,t)
	cv::Mat gripperRT = hThermal2Base * H_Thermal2Gripper.inv(); // (R,t) * H_Thermal2Gripper.inv()
	tsi._gripperPoseMat = gripperRT.clone();
	ConvertMat2String(tsi._gripperPoseMat, tsi._cmdStr); // gripper cmd
	tsi._gripperPosetrue = gripperRT({ 0,0,3,3 });
	Eigen::Vector3d eulerAngles = RotationMatrixToEulerAngles(tsi._gripperPosetrue); // ��ת����תŷ����
	//std::cout << "ŷ����Ϊ��" << eulerAngles.transpose() << endl;

	// ���浱ǰ����
	tsi._cameraRx = Eigen::Vector3d { cmaR.at<double>(0, 0), cmaR.at<double>(1, 0), cmaR.at<double>(2, 0) }; // Rx
	tsi._cameraRy = Eigen::Vector3d{ cmaR.at<double>(0, 1), cmaR.at<double>(1, 1), cmaR.at<double>(2, 1) }; ; // Ry
	tsi._cameraRz = Eigen::Vector3d{ cmaR.at<double>(0, 2), cmaR.at<double>(1, 2), cmaR.at<double>(2, 2) }; ; // Rz
	Eigen::Vector6d nowGripperPose; // ��Ż�е��ĩ��6Dλ��
	nowGripperPose << gripperRT.at<double>(0, 3), gripperRT.at<double>(1, 3), gripperRT.at<double>(2, 3),
		eulerAngles[0], eulerAngles[1], eulerAngles[2];
	tsi._gripperPoseVec = nowGripperPose;
	tsi._gripperPosition= Eigen::Vector3d{ nowGripperPose[0],nowGripperPose[1],nowGripperPose[2] };
	vector<double> vTemp(6);
	for (int i = 0; i < 6; i++) {
		vTemp[i] = tsi._gripperPoseVec[i];
	}
	string gripperCmd;
	Convert6D2String(vTemp, gripperCmd);
	tsi._cmdStr = gripperCmd;

	return;
}

/// <summary>
/// �Էֿ����ƽ���PCA��ȡ����
/// </summary>
/// <param name="pcli"></param>
/// <param name="vCropedCloudForPCA"></param>
/// <param name="_scanInfo"></param>
/// <returns></returns>
vector<std::shared_ptr<open3d::geometry::LineSet>> PointCloudPartitionPCA(
	const PointCloudInfo& pcli,
	vector<std::shared_ptr<open3d::geometry::PointCloud>>& vCropedCloudForPCA,
	vector<ThermalScanInfo>& _scanInfo) {
	vector<std::shared_ptr<open3d::geometry::LineSet>> vNormOrient;

	// �ֿ����PCA����
	for (int i = 0; i < vCropedCloudForPCA.size(); i++) {
		//auto currPcl = vCropedCloudForPCA[i];
		auto center_point = vCropedCloudForPCA[i]->GetCenter();
		//����KDTree ����K��������           
		geometry::KDTreeFlann pointTree(*vCropedCloudForPCA[i]);
		//��������ʹ��KNN��������  Ѱ��ԭ�����е������
		std::vector<int> indices;
		std::vector<double> distance;
		pointTree.SearchKNN(center_point, 1, indices, distance);
		indices.clear();
		distance.clear();
		//��������ʹ��KNN��������  Ѱ�Ҹ������kNum����������
		int kNum = vCropedCloudForPCA[i]->points_.size();
		//int kNum = vCropedCloudForPCA[i]->points_.size() / 2;
		//int kNum = 1000;
		pointTree.SearchKNN(center_point, kNum, indices, distance);
		//���K���ڵ㵽������
		Eigen::MatrixXd knearestPointData(kNum, 3);
		auto currPcl = std::make_shared<open3d::geometry::PointCloud>();
		for (int pdNum = 0; pdNum < kNum; pdNum++) {
			currPcl->points_.emplace_back(vCropedCloudForPCA[i]->points_[indices[pdNum]]);
			currPcl->colors_.emplace_back(vCropedCloudForPCA[i]->colors_[indices[pdNum]]);
		}

		// ��ǰ���ƶ�Ӧ���ǵ�i�е�j��ɨ��ʱ�ĵ���
		// ��̬R
		Eigen::Vector3d minEigVector(0, 0, 0), nowCenter(0, 0, 0);
		CalcPointCloudNormPCA(currPcl, minEigVector, nowCenter); // �ֿ�PCA�����z����
		//CalcPointCloudNormPCA(vCropedCloudForPCA[i], minEigVector, nowCenter); // �ֿ�PCA�����z����
		//std::cout << "��ǰ��������λ�ã�" << nowCenter.transpose() << endl;
		// �жϵ�ǰ���߷�����ָ���Լ��ⲿ�����ݷ���x�����������ķ��ţ�
		bool ret = OrientEstimate(minEigVector);
		//cout << "��" << i << "�����Ʒ���ֵ��" << ret << endl;
		if (!ret) {
			minEigVector = -1 * minEigVector; // ʹ��������ָ���Լ��ⲿ
		}
		minEigVector = -1 * minEigVector; // z�ᷴ��
		Eigen::Vector3d xCamOrient, yCamOrient;
		CalcPointCloudXYOrient(pcli._refRx, pcli._refRy, minEigVector, xCamOrient, yCamOrient); // ˹���������������x��y����            
		cv::Mat camR = (cv::Mat_<double>(3, 3) << xCamOrient[0], yCamOrient[0], minEigVector[0], // xyz
			xCamOrient[1], yCamOrient[1], minEigVector[1],
			xCamOrient[2], yCamOrient[2], minEigVector[2]
			);
		ThermalScanInfo tsi;
		tsi._cameraPosetrue = camR.clone();
		// λ��t
		Eigen::Vector3d camPosition(nowCenter - minEigVector * (PLAIN2CAMERA + REMAIN_PLAIN2CAMERA)); // ȷ�������ǹ��� = ������ģ����߷������� * �������ǳߴ� + Ԥ�����ȣ�
		tsi._cameraPosition = camPosition; // ����������3Dλ��
		//std::cout << "��ǰ����������꣺" << camPosition.transpose() << endl;
		IntergateScanInfo(tsi);
		_scanInfo.emplace_back(tsi);
		// ��ȡ�豸3D����
		std::shared_ptr<open3d::geometry::LineSet> device_pointcloud = DrawDeviceLinePointCloud(camPosition, xCamOrient, yCamOrient, minEigVector);
		// ��ʾ��ǰ���Ƶķ���
		vector<Eigen::Vector3d> vLinePoint;
		vLinePoint.emplace_back(camPosition); // ��Ҫ����ֱ�����ڵ�
		vLinePoint.emplace_back(camPosition + minEigVector * 50);
		vLinePoint.emplace_back(camPosition + xCamOrient * 30);
		vLinePoint.emplace_back(camPosition + yCamOrient * 20);
		vector<Eigen::Vector2i> vLineIndex; // ֱ�ߵ��������ж�Ӧ������
		vLineIndex.emplace_back(Eigen::Vector2i(0, 1));
		vLineIndex.emplace_back(Eigen::Vector2i(0, 2));
		vLineIndex.emplace_back(Eigen::Vector2i(0, 3));
		std::shared_ptr<open3d::geometry::LineSet> lineSet_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vLinePoint, vLineIndex));
		vNormOrient.emplace_back(lineSet_cloud);
		//open3d::visualization::DrawGeometries({ pcl_ptr, device_pointcloud, vCropedCloudForPCA[i] , lineSet_cloud, /*nowBox_cloud*/ }, "CloudWithDir");
	}

	return vNormOrient;
}

void CalcPointCloudNormPCA(std::shared_ptr<open3d::geometry::PointCloud>& pointCloud, Eigen::Vector3d& norm, Eigen::Vector3d& center) {
	auto sampled_cloud = pointCloud->VoxelDownSample(10);//�Ե��ƽ����²���
	auto inlier = std::get<0>(sampled_cloud->RemoveStatisticalOutliers(30, 1.0)); //0��������ֵ����
	vector<Eigen::Vector3d> vNowPoints = inlier->points_; // vCropedCloud[i]
	Eigen::Matrix3Xd inputX(3, vNowPoints.size()); // �������� 
	Eigen::Matrix3d convX; // Э�������
	for (int i = 0; i < vNowPoints.size(); i++) {
		// ��ȡ����
		inputX(0, i) = vNowPoints[i][0];
		inputX(1, i) = vNowPoints[i][1];
		inputX(2, i) = vNowPoints[i][2];
	}
	//cout << "�У�" << inputX.rows() << "�У�" << inputX.cols() << endl;
	// PCA��ȥ��ֵ����Э������������ֽ⡢ѡ����С����ֵ�����߷���
	// 01��ȡ��ֵx~=1/n*sum(xi)
	Eigen::MatrixXd meanVal = inputX.rowwise().mean(); //����ÿһά�Ⱦ�ֵrowwise().mean��ÿһ�����ֵ
	Eigen::Vector3d meanColVal = meanVal;
	inputX.colwise() -= meanColVal; //������ֵ��Ϊ0
	//cout << "meanVal.cols()��" << meanVal.cols() << "meanVal.rows()��" << meanVal.rows() << endl;
	// 02Э�������convX = X*X.T
	convX = inputX * inputX.adjoint(); // ��Э�������
	convX = convX / (inputX.rows() - 1);
	//// SVD�ֽ�
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(convX, Eigen::ComputeThinU | Eigen::ComputeThinV); 
	//Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
	//Eigen::Matrix3d S = U.inverse() * convX * V.transpose().inverse();
	//cout << U << endl;
	//cout << S << endl;
	//cout << V << endl;
	//Eigen::Matrix3d eigVector = V.transpose(); // ÿһ���Ƕ�Ӧ����������
	//Eigen::Vector3d minEigVector = eigVector.col(0);
	// 03�������ֽ�
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(convX); // �����ֽ�
	Eigen::Matrix3d eigVector = eig.eigenvectors(); // ÿһ���Ƕ�Ӧ����������
	Eigen::Vector3d eigValue = eig.eigenvalues();
	//cout << "����������" << eigVector << endl;
	//cout << "����ֵ��" << eigValue.transpose() << endl;
	// 04�����ߣ�����ֵ��С��Ӧ����������
	Eigen::Vector3d minEigVector = eigVector.col(0);
	//Eigen::Vector3d secEigVector = eigVector.col(1);
	//Eigen::Vector3d maxEigVector = eigVector.col(2);
	//cout << "��С����ֵ��Ӧ��������άΪ��" << minEigVector.transpose() << endl;

	norm = minEigVector;
	center = pointCloud->GetCenter();// ��ǰ���Ƶ�����
}

/// <summary>
/// ������ת��ԭʼλ�õĽǶȣ���z����ʱ��Ϊ��
/// </summary>
/// <param name="rotateTheta"></param>
/// <returns></returns>
cv::Mat RotateRzMat(double rotateTheta) {
	return (cv::Mat_<double>(4, 4) <<
		cos(rotateTheta), -sin(rotateTheta), 0, 0,
		sin(rotateTheta), cos(rotateTheta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);
}
cv::Mat RotateRyMat(double rotateTheta) {
	return (cv::Mat_<double>(4, 4) <<
		cos(rotateTheta), 0., sin(rotateTheta), 0,
		0, 1, 0, 0,
		-sin(rotateTheta), 0., cos(rotateTheta), 0,
		0, 0, 0, 1
		);
}
cv::Mat RotateRxMat(double rotateTheta) {
	return (cv::Mat_<double>(4, 4) <<
		1., 0., 0., 0.,
		0., cos(rotateTheta), -sin(rotateTheta), 0,
		0., sin(rotateTheta), cos(rotateTheta), 0,
		0, 0, 0, 1
		);
}