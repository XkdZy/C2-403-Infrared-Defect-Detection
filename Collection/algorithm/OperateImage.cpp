#include "OperateImage.h"
#include "GlobalVariable.h"
#include "AstraCameraD2C.h"
#include "CoordinateTransformation.h"
#include "RobotAlgorithm.h"
#include <Python.h>
#include "IOFile.h"
#include "MindVision.h"

const wchar_t* PythonEnvPath = L"F:/QMDownload/Anaconda/Anaconda3/envs/Env_VILD";

bool PixelInBBox(const cv::Rect& rec, const cv::Point& pixel) {
	if (rec.x > pixel.x) {
		return false;
	}
	if (rec.y > pixel.y) {
		return false;
	}
	if ((rec.x + rec.width) < pixel.x) {
		return false;
	}
	if ((rec.y + rec.height) < pixel.y) {
		return false;
	}
	return true;
}

template <typename T>
vector<int> sort_indexes(const vector<T>& v)
{
	// ��ʼ������
	vector<int> idx(v.size());
	for (int i = 0; i != idx.size(); ++i) idx[i] = i;

	// ����vector��ֵ����������
	//sort(idx.begin(), idx.end()); // Ĭ����������
	sort(idx.begin(), idx.end(),
		[&v](int i1, int i2) -> bool { return v[i1] > v[i2]; }); //C++11 Lambda���ʽ
	/*
	* [caoture] (params) opt -> ret {body;};
	* ���У�capture�ǲ����б�
	* params�ǲ�����
	* opt�Ǻ���ѡ�
	* ret�Ƿ���ֵ���ͣ�
	* body�Ǻ�����
	*/
	return idx;
}

bool refreshImgFlag = false;
// ��ʱ���ж�
void TimerInterpute(void) {
	refreshImgFlag = true;
}

cv::Point pRes;
bool exitChildThread = false;
void capFunc(int event, int x, int y, int flags, void* para) {
	if (cv::EVENT_LBUTTONDOWN == event) {
		pRes.x = x;
		pRes.y = y;
		pRes.x = pRes.x > IMAGE_WIDTH ? pRes.x - IMAGE_WIDTH : pRes.x;
		exitChildThread = true;
		cout << "�����λ���ڣ�" << pRes.y << " " << pRes.x << endl;
	}
}

cv::Point CollectImgAndInteraction(cameraImgInfo& cii) {
	static int timer10sCnt = 0; // ������ʱ��10s��10s����һ�ε��Ƹ���ͼ
	string gripper2BaseRealTimePose;
	//string colorRealTimeSavePath = "./imgs/tmp.jpg";
	//string depthRealTimeSavePath = "./imgs/tmp.png";
	string colorRealTimeSavePath = "./imgs/rgbd_frame/rgb/current_frame.jpg"; // ����ʵʱͼƬ��·��
	string depthRealTimeSavePath = "./imgs/rgbd_frame/depth/current_frame.png";
	string saveRealTimePosFilePath = "./imgs/curr.txt";
	string opencvRealTimeDisplayWindowName = "real time";

	exitChildThread = false;
	pRes.x = 0;
	pRes.y = 0;

	while (!exitChildThread) {
		if (refreshImgFlag) {
			timer10sCnt++;
			refreshImgFlag = false;
			//cout << "��ˢ��ͼƬ��" << endl;
			cv::Mat colorImageDup(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
			cv::Mat depthImageDup(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);
			cv::Mat depthImageRealTime, colorImageRealTime;
			// ����ʵʱRGBDͼƬ����ʾ������ͼ
			if (astraCameraD2C->GetStreamData(colorImageDup, depthImageDup) == CAMERA_STATUS_SUCCESS) {
				gripper2BaseRealTimePose = robotArmHandle->ReadRobotArmPosString(); // ��ȡ��ǰ��е��6Dλ��
				//cout << "current pose:" << gripper2BaseRealTimePose << endl;
				// ��ʵʱ����д�뵽�ļ���
				IOFile iof(saveRealTimePosFilePath);
				// ����ļ����ݣ�д������
				iof.WriteString2File(gripper2BaseRealTimePose, 1); // д�뵱ǰ��е��ĩ������
				flip(colorImageDup, colorImageRealTime, 1); // ������ͼƬ����
				flip(depthImageDup, depthImageRealTime, 1);
				//cv::imshow("a", colorImageRealTime);
				//cv::waitKey(0);
				imwrite(colorRealTimeSavePath, colorImageRealTime); // ���浱ǰRGB�����
				imwrite(depthRealTimeSavePath, depthImageRealTime); // PNG16
				UpdateRealTimePosInfo(colorImageRealTime, depthImageRealTime, gripper2BaseRealTimePose, cii);
				cv::Mat upDateDisplayImg;
				//cout << "vVerticalImageRealTime[2]:" << vVerticalImageRealTime[2].cols << endl;
				cv::Mat verticalImageRealTimeDisplay = colorImageRealTime; // ȡ��ֱΪ��ǰ�ӽ�
				CombinateTwoImg(verticalImageRealTimeDisplay, colorImageRealTime, upDateDisplayImg);
				cv::imshow(opencvRealTimeDisplayWindowName, upDateDisplayImg); // ������ʾ
				cv::waitKey(5);
				// �����������Ϣ
				cv::setMouseCallback(opencvRealTimeDisplayWindowName, capFunc, reinterpret_cast<void*>(&upDateDisplayImg));
			}
		}
		//if ((timer10sCnt >= (1000. / TIMER_TIMING_TIME)) && upDateVerticalVaild) {
		//	timer10sCnt = 0;
		//	// 10s���ˣ���ԭһ��3D������һ�ε��Ƹ���ͼ
		//	// ���߳����ڲɼ�ʵʱRGBDͼƬ����ʱ���ã�
		//	HANDLE handle = (HANDLE)_beginthread(ReconstructRealTime, 0, NULL);
		//}
	}

	cv::destroyWindow(opencvRealTimeDisplayWindowName);

	return pRes; 
}

void CombinateTwoImg(const cv::Mat& mVertical, const cv::Mat& mFront, cv::Mat& dst) {
	int verticalViewRows = mVertical.rows;
	int verticalViewCols = mVertical.cols;
	int currentViewRows = mFront.rows;
	int currentViewCols = mFront.cols;
	dst = cv::Mat::zeros(max(verticalViewRows, currentViewRows), currentViewCols + verticalViewCols, CV_8UC3);
	//cout << "verticalView:" << verticalViewRows << "  " << verticalViewCols << endl;
	//cout << "currentView:" << currentViewRows << "  " << currentViewCols << endl;
	mVertical.copyTo(dst(cv::Rect(0, 0, verticalViewCols, verticalViewRows)));
	mFront.copyTo(dst(cv::Rect(verticalViewCols, 0, currentViewCols, currentViewRows)));

	return;
}

// ����ʵʱͼƬ��λ����Ϣ
void UpdateRealTimePosInfo(const cv::Mat& rgb, const cv::Mat& depth, string imgPose, cameraImgInfo& cii, const Eigen::Vector3d agvPose, const Eigen::Vector3d agvIMU) {
	// �ַ���תMat
	cv::Mat str2MatData;
	AnalysisString26D(imgPose, str2MatData);
	// _Gripper2Base
	cv::Mat gripper2base = attitudeVectorToMatrix(str2MatData.row(0), false, "xyz");
	cii._Gripper2Base = gripper2base.clone();
	//cout << "image pose:" << imgPose << endl;
	//cout << "cii._Gripper2Base read:" << cii._Gripper2Base << endl;
	// _CameraPose
	cv::Mat cameraPose = gripper2base * H_Camera2Gripper;
	cii._CameraPose = cameraPose.clone();
	// _CameraPosition
	cv::Mat cameraPosition = cameraPose * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	cii._CameraPosition = Eigen::Vector3d(cameraPosition.at<double>(0, 0), cameraPosition.at<double>(1, 0), cameraPosition.at<double>(2, 0));
	// _CameraPostureEig��_CameraPostureEig
	cv::Mat cameraPosture = cameraPose(cv::Range(0, 3), cv::Range(0, 3)); // ǰ3*3
	cii._CameraPostureEig = RotationMatrixToEulerAngles(cameraPosture);
	cii._CameraPostureMat = cameraPosture.clone();
	// _RgbImg
	cii._RgbImg = rgb.clone();
	// _DepthImg
	cii._DepthImg = depth.clone();
	// string
	cii._poseStr = imgPose;
	// agv info
	cii._agvPose = agvPose;
	// agv imu
	cii._agvIMU = agvIMU;

	return;
}


bool CallPythonScriptInference(bool vildOrbts) {
	//ָ��python.exeλ��  python�Ļ�����������Ǩ�ƻ���ʱ����include libs���ڹ���Ŀ¼�£�ָ������Ŀ¼�Ϳ���
	Py_SetPythonHome(PythonEnvPath);
	Py_Initialize();
	// ��ֹimport tensorflow����
	wchar_t* argv[] = { LPWSTR(L" ") }; // ��ȷ����;
	PySys_SetArgv(1, argv);
	// ����VildInference.pyģ�� 
	PyObject* pModule = PyImport_ImportModule("VildInference");//������Ҫ���õ��ļ���	   
	if (!pModule)
	{
		cout << "Can't find file (VildInference)" << endl;
		return false;
	}
	else
	{
		PyObject* mainPo = PyObject_GetAttrString(pModule, "VildInference");
		if (!mainPo || !PyCallable_Check(mainPo))
		{
			cout << "Can't find funftion (VildInference)" << endl;
			return false;
		}
		else
		{
			cout << "Get function (VildInference) succeed." << endl;
			//PyObject* pArg = Py_BuildValue("(s)", dataPath.c_str());  //һ���ַ�������
			PyObject* pArg = NULL;  //һ���ַ�������
			PyEval_CallObject(mainPo, pArg);
		}
	}

	Py_DECREF(pModule);

	Py_Finalize();

	return true;
}

vector<int> CompareImageArea(const vector<cv::Mat>& vImg) {
	vector<int> vIndex;
	vector<int> vArea;

	cv::Mat depthImg(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8U, cv::Scalar(1));
	cv::Mat mask;
	// ����ÿ��ͼƬ��ɫ�������
	for (int i = 0; i < vImg.size(); i++) {
		int validCnt = CountImageWhiteArea(vImg[i], depthImg, 0, mask);
		vArea.push_back(validCnt);
	}
	// ��ÿ��������н�������
	vIndex = sort_indexes(vArea);

# if 1

	cout << "*****************************************************************ͼ�����ŶȱȽ�" << endl;
	cout << "vArea��";
	for (int i = 0; i < vImg.size(); i++) {
		cout << vArea[i] << " ";
	}
	cout << endl;
	cout << "vIndex��";
	for (int i = 0; i < vImg.size(); i++) {
		cout << vIndex[i] << " ";
	}
	cout << endl;
	cout << "*******************************************************************************" << endl;

#endif

	return vIndex;
}

int CountImageWhiteArea(const cv::Mat& img, const cv::Mat& depth, const double moreThanVal, cv::Mat& mask) {
	//imshow("src", img);
	//waitKey(0);
	mask = cv::Mat(img.rows, img.cols, CV_8U, cv::Scalar(0));
	cv::Mat binary;
	if (img.channels() != 1) {
		// ��ֵ��
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
		cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
	}
	else
		binary = img;
	//cout << "row:" << img.rows << "	col:" << img.cols << endl;
	int validCnt = 0;
	for (int row = 0; row < img.rows; row++) {
		for (int col = 0; col < img.cols; col++) {
			//cout << (int)vImg[i].at<uchar>(row, col) << " ";
			if (binary.at<uchar>(row, col) == 255 && depth.at<double>(row, col) > moreThanVal) {
				// ǰ�� 
				validCnt++;
				mask.at<uchar>(row, col) = 255;
			}
			else
				mask.at<uchar>(row, col) = 128;
		}
	}

	return validCnt;
}

double CountImageAgvDepth(const cv::Mat& depth, cv::Mat& mask) {
	int validCnt = 0;
	int totalVal = 0;
	for (int row = 0; row < depth.rows; row++) {
		for (int col = 0; col < depth.cols; col++) {
			//cout << (int)vImg[i].at<uchar>(row, col) << " ";
			if (mask.at<uchar>(row, col) == 255 && depth.at<UINT16>(row, col) != 0) {
				// ǰ�� 
				validCnt++;
				totalVal += depth.at<UINT16>(row, col);
			}
		}
	}

	return double(totalVal) / validCnt;
}

// ����ʵʱͼƬ��λ����Ϣ
void IntegrateImgInfo(const cv::Mat& rgb, const cv::Mat& depth, string pose, int type, cameraImgInfo& cameraInfo, const Eigen::Vector3d agvPose, const Eigen::Vector3d agvIMU) {
	// ͼƬ����
	cameraInfo._type = type;
	// �ַ���תMat
	cv::Mat str2MatData(1, 6, CV_64F, 0.0);
	AnalysisString26D(pose, str2MatData);
	//cout << str2MatData.at<double>(0, 0) << "	" << str2MatData.at<double>(0, 1) << "	" << str2MatData.at<double>(0, 2)
	//	<< "	" << str2MatData.at<double>(0, 3) << "	" << str2MatData.at<double>(0, 4) << "	" << str2MatData.at<double>(0, 5) << endl;
	// _Gripper2Base
	cameraInfo._Gripper2Base = attitudeVectorToMatrix(str2MatData.row(0), false, "xyz");
	// _CameraPose
	if (type == 1) cameraInfo._CameraPose = cameraInfo._Gripper2Base * H_Camera2Gripper;
	else if (type == 0) cameraInfo._CameraPose = cameraInfo._Gripper2Base * H_Camera2Gripper_HD;
	else cameraInfo._CameraPose = cameraInfo._Gripper2Base * H_Thermal2Gripper;
	// _CameraPosition
	cv::Mat cameraPosition = cameraInfo._CameraPose * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	cameraInfo._CameraPosition = Eigen::Vector3d(cameraPosition.at<double>(0, 0), cameraPosition.at<double>(1, 0), cameraPosition.at<double>(2, 0));
	// _CameraPostureEig��_CameraPostureEig
	cv::Mat cameraPosture = cameraInfo._CameraPose(cv::Range(0, 3), cv::Range(0, 3)); // ǰ3*3
	cameraInfo._CameraPostureEig = RotationMatrixToEulerAngles(cameraPosture);
	cameraInfo._CameraPostureMat = cameraPosture.clone();
	// _RgbImg
	cameraInfo._RgbImg = rgb.clone();
	// ��ӡ��ֵ����
	if (type == 0) {
		cv::Mat gray;
		cv::cvtColor(cameraInfo._RgbImg, gray, cv::COLOR_BGR2GRAY);
		cv::Mat meanval, varval;
		cv::meanStdDev(gray, meanval, varval);
		cout << "mean:" << meanval.t() << "	var:" << varval.t() << endl;
	}
	// _DepthImg
	cameraInfo._DepthImg = depth.clone();
	// string
	cameraInfo._poseStr = pose;
	// agv info
	cameraInfo._agvPose = agvPose;
	// agv imu
	cameraInfo._agvIMU = agvIMU;

	return;
}

void IntegrateVildBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype) {
	// type
	bboxInfo._imgType = imgtype;
	// src
	bboxInfo._src = cv::imread(bboxPth + "src.jpg");
	cv::Mat srcGray;
	cv::cvtColor(bboxInfo._src, srcGray, cv::COLOR_BGR2GRAY);
	// VILD bboxλ��
	IOFile iof(bboxPth + "box.txt", 0);
	vector<vector<float>> vvBboxInfo;
	iof.ReadFile2FloatArr(vvBboxInfo);
	//cout << "bbox size:" << vvBboxInfo.size() << "	" << bboxPth + "bbox.txt" << endl;
	vector<string> vBboxMaskInfo(vvBboxInfo.size()); // bbox maskͼ��
	vector<cv::Mat> vImg(vvBboxInfo.size()); // ͼƬ��Ĥ
	vector<double> vMaskArea(vvBboxInfo.size()); // ��Ĥ���
	vector<Eigen::Vector4i> v4DBoxPos(vvBboxInfo.size(), Eigen::Vector4i(0, 0, 0, 0)); // bbox����
	vector<cv::Rect> v4DBoxPosRect(vvBboxInfo.size(), cv::Rect(0, 0, 0, 0)); // bbox����
	// ����BBox��Ӧ����ģ��Ϣ
	for (int bbox = 0; bbox < vvBboxInfo.size(); bbox++) {
		v4DBoxPos[bbox] = Eigen::Vector4i((int)vvBboxInfo[bbox][0], (int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][2], (int)vvBboxInfo[bbox][3]);
		v4DBoxPosRect[bbox] = cv::Rect((int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][0], ((int)vvBboxInfo[bbox][3] - (int)vvBboxInfo[bbox][1]), ((int)vvBboxInfo[bbox][2] - (int)vvBboxInfo[bbox][0]));
		vBboxMaskInfo[bbox] = bboxPth + to_string(bbox) + ".jpg";
		cv::Mat srcMask = cv::imread(vBboxMaskInfo[bbox]);
		cv::Mat dst(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
		cv::Mat gray;
		cvtColor(srcMask, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, dst, 128, 255, cv::THRESH_BINARY);
		if (imgtype == 0) {
			// RGBD�ű�����Ĥ
			vImg[bbox] = dst.clone();
		}
	}
	bboxInfo._imgPath = vBboxMaskInfo;
	bboxInfo._bboxRect = v4DBoxPosRect;
	bboxInfo._bboxAB = v4DBoxPos;
	bboxInfo._maskArea = vMaskArea;
	bboxInfo._img = vImg;
	bboxInfo._selectmask = vImg[0];
}

void IntegrateBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype, int inftype) {
	//	// BTS
	//	if (inftype != 0) {
	//		char buff[10];
	//		sprintf(buff, "%04d", inftype - 1);
	//		cv::Mat srcBgr = cv::imread(bboxPth + buff + ".png");
	//		cv::Mat gray, binary;
	//		cv::cvtColor(srcBgr, gray, cv::COLOR_BGR2GRAY);
	//		cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
	//		bboxInfo._selectmask = binary;
	//
	//		return;
	//	}
	//
	//	// type
	//	bboxInfo._imgType = imgtype;
	//	// src
	//	bboxInfo._src = cv::imread(bboxPth + "src.jpg");
	//	cv::Mat srcGray;
	//	cv::cvtColor(bboxInfo._src, srcGray, cv::COLOR_BGR2GRAY);
	//	// VILD bboxλ��
	//	IOFile iof(bboxPth + "box.txt", 0);
	//	vector<vector<float>> vvBboxInfo;
	//	iof.ReadFile2FloatArr(vvBboxInfo);
	//	//cout << "bbox size:" << vvBboxInfo.size() << "	" << bboxPth + "bbox.txt" << endl;
	//	vector<string> vBboxMaskInfo(vvBboxInfo.size()); // bbox maskͼ��
	//	vector<cv::Mat> vImg(vvBboxInfo.size()); // ͼƬ��Ĥ
	//	vector<double> vMaskArea(vvBboxInfo.size()); // ��Ĥ���
	//	vector<Eigen::Vector4i> v4DBoxPos(vvBboxInfo.size(), Eigen::Vector4i(0, 0, 0, 0)); // bbox����
	//	vector<cv::Rect> v4DBoxPosRect(vvBboxInfo.size(), cv::Rect(0, 0, 0, 0)); // bbox����
	//	bboxInfo._maskContour = vector<vector<Point>>(vvBboxInfo.size()); // bbox����
	//	bboxInfo._totalContours = vector<vector<Point>>(vvBboxInfo.size()); // bbox����
	//	// ����BBox��Ӧ����ģ��Ϣ
	//	for (int bbox = 0; bbox < vvBboxInfo.size(); bbox++) {
	//		//cout << vvBboxInfo[bbox][0] << " " << vvBboxInfo[bbox][1] << " " << vvBboxInfo[bbox][2] << " " << vvBboxInfo[bbox][3] << endl;
	//		v4DBoxPos[bbox] = Eigen::Vector4i((int)vvBboxInfo[bbox][0], (int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][2], (int)vvBboxInfo[bbox][3]);
	//		v4DBoxPosRect[bbox] = cv::Rect((int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][0], ((int)vvBboxInfo[bbox][3] - (int)vvBboxInfo[bbox][1]), ((int)vvBboxInfo[bbox][2] - (int)vvBboxInfo[bbox][0]));
	//		char bboxName[12];
	//		sprintf_s(bboxName, "%d.jpg", bbox); // ".jpg"
	//		string maskImagePath = bboxPth + string(bboxName);
	//		vBboxMaskInfo[bbox] = maskImagePath; // mask
	//		cv::Mat srcMask = imread(maskImagePath);
	//		cv::Mat dst(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
	//		Mat gray;
	//		cvtColor(srcMask, gray, COLOR_BGR2GRAY);
	//		threshold(gray, dst, 128, 255, THRESH_BINARY);
	//		if (imgtype == 0) {
	//			// RGBD�ű�����Ĥ
	//			vImg[bbox] = dst.clone();
	//		}
	//		// �������
	//		vector<vector<cv::Point> > contours;
	//		vector<Vec4i> hierarchy;
	//		cv::findContours(dst, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	//		vector<vector<cv::Point>>::iterator itr = contours.begin();
	//		double area = 0.;
	//		double areaMax = 0.;
	//		int optimumIdx = 0;
	//		while (itr != contours.end()) {
	//			double currArea = cv::contourArea(*itr);
	//			if (currArea > areaMax) optimumIdx = itr - contours.begin();
	//			area += currArea;
	//			areaMax = max(currArea, areaMax);
	//			itr++;
	//		}
	//		bboxInfo._maskContour[bbox] = vector<cv::Point>(contours[optimumIdx]);
	//		vMaskArea[bbox] = area;
	//		//cv::namedWindow("bbox", cv::WINDOW_NORMAL);
	//		//cv::imshow("bbox", vMaskSrc[bbox]);
	//		//cv::waitKey(0);
	//	}
	//	bboxInfo._imgPath = vBboxMaskInfo;
	//	bboxInfo._bboxRect = v4DBoxPosRect;
	//	bboxInfo._bboxAB = v4DBoxPos;
	//	bboxInfo._maskArea = vMaskArea;
	//	bboxInfo._img = vImg;
	//	//bboxInfo._selectmask = cv::Mat(bboxInfo._src.rows, bboxInfo._src.cols, CV_8U, cv::Scalar(0));
	//	bboxInfo._selectmask = vImg[0];
	//	
	//	// image����
	//	if (imgtype == 0) {
	//		cv::Mat gray, binary, blur;
	//		vector<vector<cv::Point>>contours;//����
	//		vector<cv::Vec4i>hierachy;//��������ṹ����
	//		cv::cvtColor(bboxInfo._src, gray, COLOR_BGR2GRAY);
	//		cv::GaussianBlur(gray, gray, Size(9, 9), 2, 2);//ƽ���˲�
	//		cv::threshold(gray, binary, 170, 255, THRESH_BINARY | THRESH_OTSU);//����Ӧ��ֵ��
	//		cv::findContours(binary, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	//		//for (int i = 0; i < bboxInfo._totalContours.size(); i++) {
	//		//	for (int k = 0; k < gray.rows; k++) {
	//		//		contours[i].push_back(cv::Point(0, k));
	//		//		contours[i].push_back(cv::Point(gray.rows - 1, k));
	//		//	}
	//		//	for (int j = 0; j < gray.cols; j++) {
	//		//		contours[i].push_back(cv::Point(j, 0));
	//		//		contours[i].push_back(cv::Point(j, gray.cols - 1));
	//		//	}
	//		//}
	//		bboxInfo._totalContours = contours;
	//
	//		static int  num = 0;
	//		cv::Mat mask(bboxInfo._src.rows, bboxInfo._src.cols, CV_8UC3, Scalar(0, 0, 0));
	//		cv::drawContours(mask, contours, -1, Scalar(0, 0, 255), 2, 8);
	//		cv::imwrite(dataPath + "temp/test/" + to_string(num) + ".jpg", mask);
	//		//cv::imwrite(dataPath + "temp/test/" + to_string(num) + "src.jpg", bboxInfo._src);
	//		//for (int i = 0; i < contours.size(); ++i) {
	//		//	bboxInfo._totalContours[i] = contours[i];
	//		//}
	//
	//		//bboxInfo._contour = vector<vector<Point>>(vvBboxInfo.size());
	//		//bboxInfo._contourErr = vector<double>(vvBboxInfo.size());
	//		//for (int i = 0; i < vvBboxInfo.size(); ++i) {
	//		//	vector<vector<cv::Point>> vContours(1);
	//		//	vContours[0] = bboxInfo._maskContour[i];
	//		//	vector<cv::Point> srcOptimumContours, optimumed;
	//		//	double err = FindOptimumContours(bboxInfo._totalContours, vContours, srcOptimumContours, optimumed);
	//		//	//cout << "optimumed:" << optimumed.size() << "	" << bboxInfo._maskContour[i].size() << endl;
	//		//	bboxInfo._contourErr[i] = err;
	//		//	bboxInfo._contour[i] = vector<cv::Point>(bboxInfo._totalContours[i]);
	//		//	//bboxInfo._contour[i] = vector<cv::Point>(optimumed);
	//		//	//if (err < 10.) {
	//		//	//	//cout << err << "	" << num << "	" << i << endl;
	//		//	//	// ������Сʱ�����foundcontourѰ�ҵ�
	//		//	//	bboxInfo._totalContours[i] = optimumed;
	//		//	//}
	//		//	vector<vector<cv::Point>> vDrawContours(1);
	//		//	vDrawContours[0] = optimumed;
	//		//	cv::Mat mask(bboxInfo._src.rows, bboxInfo._src.cols, CV_8UC3, Scalar(0, 0, 0));
	//		//	cv::drawContours(mask, vDrawContours, 0, Scalar(0, 0, 255), 2, 8);
	//		//	cv::drawContours(mask, vContours, 0, Scalar(0, 0, 255), 2, 8);
	//		//	cv::imwrite(dataPath + "temp/test/" + to_string(num) + "_" + to_string(i) + ".jpg", mask);
	//		//}
	//		for (int i = 0; i < bboxInfo._totalContours.size(); i++) {
	//			cv::Mat mask(bboxInfo._src.rows, bboxInfo._src.cols, CV_8UC3, Scalar(0, 0, 0));
	//			cv::drawContours(mask, bboxInfo._totalContours, i, Scalar(0, 0, 255), 2, 8);
	//			cv::imwrite(dataPath + "temp/test/" + to_string(num) + "_" + to_string(i) + "optimum.jpg", mask);
	//		}
	//		num++;
	//	}
}

Eigen::Vector3d ConvertPixel2Camera(const int& u, const int& v, const ushort& d, const int& cameraType) {
	//�ֱ������ţ��������궨ʱ�ķֱ��ʷ�RESOULTION_X��RESOULTION_Y
	// ����ڲ�
	cv::Mat cameraInt;
	if (cameraType == 0) cameraInt = CameraMatrix;
	else if (cameraType == 1) cameraInt = CameraMatrix_HD;
	else cameraInt = ThermalMatrix;
	double fdx = cameraInt.at<double>(0, 0);
	double fdy = cameraInt.at<double>(1, 1);
	double u0 = cameraInt.at<double>(0, 2);
	double v0 = cameraInt.at<double>(1, 2);
	//cout << "fdx:" << fdx << "	fdy:" << fdy << "	u0:" << u0 << "	v0:" << v0 << endl;
	if (fdx == 0 || fdy == 0) return Eigen::Vector3d{ 0.,0.,0. };
	// ��͸��ͶӰ
	double tx = (u - u0) / fdx;
	double ty = (v - v0) / fdy;
	//cout << Eigen::Vector3d{ (double)d * tx, (double)d * ty, (double)d }.transpose() << endl;

	return Eigen::Vector3d{ (double)d * tx, (double)d * ty, (double)d };
}

double FindHHOptimumExposureTime() {
	//static int exptime = 0;
	double optimumTime = 0.;
	char hdPath[100];
	for (int i = 10; i < 50; i++) {
		set_mindivision_exposuretime((double)i);
		//sprintf(hdPath, "%s%s%04d", dataPath, "recollect/exposuretime/", i);
		save_mindvision(hdPath); // �������ͼ
		sprintf(hdPath, "%s%s", hdPath, ".bmp");
		cv::Mat currMat = cv::imread(hdPath);
		cv::Mat gray;
		cv::cvtColor(currMat, gray, cv::COLOR_BGR2GRAY);
		cv::Mat meanval, varval;
		cv::meanStdDev(gray, meanval, varval);
		cout << "exposure time:" << i << "ms	mean:" << meanval.t() << "	var:" << varval.t() << endl;
	}
	return 0.;
}

/**
	ȥ�������еı�Ե��
	@para vPoints:ȥ���㼯�еı�Ե��
*/	
void RemoveEdgePoints(vector<cv::Point>& vPoints) {
	int remain = 5;
	vector<cv::Point> vPointsDup;
	for (auto point : vPoints) {
		if (point.x <= remain || point.x >= 639 - remain || point.y <= remain || point.y >= 479 - remain) {
			continue;
		}
		vPointsDup.emplace_back(point);
	}	
	vPoints = vPointsDup;
}

cv::Mat FillImageBasePoint(const vector<cv::Point>& vPoints) {
	std::vector<cv::Point> pointsVec;
	for (const cv::Point2f& point : vPoints) {
		pointsVec.push_back(cv::Point(point.x, point.y));
	}
	std::vector<cv::Point> convexHull;
	cv::convexHull(pointsVec, convexHull);
	std::vector<std::vector<cv::Point>> contour(1, convexHull);
	cv::Mat binaryImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));  // ����һ����ͨ���ĺ�ɫͼ��
	cv::drawContours(binaryImage, contour, 0, cv::Scalar(255), cv::FILLED);  // �������
	for (const cv::Point2f& point : vPoints) {
		cv::circle(binaryImage, point, 5, cv::Scalar(255), 2);
	}


	return binaryImage;
}

vector<vector<cv::Point>> FindContours(const cv::Mat& src, bool oneMax) {
	cv::Mat img = src.clone();
	cv::Mat gray, binary, blur;
	vector<vector<cv::Point>>contours;//����
	vector<cv::Vec4i>hierachy;//��������ṹ����
	if (src.channels() != 1) {
		cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);//ƽ���˲�
		cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);//����Ӧ��ֵ��
	}
	else {
		// binaryֻ�����������
		binary = src.clone();
	}
	static int num = 0;
	//�������������
	// cv::findContours(binary, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	cv::findContours(binary, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0) return contours;
	if (!oneMax) {
		//�������� ��������
		//cout << "��������" << contours.size() << endl;
		for (int i = 0; i < contours.size(); ++i) {
			int currArea = cv::contourArea(contours[i]);
			double currLen = cv::arcLength(contours[i], true);
			//bool close = cv::isContourConvex(contours[i]);
			//cout << "���������" << currArea << "	���ȣ�" << currLen << "	��գ�" << close << endl;
			//if (currArea < 1000 || currArea > 600 * 400) continue;
			//if (currLen < 1000) continue;
			if (currArea < 100 || currArea > 600 * 400) continue;
			cv::Mat mask(src.rows, src.cols, CV_8UC3, cv::Scalar(0, 0, 0));
			cv::drawContours(mask, contours, i, cv::Scalar(0, 0, 255), 2, 8);
			//cv::imwrite(dataPath + "temp/contours/" + to_string(num) + "_" + to_string(i) + ".jpg", mask);
		}
	}
	else {
		// ����һ�������
		//cout << "��������" << contours.size() << endl;
		int maxIdx = 0;
		double maxLen = 0;
		for (int i = 0; i < contours.size(); ++i) {
			double currLen = cv::arcLength(contours[i], true);
			maxIdx = currLen > maxLen ? i : maxIdx;
			maxLen = max(maxLen, currLen);
		}
		vector<cv::Point> optimumContours = contours[maxIdx];
		cv::Mat mask(src.rows, src.cols, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::drawContours(mask, contours, maxIdx, cv::Scalar(0, 0, 255), 2, 8);
		//cv::imwrite(dataPath + "temp/contours/" + to_string(num) + ".jpg", mask);
		contours.clear();
		contours.emplace_back(optimumContours);
		// ����һ�������
		//cout << "��������" << contours.size() << endl;
	}
	num++;
	return contours;
}

double FindOptimumContours(const vector<vector<cv::Point>>& src, const vector<vector<cv::Point>>& mask, vector<cv::Point>& optimumSrc, vector<cv::Point>& optimumed) {
	//if (src.size() == 0 || mask.size() != 1) return INT32_MAX;
	if (src.size() == 0) return INT32_MAX;
	double TotalErr = 0.;
	for (int j = 0; j < mask.size(); j++) {
		//int optimumIdx = 0;
		double minErr = INT32_MAX;
		vector<cv::Point> vResult;
		for (int i = 0; i < src.size(); i++) {
			// ��������������Ŀ���������
			vector<cv::Point> currContours = src[i];
			if (currContours.size() < mask[j].size() * 0.5) continue;
			double totalSubErr = 0.;
			vector<cv::Point> vTmp;
			for (int pIdx = 0; pIdx < mask[j].size(); pIdx++) {
				// ����Ŀ���������е㣬��ο����������
				double minSubErr = INT32_MAX;
				int minSubIdx = 0;
				cv::Point dstPoint = mask[j][pIdx];
				//cout << dstPoint.x << "	" << dstPoint.y << endl;
				for (int pIdxSrc = 0; pIdxSrc < currContours.size(); pIdxSrc++) {
					cv::Point srcPoint = currContours[pIdxSrc];
					if (norm(srcPoint - dstPoint) < minSubErr) minSubIdx = pIdxSrc;
					minSubErr = min(norm(srcPoint - dstPoint), minSubErr);
					//cout << norm(srcPoint - dstPoint) << endl;
				}
				//// ͼ��߽�Ҳ������
				//if (minSubErr > dstPoint.x && dstPoint.x < 2) { minSubErr = dstPoint.x; currContours[minSubIdx] = cv::Point{ 0 , dstPoint.y }; }
				//else if (minSubErr > 639 - dstPoint.x && dstPoint.x > 637) { minSubErr = 639 - dstPoint.x; currContours[minSubIdx] = cv::Point{ 639 , dstPoint.y }; }
				//else if (minSubErr > dstPoint.y && dstPoint.y < 2) { minSubErr = dstPoint.y; currContours[minSubIdx] = cv::Point{ dstPoint.x , 0 }; }
				//else if (minSubErr > 479 - dstPoint.y && dstPoint.y > 477) { minSubErr = 479 - dstPoint.y; currContours[minSubIdx] = cv::Point{ dstPoint.x ,479 }; }
				//else vTmp.emplace_back(currContours[minSubIdx]);
				// ���������ͼ��߽磬�򲻱���
				//cout << "contours point:" << currContours[minSubIdx].x << "	" << currContours[minSubIdx].y << endl;
				int remain = 3;
				if (dstPoint.x <= remain || dstPoint.x >= 639 - remain || dstPoint.y <= remain || dstPoint.y >= 479 - remain) {
					continue;
				}
				//if (currContours[minSubIdx].x == 0 || currContours[minSubIdx].x == 639 
				//	|| currContours[minSubIdx].y == 0 || currContours[minSubIdx].y == 479) {
				//	continue;
				//}
				else vTmp.emplace_back(currContours[minSubIdx]);
				totalSubErr += minSubErr;
			}
			if (totalSubErr < minErr) {
				//optimumIdx = i;
				vResult = vTmp;
			}
			minErr = min(minErr, totalSubErr);
		}
		TotalErr += minErr;
		optimumed.insert(optimumed.begin(), vResult.begin(), vResult.end());
	}
	optimumSrc = src[0];
	return TotalErr / (double)mask[0].size();
}

//double FindOptimumContours(const vector<vector<Point>>& src, const vector<vector<Point>>& mask, vector<Point>& optimumSrc, vector<Point>& optimumed) {
//	if (src.size() == 0 || mask.size() != 1) return INT32_MAX;
//	int optimumIdx = 0;
//	double minErr = INT32_MAX;
//	//vector<cv::Point> vResult;
//	for (int i = 0; i < src.size(); i++) {
//		// ��������������Ŀ���������
//		vector<cv::Point> currContours = src[i];
//		if (currContours.size() < mask[0].size() * 0.5) continue;
//		double totalSubErr = 0.;
//		vector<cv::Point> vTmp;
//		set<cv::Point> sTmp;
//		for (int pIdx = 0; pIdx < mask[0].size(); pIdx++) {
//			// ����Ŀ���������е㣬��ο����������
//			double minSubErr = INT32_MAX;
//			int minSubIdx = 0;
//			cv::Point dstPoint = mask[0][pIdx];
//			//cout << dstPoint.x << "	" << dstPoint.y << endl;
//			for (int pIdxSrc = 0; pIdxSrc < currContours.size(); pIdxSrc++) {
//				cv::Point srcPoint = currContours[pIdxSrc];
//				if (norm(srcPoint - dstPoint) < minSubErr) minSubIdx = pIdxSrc;
//				minSubErr = min(norm(srcPoint - dstPoint), minSubErr);
//				//cout << norm(srcPoint - dstPoint) << endl;
//			}
//			
//			//if (minSubErr > dstPoint.x || minSubErr > abs(640 - dstPoint.x)
//			//	|| minSubErr > dstPoint.y || minSubErr > abs(480 - dstPoint.y)) {
//			//	// û�иõ�������
//			//	minSubErr = min((double)dstPoint.x, minSubErr);
//			//	minSubErr = min((double)abs(640 - dstPoint.x), minSubErr);
//			//	minSubErr = min((double)dstPoint.y, minSubErr);
//			//	minSubErr = min((double)abs(480 - dstPoint.y), minSubErr);
//			//	totalSubErr += minSubErr;
//			//	vTmp.emplace_back(dstPoint);
//			//}
//			//else {
//				totalSubErr += minSubErr;
//				vTmp.emplace_back(currContours[minSubIdx]);
//			//}
//			//cout << pIdx << "	" << minSubIdx << "	" << minSubErr << endl;
//			//sTmp.insert(currContours[minSubIdx]);
//		}
//		if (totalSubErr < minErr) {
//			optimumIdx = i;
//			//vResult = vTmp;
//			optimumed = vTmp;
//		}
//		minErr = min(minErr, totalSubErr);
//	}
//	optimumSrc = src[optimumIdx];
//	//cout << "err:" << minErr / mask[0].size() << endl;
//	//return vResult;
//	return minErr / (double)mask[0].size();
//}

bool InterSect(const cv::Mat& m1, const cv::Mat& m2) {
	if (m1.rows != m2.rows || m1.cols != m2.cols) return false;
	if (m1.channels() != m2.channels() || m1.channels() != 1) return false;
	bool haveVildPixel = false;
	for (int row = 0; row < m1.rows; row++) {
		for (int col = 0; col < m1.cols; col++) {
			if (m1.at<uchar>(row, col) == 255 && m1.at<uchar>(row, col) == m2.at<uchar>(row, col)) return true;
			if (!haveVildPixel && m1.at<uchar>(row, col) == 255) haveVildPixel = true;
		}
	}
	return haveVildPixel ? false : true;
}

bool WhetherUpdate(const cv::Mat& src, const cv::Mat& update) {
	cv::Mat mask;
	cv::Mat depth(src.rows, src.cols, CV_64F, cv::Scalar(0.));
	int num1 = CountImageWhiteArea(src, depth, -1., mask);
	int num2 = CountImageWhiteArea(update, depth, -1., mask);
	return (num2 > 0.5 * num1 ? true : false);
}

void IntegrateTextInfo(const string path, const int imgType, vector<cameraImgInfo>& vCam, vector<Eigen::Vector3d>& vAgv) {
	// ��ȡtxt����
	string file;
	if (imgType == 0)file = path + "rgbd.txt";
	else if (imgType == 1)file = path + "thermal.txt";
	cout << file << endl;
	fstream fsread(file, ios::in);
	string currLine;
	vector<double> currAgvInfo; // ��ǰAGVλ��
	string currArmInfo; // ��ǰ��е��λ��
	int infoIdx = 0;
	if (imgType == 1) infoIdx = 0;
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
		// ��ȡͼƬ��ӦAGVλ�ã������ַ�����double
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
			vAgv.emplace_back(Eigen::Vector3d{ currAgvInfo[0], currAgvInfo[1], currAgvInfo[2] });
			cameraImgInfo cii;
			if (imgType == 0) {
				cv::Mat rgb = cv::imread(path + "rgbd/" + to_string(infoIdx) + ".jpg");
				cv::Mat depth = cv::imread(path + "rgbd/" + to_string(infoIdx) + ".png", cv::IMREAD_ANYDEPTH);
				IntegrateImgInfo(rgb, depth, currArmInfo, 1, cii);
			}
			else if (imgType == 1) {
				cv::Mat rgb = cv::imread(path + "thermal/" + to_string(infoIdx) + ".jpg");
				IntegrateImgInfo(rgb, rgb, currArmInfo, 2, cii);
			}
			vCam.emplace_back(cii);
		}
	}
	fsread.close();
}