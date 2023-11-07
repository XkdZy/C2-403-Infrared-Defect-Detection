#include "RoutePlaning.h"
#include "GlobalVariable.h"
#include "BaseRobotArm.h"
#include "CoordinateTransformation.h"
#include "RobotAlgorithm.h"
#include "OperatePointCloud.h"
#include "IOFile.h"
#include "AstraCameraD2C.h"

vertex startVertex;
vertex goalVertex;
vertex* allVertexArr;
vertex* expandedVertexArr;
Voxel2PclInfo v2pInfo;
vector<int> vCurrentTrajectory;
vector<vertex> vCurrentTrajectoryVertex;
vector<Eigen::Vector3d> vAGVMovePosition; // AGV�ƶ����꣨AGV����ϵ�£�
vector<Eigen::Vector3d> vArmMovePosition; // ��е���ƶ����꣨AGV��ʼλ��ʱ�Ļ�е������ϵ�£�

vector<int> vTest;

int CurrAGVMoveIdx; // ��ǰAGV�ƶ���λ����
Eigen::Vector3d objectCenterPos; // �ֶ�λĿ����������

void FromObjPos2CamObservePos(const Eigen::Vector3d& camPos, const Eigen::Vector3d& objPos, Eigen::Vector3i camDisPos) {

	return;
}

void GetSearchRegion(const Eigen::Vector3i& start, const Eigen::Vector3i& goal, SearchVoxelRegion& svr) {
	// �豸ռ�����ؿ��С
	int occupNum = DEVICE_OCCUPYVOXEL;
	cout << "�豸ռ������Ϊ��" << occupNum << endl;
	// �����豸��Сȷ��ѡ�õ����ؿ�������Ч�������� = abs(start - goal) + 2*occupNum
	int minX = min(start[0], goal[0]) - occupNum;
	int minY = min(start[1], goal[1]) - occupNum;
	int minZ = min(start[2], goal[2]) - occupNum;
	int height = abs(goal[2] - start[2]) + 2 * occupNum;
	int width = abs(goal[1] - start[1]) + 2 * occupNum;
	int length = abs(goal[0] - start[0]) + 2 * occupNum;
	// �޷�
	minX = minX > 0 ? minX : 0;
	minY = minY > 0 ? minY : 0;
	minZ = minZ > 0 ? minZ : 0;
	// ��ֵ
	svr.v = Eigen::Vector3i(minX, minY, minZ);
	svr.height = height;
	svr.width = width;
	svr.length = length;

	return;
}

void VertexNumFromSearchRegion(const SearchVoxelRegion& svr, const Eigen::Vector3i& start, const Eigen::Vector3i& goal, vertex& startVertex, vertex& goalVertex) {
	int xNorm = svr.length;
	int yNorm = svr.width;
	int zNorm = svr.height;
	cout << "xNorm��" << xNorm << "	yNorm��" << yNorm << "	zNorm��" << zNorm << endl;
	// ��ʼ��������Ϊ0�����ؿ���ԭ�㣨minX��minY��minZ��
	// �������飬vector ���벻��̫���ڴ�
	allVertexArr = (vertex*)malloc(xNorm * yNorm * zNorm * sizeof(vertex{ false, 0, svr.v, Eigen::Vector3d(0.0, 0.0, 0.0),(int*)malloc(sizeof(int) * 6) }));
	// Minxyz��ԭ�㣩
	for (int z = 0; z < zNorm; z++) {
		for (int y = 0; y < yNorm; y++) {
			for (int x = 0; x < xNorm; x++) {
				// �������
				int voxelNum = (x)+xNorm * (y)+xNorm * yNorm * (z); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
				//cout << "z:" << z << "	y:" << y << "	x:" << x << "	voxelNum:" << voxelNum << endl;
				vertex v;
				v.num = voxelNum; // ���
				v.v = svr.v + Eigen::Vector3i(x, y, z); // ��������
				// ���ر�Ŷ�Ӧ��ʵ�ʵ���λ�� ???????????????????������е�����
				v.vPcl = v2pInfo.minPoint + VOXEL_SIZE * Eigen::Vector3d(v.v[0] + 0.9, v.v[1] - 0.1, v.v[2] + 1.6); // �������ض�������������λ��xyz
				//v.vPcl = v2pInfo.minPoint + VOXEL_SIZE * Eigen::Vector3d(v.v[0] + 0.5, v.v[1] + 0.5, v.v[2] + 0.5); // �������ض�������������λ��xyz
				// v2pInfo.minPoint + VOXEL_SIZE * Eigen::Vector3d(nowVertex.v[0] - 0.5, nowVertex.v[1] - 0.5, nowVertex.v[2] - 0.5); // �������ض�������������λ��xyz
				auto itr = pcl_voxel->voxels_.find(v.v); // ռ�����
				if (itr != pcl_voxel->voxels_.end()) {
					v.occupy = true;
				}
				else {
					v.occupy = false;
				}
				v.vVertexNum = (int*)malloc(sizeof(int) * 6); // ��������
				memset(v.vVertexNum, -1, sizeof(int) * 6);
				//v.vVertexNum = vector<int>(6, -1);
				// 6�����������Ƿ���������Χ��
				if (x + 1 < xNorm) {
					int num = (x + 1) + xNorm * (y)+xNorm * yNorm * (z); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
					v.vVertexNum[0] = num;
				}
				if (x - 1 >= 0) {
					int num = (x - 1) + xNorm * (y)+xNorm * yNorm * (z); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
					v.vVertexNum[1] = num;
				}
				if (y + 1 < yNorm) {
					int num = (x)+xNorm * (y + 1) + xNorm * yNorm * (z); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
					v.vVertexNum[2] = num;
				}
				if (y - 1 >= 0) {
					int num = (x)+xNorm * (y - 1) + xNorm * yNorm * (z); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
					v.vVertexNum[3] = num;
				}
				if (z + 1 < zNorm) {
					int num = (x)+xNorm * (y)+xNorm * yNorm * (z + 1); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
					v.vVertexNum[4] = num;
				}
				if (z - 1 >= 0) {
					int num = (x)+xNorm * (y)+xNorm * yNorm * (z - 1); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
					v.vVertexNum[5] = num;
				}
				allVertexArr[voxelNum] = v;
			}
		}
	}
	startVertex = allVertexArr[(start[0] - svr.v[0]) + (start[1] - svr.v[1]) * xNorm + (start[2] - svr.v[2]) * xNorm * yNorm];
	cout << "---------------------startVertex:" << startVertex.v.transpose() << "	" << startVertex.vPcl.transpose() << endl;
	goalVertex = allVertexArr[(goal[0] - svr.v[0]) + (goal[1] - svr.v[1]) * xNorm + (goal[2] - svr.v[2]) * xNorm * yNorm];
	//cout << endl << "��ʼ������Ϣ��" << endl;
	//cout << "���ر�ţ�" << startVertex.num << endl;
	//cout << "����λ�ã�" << startVertex.v.transpose() << endl;
	//for (int j = 0; j < 6; j++) {
	//	cout << startVertex.vVertexNum[j] << " ";
	//}
	//cout << endl << "Ŀ��������Ϣ��" << endl;
	//cout << "���ر�ţ�" << goalVertex.num << endl;
	//cout << "����λ�ã�" << goalVertex.v.transpose() << endl;
	//for (int j = 0; j < 6; j++) {
	//	cout << goalVertex.vVertexNum[j] << " ";
	//}

	return;
}

void FillCircleBasePoint(Eigen::Vector3i voxel, vertex* expandedVertex, SearchVoxelRegion& svr) {
	int xNorm = svr.length;
	int yNorm = svr.width;
	int zNorm = svr.height;
	// �豸ռ�����ؿ��С
	int occupNum = DEVICE_OCCUPYVOXEL;
	for (int i = -occupNum; i < occupNum; i++) {
		for (int j = -occupNum; j < occupNum; j++) {
			for (int k = -occupNum; k < occupNum; k++) {
				double distance = (Eigen::Vector3i(i, j, k) - voxel).norm();
				if (distance > DEVICE_OCCUPYVOXEL + 1) {
					continue;
				}
				Eigen::Vector3i nowVoxel = voxel + Eigen::Vector3i(i, j, k) + svr.v;
				// �޷�
				nowVoxel[0] = nowVoxel[0] > svr.v[0] + svr.length - 1 ? svr.v[0] + svr.length - 1 : nowVoxel[0];
				nowVoxel[1] = nowVoxel[1] > svr.v[1] + svr.width - 1 ? svr.v[1] + svr.width - 1 : nowVoxel[1];
				nowVoxel[2] = nowVoxel[2] > svr.v[2] + svr.height - 1 ? svr.v[2] + svr.height - 1 : nowVoxel[2];
				nowVoxel[0] = nowVoxel[0] < svr.v[0] ? svr.v[0] : nowVoxel[0];
				nowVoxel[1] = nowVoxel[1] < svr.v[1] ? svr.v[1] : nowVoxel[1];
				nowVoxel[2] = nowVoxel[2] < svr.v[2] ? svr.v[2] : nowVoxel[2];
				nowVoxel = nowVoxel - svr.v;
				// �������
				int voxelNum = (nowVoxel[0]) + xNorm * (nowVoxel[1]) + xNorm * yNorm * (nowVoxel[2]); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
				expandedVertex[voxelNum].occupy = true;
			}
		}
	}
}

void ExpandBarrierRegion(SearchVoxelRegion& svr, vertex* expandedVertex) {
	// �������ؿ�
	cout << endl << endl << endl << endl;
	int cnt = 0;
	for (int k = 0; k < svr.height; k++) {
		for (int j = 0; j < svr.width; j++) {
			for (int i = 0; i < svr.length; i++) {
				int voxelNum = (i)+svr.length * (j)+svr.length * svr.width * (k); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
				//cout << voxelNum << "	";
				expandedVertex[voxelNum] = allVertexArr[voxelNum];
				if (allVertexArr[voxelNum].occupy) {
					FillCircleBasePoint(Eigen::Vector3i(i, j, k), expandedVertex, svr);
					cnt++;
				}
			}
		}
	}
	cout << "�ܹ��У�" << pcl_voxel->GetVoxels().size() << "�����أ����б�ռ�õ������У�" << cnt << "��" << endl;
}

bool BreadthFirst(const int len, const vertex& start, const vertex& goal, vector<int>& bfsLine) {
	cout << "len:" << len << endl;
	vertex v{ start };
	bool* vertexUse = (bool*)malloc(sizeof(bool) * len); // ���ж����Ƿ��ѱ�����
	int* vertexSort = (int*)malloc(sizeof(int) * len); // ��������������
	memset(vertexUse, false, sizeof(bool) * len);
	memset(vertexSort, 0, sizeof(int) * len);
	int nowVertexIndex = 0; // �������
	int lastVertex = 0; // ǰһʱ�̶�������
	vertexUse[v.num] = true;
	vertexSort[nowVertexIndex] = v.num; // ��һ���������ر��
	nowVertexIndex++;
	bool findFlag = false;
	int totalLen = 1;
	vector<int> vTotalSortIndex;
	vTotalSortIndex.emplace_back(0);
	//vector<vector<int>> vvAllLine;
	while (1) {
		int temp = nowVertexIndex;
		if (lastVertex == temp) {
			// ��ǰ�����������û�����Ž�
			return false;
		}
		for (int i = lastVertex; i < temp; i++) {
			//cout << lastVertex << "	" << temp << endl;
			v = expandedVertexArr[vertexSort[i]]; // ������ǰ��ȵ��������ؿ�

			if (v.occupy) {
				continue;
			}
			for (int j = 0; j < 6; j++) {
				int nextVoxelNum = v.vVertexNum[j];
				if (nextVoxelNum == -1) {
					// ��ǰ����Ķ�����Ч
					continue;
				}
				else {
					// ��Ч�ڵ�
					if (vertexUse[nextVoxelNum] == true) {
						continue;
					}
					//cout << v.num << " ";
					vertexSort[nowVertexIndex] = nextVoxelNum; // ��ǰ�ڵ��������������
					nowVertexIndex++;
					vertexUse[nextVoxelNum] = true; // ָʾ��ǰ�ڵ���Ϊ����
					//cout << "v.vVertexNum[j]:" << v.vVertexNum[j] << endl;
					if (nextVoxelNum == goal.num) {
						// �ҵ�Ŀ�궥��
						findFlag = true;
						break;
					}
				}
			}
			if (findFlag) {
				break;
			}
			//cout << "���������������" << endl;
			//for (int i = 0; i < nowVertexIndex; i++) {
			//	cout << vertexSort[i] << " ";
			//}
		}
		lastVertex = temp;
		vTotalSortIndex.emplace_back(lastVertex);
		totalLen++;
		if (findFlag) {
			break;
		}
		//if (nowVertexIndex == len) {
		//	break;
		//}
	}
	vTotalSortIndex.swap(vTotalSortIndex);
	cout << "���·������Ϊ��" << totalLen << endl;
	// ���·����Ӧ�����������������
	for (int i = 0; i < vTotalSortIndex.size(); i++) {
		cout << vTotalSortIndex[i] << " ";
	}
	//cout << endl << "���������������" << endl;
	//for (int i = 0; i < len; i++) {
	//	cout << vertexSort[i] << " ";
	//}
	cout << endl;

	// ���ݹ�����Ȳ㼶�����ԭ��·��
	vCurrentTrajectoryVertex = vector<vertex>(vTotalSortIndex.size(), vertex());
	bfsLine = vector<int>(vTotalSortIndex.size(), 0);
	bfsLine[vTotalSortIndex.size() - 1] = goal.num;
	for (int i = vTotalSortIndex.size() - 2; i >= 0; i--) {
		int lastVoxelNum = bfsLine[i + 1];
		int nowFinded = false;
		cout << "vTotalSortIndex[i]:" << vTotalSortIndex[i] << "	vTotalSortIndex[i + 1]:" << vTotalSortIndex[i + 1] << endl;
		for (int j = vTotalSortIndex[i]; j < vTotalSortIndex[i + 1]; j++) {
			vertex nowVertex = expandedVertexArr[vertexSort[j]];
			//cout << "nowVertex:"<< vertexSort[j] << " " << nowVertex.vVertexNum[0] << " " << nowVertex.vVertexNum[1]
			//	<< " " << nowVertex.vVertexNum[2] << " " << nowVertex.vVertexNum[3]
			//	<< " " << nowVertex.vVertexNum[4] << " " << nowVertex.vVertexNum[5] << endl;
			// Ѱ�Ҹ��׽ڵ�
			for (int k = 0; k < 6; k++) {
				if (nowVertex.vVertexNum[k] == lastVoxelNum) {
					//cout << "���ڽڵ�Ϊ��" << vVertex[j].vVertexNum[k] << endl;
					bfsLine[i] = nowVertex.num;
					nowFinded = true;
					vCurrentTrajectoryVertex[i] = nowVertex;
					break;
				}
			}
			if (nowFinded) {
				break;
			}
		}
	}
	cout << endl << "���������������" << endl;
	for (int i = 0; i < bfsLine.size(); i++) {
		cout << bfsLine[i] << " ";
	}

	return true;
}

void UpDateRouteBaseCurrentPointCloud(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
	// �������ػ�ȷ��Ŀ����нⷶΧ
	// ��ʼ�㣨���λ�ã���Ŀ��㣨�ֶ�λ�㣩
	Eigen::Vector3i startVertexVoxel = Eigen::Vector3i(0, 0, 0);
	Eigen::Vector3i endVertexVoxel = Eigen::Vector3i(0, 0, 0);
	v2pInfo.size = pcl_ptr->GetAxisAlignedBoundingBox().GetExtent(); // ���µ�����Ϣ
	v2pInfo.minPoint = pcl_ptr->GetAxisAlignedBoundingBox().GetAxisAlignedBoundingBox().GetBoxPoints()[0]; // 0��Ӧxyz��С�Ķ���
	ConvertPoint2VoxelPos(start, startVertexVoxel); // ���λ��
	//cout << "*******��ʼ���Ӧ�Ķ�����Ϊ��" << startVertexVoxel.transpose() << endl;
	ConvertPoint2VoxelPos(goal, endVertexVoxel); // Ŀ���Ӧ������λ��
	// Ŀ�궥�㲻Ӧ������ʵ��λ�ã���Ӧ������Ĺ۲�λ�á����������Ϸ����л�Բ������    
	endVertexVoxel[2] = endVertexVoxel[2] + 2 * DEVICE_OCCUPYVOXEL;
	cout << "��ʼ���ؿ��Ӧ��������ϵ������:" << startVertexVoxel.transpose() << "	��������Ϊ��" << start.transpose() << endl;
	cout << "Ŀ�����ؿ��Ӧ��������ϵ������:" << endVertexVoxel.transpose() << endl;
	// ���ػ����ӵ���xyz��С���Ǹ��ǽ��б�ţ�����bbox��xyz��С��Ϊ0��
	// �����豸�ߴ磬���ϰ���������ͣ����豸�����Ϊ�ʵ�
	SearchVoxelRegion svr;
	GetSearchRegion(startVertexVoxel, endVertexVoxel, svr); // ȷ��������Χ
	// ��������������ؿ���б�š�����
	vertex startVertex, endVertex;
	VertexNumFromSearchRegion(svr, startVertexVoxel, endVertexVoxel, startVertex, endVertex); // ��������Χ���б��
	expandedVertexArr = (vertex*)malloc(svr.height * svr.width * svr.length * sizeof(vertex));
	ExpandBarrierRegion(svr, expandedVertexArr); // �����ϰ�������
	// �ڵ����л������ͺ�����
	for (int i = 0; i < svr.length; i++) {
		for (int j = 0; j < svr.width; j++) {
			for (int k = 0; k < svr.height; k++) {
				int voxelNum = (i)+svr.length * (j)+svr.length * svr.width * (k); // ���ر�ţ���ԭ�㿪ʼ���㣺x��y��z˳��
				if (expandedVertexArr[voxelNum].occupy) {
					pcl_voxel->AddVoxel(open3d::geometry::Voxel(expandedVertexArr[voxelNum].v, Eigen::Vector3d(255, 0, 0)));
				}
				//else {
				//    pcl_voxel->AddVoxel(open3d::geometry::Voxel(expandedVertexArr[voxelNum].v, Eigen::Vector3d(255, 0, 255)));
				//}
			}
		}
	}
	pcl_voxel->AddVoxel(open3d::geometry::Voxel(startVertexVoxel, Eigen::Vector3d(0, 0, 255)));
	pcl_voxel->AddVoxel(open3d::geometry::Voxel(endVertexVoxel, Eigen::Vector3d(0, 0, 255)));
	//open3d::visualization::DrawGeometries({ pcl_voxel }, "expanded");
	// BFS�����������л�ͼ�����������޻�ͼ���й����������
	vCurrentTrajectory.clear();
	BreadthFirst(svr.height * svr.length * svr.width, startVertex, endVertex, vCurrentTrajectory);
	// �����л���BFS���
	cout << "bfsLine.size():" << vCurrentTrajectory.size() << endl;
	for (int i = 0; i < vCurrentTrajectory.size(); i++) {
		pcl_voxel->AddVoxel(open3d::geometry::Voxel(allVertexArr[vCurrentTrajectory[i]].v, Eigen::Vector3d(0, 255, 0)));
	}
}

bool VertifyRouteWhetherMeetRequire(const int startIndex) {
	int occupyNumX = ceil(DEVICE_WIDTH / VOXEL_SIZE / 2.);
	int occupyNumY = ceil(DEVICE_LENGTH / VOXEL_SIZE / 2.);
	int occupyNumZ = ceil(DEVICE_HEIGHT / VOXEL_SIZE / 2.);
	//for (int num = startIndex; num < startIndex + ROBOT_UPDATE_MINDIS * 2; num++) {
	for (int num = startIndex; num < vCurrentTrajectoryVertex.size(); num++) {
		//if (num >= vCurrentTrajectoryVertex.size()) return true;
		vertex tmpVertex = vCurrentTrajectoryVertex[num];
		cv::Mat adjustedNowGripperPostrue;
		AdjustCameraPosture(tmpVertex.vPcl, objectCenterPos, adjustedNowGripperPostrue); // ��ǰ���ָ��Ŀ��λ��
		vector<Eigen::Vector3d> vDrawLinePoint;
		vector<Eigen::Vector2i> vDrawLineIndex;
		Eigen::Vector3d xOrient(adjustedNowGripperPostrue.at<double>(0, 0), adjustedNowGripperPostrue.at<double>(1, 0), adjustedNowGripperPostrue.at<double>(2, 0));
		Eigen::Vector3d yOrient(adjustedNowGripperPostrue.at<double>(0, 1), adjustedNowGripperPostrue.at<double>(1, 1), adjustedNowGripperPostrue.at<double>(2, 1));
		Eigen::Vector3d zOrient(adjustedNowGripperPostrue.at<double>(0, 2), adjustedNowGripperPostrue.at<double>(1, 2), adjustedNowGripperPostrue.at<double>(2, 2));
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl);
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl + 500 * xOrient);
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl + 800 * yOrient);
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl + 1600 * zOrient);
		//vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
		//vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
		//vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
		//std::shared_ptr<open3d::geometry::LineSet> nowCameraCloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
		//std::shared_ptr<open3d::geometry::LineSet> originandaxiscloud;
		//DrawXYZAxisAtOrient(originandaxiscloud);
		//open3d::visualization::DrawGeometries({pcl_voxel, nowCameraCloud, originandaxiscloud }, "Cam Pose"); // ��ʾ��ǰ�������
		for (int i = -occupyNumX + 1; i < occupyNumX; i++) {
			for (int j = -occupyNumY + 1; j < occupyNumY; j++) {
				for (int k = 0; k < 2 * occupyNumZ; k++) {
					//double distance = (Eigen::Vector3i(i, j, k) - tmpVertex.vPcl).norm();
					//// �����뾶��Χ
					//if (distance > DEVICE_OCCUPYVOXEL + 1) {
					//	continue;
					//}
					// ��ѯ��ǰλ���Ƿ�ռ��
					vector<Eigen::Vector3d> vCheck(1, Eigen::Vector3d(0, 0, 0));
					//vCheck[0] = tmpVertex.vPcl + Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					Eigen::Vector3d nowVoxelPos = Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					vCheck[0] = tmpVertex.vPcl + nowVoxelPos[0] * xOrient + nowVoxelPos[1] * yOrient + nowVoxelPos[2] * zOrient;
					//cout << "��ǰ���ڵ�����λ��Ϊ��" << tmpVertex.vPcl.transpose() << "	��ѯ������Ϊ��" << vCheck[0].transpose() << endl;
					vector<bool> vResult = pcl_voxel->CheckIfIncluded(vCheck); // ռ�����
					//cout << "��ǰ����ռ����Ϣ��" << vResult[0] << endl;
					if (vResult[0]) {
						// ��ռ��
						cout << "��" << num << "���㲻����Ҫ��" << endl;
						return false;
					}

				}
			}
		}
	}
	return true;
}

cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, Eigen::Vector3d& rx, Eigen::Vector3d& ry) {
	double A = rz[0], B = rz[1], C = rz[2];
	if (A != 0) {
		ry = Eigen::Vector3d(-C / A, 0, 1).normalized();
		if (B == 0) {
			rx = Eigen::Vector3d(0, 1, 0);
		}
		else {
			rx = Eigen::Vector3d(1, -(C * C + A * A) / A / B, C / A).normalized();
		}
	}
	else if (A == 0 && B != 0) {
		ry = Eigen::Vector3d(0, -C / B, 1).normalized();
		rx = Eigen::Vector3d(1, 0, 0);
	}
	else {
		ry = Eigen::Vector3d(0, 1, 0);
		rx = Eigen::Vector3d(1, 0, 0);
	}

	cv::Mat rotate = (cv::Mat_<double>(3, 3) <<
		rx[0], ry[0], rz[0],
		rx[1], ry[1], rz[1],
		rx[2], ry[2], rz[2]
		);
	//cout << "rx��" << rx.transpose() << endl;
	//cout << "ry��" << ry.transpose() << endl;
	//cout << "rz��" << rz.transpose() << endl;
	//cout << "��ǰ�����ת����Ϊ��" << rotate << "�Ƿ�Ϊ��ת���󣿣�"<< isRotationMatrix(rotate) << endl;

	return rotate;
}

//cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, const Eigen::Vector3d& rx, const Eigen::Vector3d& ry, Eigen::Vector3d& rxAdjusted, Eigen::Vector3d& ryAdjusted) {
//	// ����rz����ʹrx��ry��rz����
//}

void AdjustCameraPosture(const Eigen::Vector3d& nowPose, const Eigen::Vector3d& objPos, cv::Mat& adjustedR) {
	// ���Rz����
	Eigen::Vector3d nowCameraRz = CalcVectorAB(nowPose, objPos);
	//cout << "��ǰ���Ӧ��ָ��ķ���Ϊ��" << nowCameraRz.transpose() << endl;
	// ����Rz����Rx��Ry����ϳ���ת����
	Eigen::Vector3d nowCameraRx, nowCameraRy;
	cv::Mat nowCameraRotate = CalcRxRyBaseRz(nowCameraRz, nowCameraRx, nowCameraRy);
	adjustedR = nowCameraRotate({ 0,0,3,3 });
}

void AdjustCameraPosture(const string& nowGripperPose, const Eigen::Vector3d& objPos, string& adjustedGripperCmd, cv::Mat& adjustedR) {
	// ����string
	cv::Mat nowCameraRT, nowGripperRT;
	ConvertString2Mat44(nowGripperPose, 1, nowCameraRT); // �ַ�ת������6D����
	ConvertString2Mat44(nowGripperPose, 0, nowGripperRT); // �ַ�ת������6D����
	//cout << "���λ��Ϊ��" << nowCameraRT << endl;
	//cout << "��е��ĩ��λ��Ϊ��" << nowGripperRT << endl;
	//cout << "���λ�˼����е��ĩ��λ��Ϊ��" << nowCameraRT * H_Camera2Gripper.inv() << endl;
	// ���Rz����
	Eigen::Vector3d nowCameraRz = CalcVectorAB(
		Eigen::Vector3d(nowCameraRT.at<double>(0, 3), nowCameraRT.at<double>(1, 3), nowCameraRT.at<double>(2, 3)),
		objPos
	);
	//cout << "��ǰ���Ӧ��ָ��ķ���Ϊ��" << nowCameraRz.transpose() << endl;
	// ����Rz����Rx��Ry����ϳ���ת����
	Eigen::Vector3d nowCameraRx, nowCameraRy;
	cv::Mat nowCameraRotate = CalcRxRyBaseRz(nowCameraRz, nowCameraRx, nowCameraRy);
	// �����̬ת��е��ĩ��
	cv::Mat nowCameraT = nowCameraRT * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	//cout << "��е��ĩ��λ��Ϊ��" << nowCameraT.t() << endl;
	//cout << "���λ��1Ϊ��" << ((H_Camera2Gripper.inv() * nowGripperRT.inv()).inv() * (Mat_<double>(4, 1) << 0, 0, 0, 1)).t() << endl;
	//cout << "���λ��2Ϊ��" << (nowGripperRT * H_Camera2Gripper * (Mat_<double>(4, 1) << 0, 0, 0, 1)).t() << endl;
	cv::Mat adjustedCameraRT = R_T2RT(nowCameraRotate, nowCameraT); // �ϲ�R��T
	//cout << "Camera2Gripper���棺" << H_Camera2Gripper.inv() << endl;
	cv::Mat adjustedGripper = adjustedCameraRT * H_Camera2Gripper.inv(); // ���תGipper
	adjustedR = adjustedGripper({ 0,0,3,3 }); // �������ϵת������е��ĩ�˶�Ӧ����ת����
	//cout << "�������е��ĩ����̬Ϊ��" << adjustedGripper << endl;
	ConvertMat2String(adjustedGripper, adjustedGripperCmd); // Matת��е��ָ��
	//cout << "�������е��ĩ����̬Ϊ��" << adjustedGripperCmd << endl;
}

Eigen::Vector3d CalcVectorAB(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	return (b - a).normalized();
}

void ConvertString2Mat44(const string& str, const bool gripper2cam, cv::Mat& m44) {
	vector<double> v; // 79.4 -446.7 357.6 179 -027 179
	AnalysisString26D(str, v); // A+0794-4467+3576+179-027+179BC ---> 79.4 -446.7 357.6 179 -027 179
	cv::Mat euler = (cv::Mat_<double>(1, 3) << v[3], v[4], v[5]);
	cv::Mat rotate = eulerAngleToRotatedMatrix(euler, "xyz");
	cv::Mat translate = (cv::Mat_<double>(3, 1) << v[0], v[1], v[2]);
	cv::Mat m = R_T2RT(rotate, translate); // 4*4��תƽ�ƾ���Gripper2base
	if (gripper2cam) {
		// ��е��ĩ������ϵת�������ϵ
		m44 = m * H_Camera2Gripper;
	}
	else {
		m44 = m.clone();
	}

	return;
}

/// <summary>
/// д������
/// </summary>
/// <param name="v"></param>
/// <param name="trunc"></param>
void WriteTrajectory2File(const vector<vertex>& v, fstream& fs) {
	fs << "��i����	���ر��	ռ����Ϣ	�ռ�λ��	����λ��" << endl;
	for (int i = 0; i < v.size(); i++) {
		fs << i << " " << v[i].num << "	" << v[i].occupy << "	" << v[i].v.transpose() << "	" << v[i].vPcl.transpose() << endl;
	}
	fs.close();

	return;
}


bool VertifyRouteWhetherSatisfy(const vector<Eigen::Vector3d>& vPos, const Eigen::Vector3d& point, double dist) {
	int occupyNumX = ceil(DEVICE_WIDTH / VOXEL_SIZE / 2.);
	int occupyNumY = ceil(DEVICE_LENGTH / VOXEL_SIZE / 2.);
	int occupyNumZ = ceil(DEVICE_HEIGHT / VOXEL_SIZE / 2.);

	for (int num = 0; num < vPos.size(); num++) {
		if ((vPos[num] - point).norm() > dist) continue;
		// ��ȡ��ǰ��е������
		string currStrPose = robotArmHandle->ReadRobotArmPosString();
		// ����string
		cv::Mat curr6DMatPose;
		AnalysisString26D(currStrPose, curr6DMatPose);
		cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
		Eigen::Vector3d xOrient(H_gripper2base.at<double>(0, 0), H_gripper2base.at<double>(1, 0), H_gripper2base.at<double>(2, 0));
		Eigen::Vector3d yOrient(H_gripper2base.at<double>(0, 1), H_gripper2base.at<double>(1, 1), H_gripper2base.at<double>(2, 1));
		Eigen::Vector3d zOrient(H_gripper2base.at<double>(0, 2), H_gripper2base.at<double>(1, 2), H_gripper2base.at<double>(2, 2));
		Eigen::Vector3d currPos = vPos[num];
		for (int i = -occupyNumX + 1; i < occupyNumX; i++) {
			for (int j = -occupyNumY + 1; j < occupyNumY; j++) {
				for (int k = -occupyNumZ + 1; k < occupyNumZ; k++) {
					//double distance = (Eigen::Vector3i(i, j, k) - tmpVertex.vPcl).norm();
					//// �����뾶��Χ
					//if (distance > DEVICE_OCCUPYVOXEL + 1) {
					//	continue;
					//}
					// ��ѯ��ǰλ���Ƿ�ռ��
					vector<Eigen::Vector3d> vCheck(1, Eigen::Vector3d(0, 0, 0));
					//vCheck[0] = tmpVertex.vPcl + Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					Eigen::Vector3d nowVoxelPos = Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					//vCheck[0] = currPos + nowVoxelPos[0] * xOrient + nowVoxelPos[1] * yOrient + nowVoxelPos[2] * zOrient;
					vCheck[0] = currPos + DEVICE_HEIGHT * zOrient / 2 + nowVoxelPos[0] * xOrient + nowVoxelPos[1] * yOrient + nowVoxelPos[2] * zOrient;
					//vCheck[0] = currPos + nowVoxelPos;
					//cout << "��ǰ���ڵ�����λ��Ϊ��" << tmpVertex.vPcl.transpose() << "	��ѯ������Ϊ��" << vCheck[0].transpose() << endl;
					vector<bool> vResult = pcl_voxel->CheckIfIncluded(vCheck); // ռ�����
					//cout << "��ǰ����ռ����Ϣ��" << vResult[0] << endl;
					if (vResult[0]) {
						// ��ռ��
						//cout << "��" << num << "���㲻����Ҫ��" << endl;
						return false;
					}

				}
			}
		}
	}
	return true;
}

void UpdateRealTimePointCloud() {
	string saveRealTimePosFilePath_Route = "./imgs/routeplaning/"; // ·���滮�пؽ׶Σ��ļ�·��λ�ã�ʵʱͼƬ��6D����
	CreateDir(saveRealTimePosFilePath_Route);
	//while (!existUpdateSubThread) {
	static int robotMoveCnt = 0;
	while (!agi._move2goal) {
		//Sleep(20);
		if (vAGVMovePosition.size() == 0) continue;

		// �̶�����ROBOT_UPDATE_MINDIS����3D������Ϣ������-�����ػ�-��·���Ƿ����ã������ã����±��BFS����·�������ã������ߣ�
		cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
		cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);//640x480
		if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
			long t1 = GetTickCount();
			//cout << "��ȡ������ͼƬ�ɹ���" << endl;
			cv::Mat colorImageDup, depthImageDup;
			flip(colorImage, colorImageDup, 1);
			flip(depthImage, depthImageDup, 1);

			// ��ȡ��ǰ��е������
			string currStrPose = robotArmHandle->ReadRobotArmPosString();
			// ���µ�ǰʵʱͼƬ��Ϣ
			//mutWritePos.lock();
			cameraImgInfo currentDepthImgInfo;
			UpdateRealTimePosInfo(colorImageDup, depthImageDup, currStrPose, currentDepthImgInfo);
			//mutWritePos.unlock();

			// �ع���ǰ3D������ֻ��ĩ��distance��Χ��(mm)�ĵ㣬����������
			//double distance = 800.;
			double distance = 10000.;
			// ����ǰ��ͼҲ��ͶӰ��3D�ռ� 
			std::shared_ptr<open3d::geometry::PointCloud> tmpNowPointCloud = std::make_shared<open3d::geometry::PointCloud>();
			reconstructInfo ri;
			*tmpNowPointCloud = *ReconstructFromOneImg(currentDepthImgInfo, ri);
			long t2 = GetTickCount();

			// ����ǰAGV״̬�����е�ת������ʼ״̬��
			//Eigen::Vector3d diff = Eigen::Vector3d{
			//	agi._currAgvPos[0] - agi._oriAgvPos[0],
			//	agi._currAgvPos[1] - agi._oriAgvPos[1],
			//	agi._currAgvPos[2] - agi._oriAgvPos[2]
			//};
			Eigen::Vector3d diff = agi._currAgvPos - agi._oriAgvPos;
			//Eigen::Vector3d diff = agi._oriAgvPos - agi._currAgvPos;
			for (int pd = 0; pd < tmpNowPointCloud->points_.size(); pd++) {
				Eigen::Vector3d convertPoint = AGVMove2ArmPos(diff, tmpNowPointCloud->points_[pd], true, agi._oriAgvPos[2]);
				tmpNowPointCloud->points_[pd] = convertPoint;
			}

			//vector<Eigen::Vector3d> vArmMovePositionRTed(vArmMovePosition.size());
			//for (int i = 0; i < vArmMovePosition.size(); i++) {
			//	vArmMovePositionRTed[i] = AGVMove2ArmPos(diff, vArmMovePosition[i], false);
			//}
			//Eigen::Vector3d goalInfo = agi._oriAgvPos;
			//Eigen::Vector3d goalInfoRT = AGVMove2ArmPos(diff, goalInfo, false);
			long t3 = GetTickCount();

			//open3d::visualization::DrawGeometries({ pclRealTime_ptr, originandaxis_cloud }, "Point Cloud");
			// ���ػ����ӵ���xyz��С���Ǹ��ǽ��б�ţ�����bbox��xyz��С��Ϊ0��
			pcl_voxel = open3d::geometry::VoxelGrid::CreateFromPointCloud(open3d::geometry::PointCloud(tmpNowPointCloud->points_), VOXEL_SIZE);
			long t4 = GetTickCount();

			vector<Eigen::Vector3d> vDrawLinePoint; // ���������
			vector<Eigen::Vector2i> vDrawLineIndex; // ֱ�ߵ��������ж�Ӧ������
			std::shared_ptr<open3d::geometry::LineSet> totalLine = std::make_shared<open3d::geometry::LineSet>();
			// ����AGV�켣ת������е���˶��켣
			for (int point = 1; point < vArmMovePosition.size(); point++) {
				if (!vTest.empty() && vTest.front() <= point) {
					// ����������
					cout << vTest.front() << endl;
					vTest.erase(vTest.begin());
					continue;
				}
				//cout << vArmMovePosition[point - 1].transpose() << "	" << vArmMovePosition[point].transpose() << endl;
				vDrawLinePoint.clear();
				vDrawLineIndex.clear();
				vDrawLinePoint.emplace_back(vArmMovePosition[point]);
				vDrawLinePoint.emplace_back(vArmMovePosition[point - 1]);
				vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
				*totalLine += *std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
				//open3d::visualization::DrawGeometries({ pcl_voxel, line }, "Voxeled");

			}
			vDrawLinePoint.clear();
			vDrawLineIndex.clear();
			vDrawLinePoint.emplace_back(Eigen::Vector3d{ 0., 0., 0. });
			vDrawLinePoint.emplace_back(agi._worldGoalPos);
			//vDrawLinePoint.emplace_back(goalInfoRT);
			vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
			*totalLine += *std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
			std::shared_ptr<open3d::geometry::LineSet> originandaxis_cloud = DrawXYZAxisAtOrient(drawAxisInfo{});
			*totalLine += *originandaxis_cloud;
			//open3d::visualization::DrawGeometries({ tmpNowPointCloud, totalLine, originandaxis_cloud }, "Voxeled");

			//string dirPath = saveRealTimePosFilePath_Route + to_string(robotMoveCnt);
			//open3d::io::WritePointCloudToPLY(dirPath + "pointcloud.ply", *tmpNowPointCloud);
			////open3d::io::WritePointCloudToPLY(dirPath + "_line.ply", totalLine);
			//robotMoveCnt++;

			//{
			//	// ��������
			//	string dirPath = saveRealTimePosFilePath_Route + to_string(robotMoveCnt) + "/";
			//	CreateDir(dirPath);
			//	string filePath = dirPath + "pos.txt";
			//	// ��ʵʱ����д�뵽�ļ���
			//	IOFile iof(filePath.c_str());
			//	// ����ļ����ݣ�д������
			//	iof.WriteString2File(currStrPose, 1);
			//	// ��ʵʱ�켣д�뵽�ļ���
			//	filePath = dirPath + "trajectory.txt";
			//	// ����RGBD����
			//	string colorSavePath = dirPath + "rgb.jpg";
			//	string depthSavePath = dirPath + "depth.png";
			//	imwrite(colorSavePath, colorImageDup);
			//	imwrite(depthSavePath, depthImageDup); // PNG16
			//	// �������
			//	open3d::io::WritePointCloudToPLY(dirPath + "pointcloud.ply", *tmpNowPointCloud);
			//	// ����ļ����ݣ�д�뵱ǰ�켣����
			//	iof = IOFile(filePath.c_str());
			//	//WriteTrajectory2File(vCurrentTrajectoryVertex, iof);
			//}
			//// �ж�֮ǰ·��vCurrentTrajectoryVertex�Ƿ����㵱ǰ����Ҫ��
			//vector<Eigen::Vector3d> convertedPos;
			//ConvertOriMovePos2Curr(vArmMovePosition, convertedPos);
			// ���µ�ǰ·����Ӧ����sssssssss
			Eigen::Vector3d rotatedCamera = AGVMove2ArmPos(diff, currentDepthImgInfo._CameraPosition, true, agi._oriAgvPos[2]);
			int vertifyRet = VertifyRouteWhetherSatisfy(vArmMovePosition, rotatedCamera, distance);
			long t5 = GetTickCount();
			cout << "   ��ǰ�˶��켣" << (vertifyRet ? "����" : "������") << "Ҫ��!"
				<< "(" << t2 - t1 << "," << t3 - t2 << "," << t4 - t3 << "," << t5 - t4 << ")" << endl;

			if (!vertifyRet) {
				agi._move2goal = 2;
				cout << agi._move2goal << "-----------------------------------------" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl << flush;
				break;
			}
			//robotMoveCnt++;
		}
		// Sleep(2);
	}
	agi._move2goal = 3;
}