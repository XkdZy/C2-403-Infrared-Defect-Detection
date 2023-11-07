#include "AgvControl.h"
#include "Seer.h"

/// <summary>
/// 调整AGV指向目标位姿（以mm为单位，相对AGV世界坐标系）
/// </summary>
/// <param name="goalPos">目标在AGV下位姿</param>
/// <returns>保留，调整是否成功</returns>
bool AdjustAGV2Goal(const Eigen::Vector3d& goalPos) {
    // 读取当前agv位姿
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    // 计算目标朝向
    double goalTheta = atan2(goalPos[1] - currAmrli._y, goalPos[0] - currAmrli._x);
    // 计算需要旋转的角度
    double rotTheta = goalTheta - currAmrli._angle;  
    cout << "goal dir:" << goalTheta << "  curr agv dir:" << currAmrli._angle << "  need rotate theta:" << rotTheta << endl;
    // 旋转AGV指向目标
    AMRRotation(abs(rotTheta), rotTheta > 0 ? 1 : -1, m_SockClient06);
    while (1) {
        int navigation_status = AMRnavigation_status(m_SockClient04);
        if (navigation_status == 4 || navigation_status == 0) {
            //运动完成||无导航   进行下一步运动
            break;
        }
    }

    return true;
}

/// <summary>
/// 控制AGV自动导航到一个目标点（以mm为单位，相对AGV世界坐标系）
/// </summary>
/// <param name="goal">目标点</param>
/// <param name="towardGoal">是否朝向目标</param>
/// <returns>保留，运动是否成功</returns>
bool MoveAGV2OneGoal(const Eigen::Vector3d& goal, bool towardGoal, const Eigen::Vector3d towardPos) {
    AMRRobotNavigation(goal[0] / 1000., goal[1] / 1000., m_SockClient06);
    while (1) {
        int navigation_status = AMRnavigation_status(m_SockClient04);
        //cout << "curr status:" << navigation_status << endl;
        if (navigation_status == 4 || navigation_status == 0) {
            //运动完成||无导航   进行下一步运动
            break;
        }
    }
    AdjustAGV2Goal(towardPos);
    return true;
}