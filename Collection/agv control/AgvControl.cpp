#include "AgvControl.h"
#include "Seer.h"

/// <summary>
/// ����AGVָ��Ŀ��λ�ˣ���mmΪ��λ�����AGV��������ϵ��
/// </summary>
/// <param name="goalPos">Ŀ����AGV��λ��</param>
/// <returns>�����������Ƿ�ɹ�</returns>
bool AdjustAGV2Goal(const Eigen::Vector3d& goalPos) {
    // ��ȡ��ǰagvλ��
    AMRLocalInfo currAmrli;
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    // ����Ŀ�곯��
    double goalTheta = atan2(goalPos[1] - currAmrli._y, goalPos[0] - currAmrli._x);
    // ������Ҫ��ת�ĽǶ�
    double rotTheta = goalTheta - currAmrli._angle;  
    cout << "goal dir:" << goalTheta << "  curr agv dir:" << currAmrli._angle << "  need rotate theta:" << rotTheta << endl;
    // ��תAGVָ��Ŀ��
    AMRRotation(abs(rotTheta), rotTheta > 0 ? 1 : -1, m_SockClient06);
    while (1) {
        int navigation_status = AMRnavigation_status(m_SockClient04);
        if (navigation_status == 4 || navigation_status == 0) {
            //�˶����||�޵���   ������һ���˶�
            break;
        }
    }

    return true;
}

/// <summary>
/// ����AGV�Զ�������һ��Ŀ��㣨��mmΪ��λ�����AGV��������ϵ��
/// </summary>
/// <param name="goal">Ŀ���</param>
/// <param name="towardGoal">�Ƿ���Ŀ��</param>
/// <returns>�������˶��Ƿ�ɹ�</returns>
bool MoveAGV2OneGoal(const Eigen::Vector3d& goal, bool towardGoal, const Eigen::Vector3d towardPos) {
    AMRRobotNavigation(goal[0] / 1000., goal[1] / 1000., m_SockClient06);
    while (1) {
        int navigation_status = AMRnavigation_status(m_SockClient04);
        //cout << "curr status:" << navigation_status << endl;
        if (navigation_status == 4 || navigation_status == 0) {
            //�˶����||�޵���   ������һ���˶�
            break;
        }
    }
    AdjustAGV2Goal(towardPos);
    return true;
}