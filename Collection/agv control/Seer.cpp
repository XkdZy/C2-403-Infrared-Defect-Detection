#include "Seer.h"

char buffersss[50048];

SOCKET m_SockClient10;//�˿�19210  ����API DO���  ��բ��
SOCKET m_SockClient04;//�˿�19204  ��ѯ
SOCKET m_SockClient05;//�˿�19205  
SOCKET m_SockClient06;//�˿�19206  ����

void InitSeerAGV() {
    std::cout << "Init Seer AGV start..." << std::endl;
    InitInternet();
    if (ConnectSeer(19210, m_SockClient10)) {
        std::cout << "19210���ӳɹ�..." << endl;
    }
    else {
        std::cout << "19210����ʧ��..." << endl;
    }
    if (ConnectSeer(19204, m_SockClient04)) {
        std::cout << "19204���ӳɹ�..." << endl;
    }
    else {
        std::cout << "19204����ʧ��..." << endl;
    }
    if (ConnectSeer(19205, m_SockClient05)) {
        std::cout << "19205���ӳɹ�..." << endl;
    }
    else {
        std::cout << "19205����ʧ��..." << endl;
    }
    if (ConnectSeer(19206, m_SockClient06)) {
        std::cout << "19206���ӳɹ�..." << endl;
    }
    else {
        cout << "19206����ʧ��..." << endl;
    }
    AMRUnlock(0, 1, m_SockClient10);//��բ��
    std::cout << "Init Seer AGV end!" << std::endl;
}

void delay_ms(int ms,int x)
{
    clock_t start = clock();
    while (clock() - start < ms);
}

void InitInternet() {
    WSADATA wsd; // ��ʼ�����绷��
    if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) { //ʹ��2.2�汾��Socket
        cout << "INITIAL  FAILED!" << endl;
        system("pause");
    }
    return;
}

bool ConnectSeer(const u_short port, SOCKET& client) {
    sockaddr_in clientaddr;
    clientaddr.sin_family = AF_INET;//���÷�������ַ	����
    clientaddr.sin_port = htons(port);//���÷������˿ں�	
    clientaddr.sin_addr.S_un.S_addr = inet_addr(SEERCONTROLIP);
    client = socket(AF_INET, SOCK_STREAM, 0);
    int result = connect(client, (sockaddr*)&clientaddr, sizeof(clientaddr));//���ӳ�ʱ 
    return result == 0;
}

char* RobotState2SeerFrame(const uint16_t m_number, const uint16_t m_type) {
    char* s_data = new char[16];
    uint8_t m_header = 0x5A;
    uint8_t m_version = 0x01;
    sprintf(
        s_data, 
        "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
        m_header,
        m_version,
        (m_number / 256) % 256, m_number % 256,
        0, 0, 0, 0,
        (m_type / 256) % 256, m_type % 256,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    );
    return s_data;
}

char* RobotNavigationJSON2SeerFrame(const uint16_t m_number, const uint16_t m_type, const char* json) {
        char* s_date = new char[1000];
        uint8_t m_header = 0x5A;
        uint8_t m_version = 0x01;
        uint8_t m_reserved[6] = { 0 };
        uint32_t m_length = 0;
        
        m_length = strlen(json);
        sprintf(
            s_date, 
            "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%s",
            m_header,
            m_version,
            (m_number / 256) % 256, m_number % 256,
            (m_length / 256 / 256 / 256) % 256, (m_length / 256 / 256) % 256, (m_length / 256) % 256, m_length % 256,
            (m_type / 256) % 256, m_type % 256,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            json
        );
        return s_date;
}

char* AMRUnlock(const double id, const bool status, SOCKET& client) {
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    ////��������
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddBoolToObject(root, "status", status);
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(6001, 6001, json_data);

    send(client, s_data, strlen(json_data)+16, 0);

    return ReadAMRRespond(client);
}


char* AMRLibration(const double dist, const double vx, const double vy, SOCKET& client, const bool mode) {
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    ////��������
    cJSON_AddNumberToObject(root, "dist", dist);
    cJSON_AddNumberToObject(root, "vx", vx);
    cJSON_AddNumberToObject(root, "vy", vy);
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3055, 3055, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);
    char* result = ReadAMRRespond(client);
    while (1) {
        int navigation_status = AMRnavigation_status(m_SockClient04);
        //cout << "curr status:" << navigation_status << endl;
        if (navigation_status == 4 || navigation_status == 0) {
            //�˶����||�޵���   ������һ���˶�
            break;
        }
    }
    return result;
}

void AMRRotation(const double angle, const double vw, SOCKET& client) {//�޷���ֵ
    AMRLocalInfo amrliStart;
    RequestAMRLocal(amrliStart);
    //cout << amrliStart._x << "   " << amrliStart._y << "  " << amrliStart._angle << endl;
    AMRLocalInfo amrliGoal = amrliStart;
    amrliGoal._angle += vw > 0 ? angle : -angle;

    cJSON* root = cJSON_CreateObject();//json���ݶ�
    ////��������
    cJSON_AddNumberToObject(root, "angle", angle);
    cJSON_AddNumberToObject(root, "vw", vw);
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3056, 3056, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);
    ReadAMRRespond(client);
    AMRLocalInfo curr = amrliGoal;
    while (1) {
        int navigation_status = AMRnavigation_status(m_SockClient04);
        //cout << "curr status:" << navigation_status << endl;
        if (navigation_status == 4 || navigation_status == 0) {
            //�˶����||�޵���   ������һ���˶�
            break;
        }
    }

    return;
}
bool WhetherAMRMove2Goal(const SOCKET& client, AMRLocalInfo& goal, AMRLocalInfo& last, int which) {
    AMRLocalInfo curr;
    RequestAMRLocal(curr);

    //cout << goal._angle << "    " << curr._angle << "    " << curr._angle << endl;
    if (which == 2) {
        //return abs(curr._angle - goal._angle) < 0.01 || abs(curr._angle - last._angle) < 1.e-8;
        return abs(curr._angle - last._angle) < 1.e-8;
    }
    else if (which == 1) {
        return abs(curr._y - goal._y) < 0.01 || abs(curr._y - last._y) < 1.e-8;
    }
    else {
        return abs(curr._x - goal._x) < 0.01 || abs(curr._x - last._x) < 1.e-8;
    }
}


void AMRRobotNavigation(const double x, const double y, const double theta, SOCKET& client) {
    // ������� { }
    cJSON* script_args = cJSON_CreateObject();
    cJSON_AddItemToObject(script_args, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(script_args, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(script_args, "coordinate", cJSON_CreateString("world"));
    cJSON_AddItemToObject(script_args, "theta", cJSON_CreateNumber(theta));
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    cJSON_AddItemToObject(root, "script_name", cJSON_CreateString("syspy/goPath.py"));
    cJSON_AddItemToObject(root, "script_args", script_args);
    cJSON_AddItemToObject(root, "operation", cJSON_CreateString("Script"));
    cJSON_AddItemToObject(root, "id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "source_id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "task_id", cJSON_CreateString("12344321"));
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3051, 3051, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);
    ReadAMRRespond(client);
    return;
}


void AMRRobotNavigation(const double x, const double y, SOCKET& client) {
    // ������� { }
    cJSON* script_args = cJSON_CreateObject();
    cJSON_AddItemToObject(script_args, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(script_args, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(script_args, "coordinate", cJSON_CreateString("world"));
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    cJSON_AddItemToObject(root, "script_name", cJSON_CreateString("syspy/goPath.py"));
    cJSON_AddItemToObject(root, "script_args", script_args);
    cJSON_AddItemToObject(root, "operation", cJSON_CreateString("Script"));
    cJSON_AddItemToObject(root, "id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "source_id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "task_id", cJSON_CreateString("12344321"));
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3051, 3051, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);
    ReadAMRRespond(client);
    return;
}


void AMRgofree(const double x, const double y, const double theta, SOCKET& client) {
    // ������� { }
    cJSON* script_args = cJSON_CreateObject();
    cJSON_AddItemToObject(script_args, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(script_args, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(script_args, "theta", cJSON_CreateNumber(theta));
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    cJSON_AddItemToObject(root, "id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "freeGo", script_args);
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3051, 3051, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);
    ReadAMRRespond(client);
    return;
}

char* AMRstop_Navigation(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(3003, 3003);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}





////////////////////////////////////////////////////��
char* ReadAMRRespond(const SOCKET& client) {


    //char* buffer = new char[50048];//malloc
    memset(buffersss, '\0', sizeof(buffersss));
    while (1) {
        int num;
        //Sleep(1);
        num = recv(client, buffersss, 50048, 0); //���տͻ��˷��͹���������	
        if (num > 0) {
            //cout << "����֡���ȣ�" << num << endl;
            //for (int i = 0; i < num; i++) {
            //    cout << (char)buffersss[i] << " ";
            //}
            //cout << "������Ϣ��" << string(buffersss) << endl;

            return buffersss;
        }
        //else
        //    cout << " ���������� " << endl;
    }

    return nullptr;
}
int ReadAMRRespond(const SOCKET& client, char*& data) {
    //char* buffer = new char[50048];//malloc
    memset(buffersss, '\0', sizeof(buffersss));
    while (1) {
        int num;
        num = recv(client, buffersss, 50048, 0); //���տͻ��˷��͹���������	
        if (num > 0) {
            //cout << "����֡���ȣ�" << num << endl;
            //for (int i = 0; i < num; i++) {
            //    cout << (char)buffersss[i] << " ";
            //}
            //cout << "������Ϣ��" << string(buffersss) << endl;
            data = buffersss;
            return num;
        }
        //else
        //    cout << " ���������� " << endl;
    }

    return 0;
}

char* ReadAMRPostion(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1004, 1004);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

char* ReadAMRLaser_res(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1009, 1009);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}
int ReadAMRLaser_res(const SOCKET& client, char*& data, bool multi) {
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    ////��������
    cJSON_AddBoolToObject(root, "return_beams3D", multi);
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(1009, 1009, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return ReadAMRRespond(client, data);
}

char* ReadAMRnavigation(const SOCKET& client) {
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    ////��������
    cJSON_AddBoolToObject(root, "simple", true);
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(1020, 1020, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return ReadAMRRespond(client);
}



int AMRnavigation_status(SOCKET& client) {
    //cout << "---------------------------------23-1" << endl;
    char* jsonStr = ReadAMRnavigation(client);
    jsonStr = jsonStr + 16;
    //cout << "��ȡ����״̬:" << jsonStr << endl;
    // ����ȡ����json�ַ���ת����json����ָ��
    //cout << "---------------------------------23-12" << endl;
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        return 0;
    }
    //cout << "---------------------------------23-2" << endl;
    //free(jsonStr);//08162600008
    /*************** ���� json***************/
    cJSON* item = NULL;
    int x=-1;
    item = cJSON_GetObjectItem(root, "task_status");
    if (item != NULL) {	// �Ϸ��Լ��
        if (item->type == cJSON_Number) {		// �ж��ǲ�������
            x = item->valueint;			// ��ȡֵ
            //printf("x = %f\n", x);
        }
    }
    //cout << "---------------------------------23-3" << endl;
    cJSON_Delete(root);
    //cJSON_Delete(item);
    //cout << "---------------------------------23-4" << endl;
    //free(jsonStr);  
    return x;
}

char* ReadAMRInfo(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1000, 1000);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

int ReadAMRInfo(const SOCKET& client, char*& data) {
    char* s_data = RobotState2SeerFrame(1000, 1000);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client, data);
}

char* ReadAMblocked(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1006, 1006);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

int AMRblocked_status(SOCKET& client) {
    char* jsonStr = ReadAMblocked(client);
    jsonStr = jsonStr + 16;
    //cout << "��ȡ����״̬:" << jsonStr << endl;
    // ����ȡ����json�ַ���ת����json����ָ��
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //delete[] jsonStr;
        return 0;
    }
    //free(jsonStr);//08162600008
    
    /*************** ���� json***************/
    cJSON* item = NULL;
    int x = 0;
    item = cJSON_GetObjectItem(root, "blocked");
    if (item != NULL) {	// �Ϸ��Լ��
        if (item->type == cJSON_True) {		// �ж��ǲ�������
            x = 1;			// ��ȡֵ
            //printf("x = %f\n", x);
        }
    }
    cJSON_Delete(root);
    //delete[] jsonStr;
    return x;
}


float AMRnow_position(const string ss, SOCKET& client) {
    char* jsonStr = ReadAMRPostion(client);
    jsonStr = jsonStr + 16;
    //cout << "��ȡx��y����:" << jsonStr << endl;
    // ����ȡ����json�ַ���ת����json����ָ��
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //free(jsonStr);
        //delete[] jsonStr;
        return 0;
    }
    //free(jsonStr);//08162600008
    /*************** ���� json***************/
    cJSON* item = NULL;
    if (ss == "x") {
        float x;
        item = cJSON_GetObjectItem(root, "x");
        if (item != NULL) {	// �Ϸ��Լ��
            if (item->type == cJSON_Number) {		// �ж��ǲ�������
                x = item->valuedouble;			// ��ȡֵ
                //printf("��ȡx = %f\n", x);
            }
        }
        cJSON_Delete(root);
        //free(jsonStr);  
        //delete[] jsonStr;
        return x;
    }
    if (ss == "y") {
        float y;
        item = cJSON_GetObjectItem(root, "y");
        if (item != NULL) {	// �Ϸ��Լ��
            if (item->type == cJSON_Number) {		// �ж��ǲ�������
                y = item->valuedouble;			// ��ȡֵ
                //printf("��ȡy = %f\n", y);
            }
        }
        cJSON_Delete(root);
        //free(jsonStr);
        //delete[] jsonStr;
        return y;
    }
    
    if (ss == "angle") {
        float angle;
        item = cJSON_GetObjectItem(root, "angle");
        if (item != NULL) {	// �Ϸ��Լ��
            if (item->type == cJSON_Number) {		// �ж��ǲ�������
                angle = item->valuedouble;			// ��ȡֵ
                //printf("��ȡangle = %f\n", angle);
            }
        }
        cJSON_Delete(root);
        //free(jsonStr);
        //delete[] jsonStr;
        return angle;
    }
    //delete[] jsonStr;
    return -1;

}

//char* QueryRobotLaserData(bool multi) {
//    cJSON* root = cJSON_CreateObject();//json���ݶ�
//    ////��������
//    cJSON_AddNumberToObject(root, "return_beams3D", multi);
//    //��������
//    char* json_data = cJSON_PrintUnformatted(root);
//    cJSON_Delete(root);
//    printf("%s\n", json_data);
//    char* s_data = RobotNavigationJSON2SeerFrame(1009, 1009, json_data);
//    send(client, s_data, strlen(json_data) + 16, 0);
//}

void RequestAMRLocal(AMRLocalInfo& amrli) {
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(1004, 1004, json_data);
    send(m_SockClient04, s_data, strlen(json_data) + 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient04, r_data);
    string rStr1;
    for (int i = 0; i < num; i++) {
        //cout << r_data[i] << " ";
        rStr1.append(1, r_data[i]);
    }
    //cout << "read info:" << rStr1 << endl;
    //string rStr2 = r_data;
    //cout << "read info:" << rStr2 << endl;
    int idxSta = rStr1.find("angle"); // a:find index
    int idxEnd = rStr1.find(',');
    amrli._angle = stold(rStr1.substr(idxSta + 7, idxEnd - idxSta - 7));
    //cout << "idxSta:" << idxSta << "    idxEnd:" << idxEnd << " val:" << amrli._angle << endl;
    idxSta = rStr1.find("\"x\""); // "x:find index
    idxEnd = rStr1.find(',');
    amrli._x = stold(rStr1.substr(idxSta + 4, idxEnd - idxSta - 4));
    //cout << "idxSta:" << idxSta << "    idxEnd:" << idxEnd << " val:" << amrli._x << endl;
    idxSta = rStr1.find("\"y\""); // "y:find index
    idxEnd = rStr1.find(',');
    amrli._y = stold(rStr1.substr(idxSta + 4, idxEnd - idxSta - 4));
    //cout << "idxSta:" << idxSta << "    idxEnd:" << idxEnd << " val:" << amrli._y << endl;
}


void AMRStartCreateMap(int slam_tpye, bool real_time, int screen_width, int screen_height) {
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_AddNumberToObject(root, "slam_tpye", slam_tpye);
    cJSON_Delete(root);
    char* s_data = RobotNavigationJSON2SeerFrame(6100, 6100, json_data);
    send(m_SockClient10, s_data, strlen(json_data) + 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient10, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void AMREndCreateMap() {
    char* s_data = RobotState2SeerFrame(6101, 6101);
    send(m_SockClient10, s_data, 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient10, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void AMRQueryScanMapStatus() {
    char* s_data = RobotState2SeerFrame(1025, 1025);
    send(m_SockClient04, s_data, 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient04, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void AMRQueryToatlMap() {
    char* s_data = RobotState2SeerFrame(1300, 1300);
    send(m_SockClient04, s_data, 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient04, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void ReScanMap() {
    // ������ͼ
    AMRStartCreateMap(1, 0, 0, 0);
    //AMRQueryToatlMap();
    //for (int i = 1; i <= 3; i++) {
    for (int i = 1; i <= 6; i++) {
        AMRRotation(2 * AGV_PI / 6., 1., m_SockClient06);
        AMRLocalInfo amrli;
        RequestAMRLocal(amrli);
        cout << "rad:" << AGV_PI / 6 << "    " << amrli._x << "   " << amrli._y << "  " << amrli._angle << endl;
        //AMRQueryScanMapStatus();
    }
    AMREndCreateMap();
    //AMRQueryToatlMap();
}


char* robot_control_reloc_req(const double x, const double y, const double theta, SOCKET& client) {
    // ������� { }
    cJSON* root = cJSON_CreateObject();//json���ݶ�
    cJSON_AddItemToObject(root, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(root, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(root, "theta", cJSON_CreateNumber(theta));
    cJSON_AddItemToObject(root, "length", cJSON_CreateNumber(2));
    //��������
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(2002, 2002, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return ReadAMRRespond(client);
}

char* robot_control_comfirmloc_req(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(2003, 2003);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

void robot_reloc_req(void) {
    int a;
    float x = 0, y = 0;
    x = AMRnow_position("x", m_SockClient04);
    y = AMRnow_position("y", m_SockClient04);//���λ��
    float theta = AMRnow_position("angle", m_SockClient04);
    cout << "x,y:" <<x<<"  " <<y<< endl;
    robot_control_reloc_req(x, y, theta, m_SockClient05);
    Sleep(10000);
    robot_control_comfirmloc_req(m_SockClient05);
    cout << "���������ַ����뷢��  ȷ����λ" << endl;
    //cin >> a;
    //robot_control_comfirmloc_req(m_SockClient05);

}

char* status_imu_req(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1014, 1014);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

Eigen::Vector3d robot_status_imu_req(SOCKET& client) {
    Eigen::Vector3d imu_req;
    char* jsonStr = status_imu_req(client);
    jsonStr = jsonStr + 16;
    //cout << "��ȡx��y����:" << jsonStr << endl;
    // ����ȡ����json�ַ���ת����json����ָ��
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        return imu_req;
    }
    //free(jsonStr);//08162600008
    /*************** ���� json***************/
    
    cJSON* item = NULL;
    item = cJSON_GetObjectItem(root, "yaw");
    if (item != NULL) {	// �Ϸ��Լ��
        if (item->type == cJSON_Number) {		// �ж��ǲ�������
            imu_req[0] = item->valuedouble;			// ��ȡֵ
            //printf("��ȡx = %f\n", x);
        }
    }
    item = cJSON_GetObjectItem(root, "roll");
    if (item != NULL) {	// �Ϸ��Լ��
        if (item->type == cJSON_Number) {		// �ж��ǲ�������
            imu_req[1] = item->valuedouble;			// ��ȡֵ
            //printf("��ȡx = %f\n", x);
        }
    }
    item = cJSON_GetObjectItem(root, "pitch");
    if (item != NULL) {	// �Ϸ��Լ��
        if (item->type == cJSON_Number) {		// �ж��ǲ�������
            imu_req[2] = item->valuedouble;			// ��ȡֵ
            //printf("��ȡx = %f\n", x);
        }
    }



    cJSON_Delete(root);
    //delete[] jsonStr;
    return imu_req;

}




