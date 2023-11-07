#include "dijkstra.h"
#include "GlobalVariable.h"

extern int toward;


int FR[X][Y] = { 0 };
int FR1[X][Y] = { 0 };
int FR2[X][Y] = { 0 };

char atom_window[] = "Drawing 1: Atom";
cv::Mat atom_image = cv::Mat::zeros(Y * 10, X * 10, CV_8UC3);//��

//���ܷ��ں�����  ����̫��
float edges[X * Y + 10][X * Y + 10];//����x*x�ĵ�ͼ   ��Ҫx*x����ű��   // ������еıߣ����� edges[i][j] �����i��j�ľ���

int startendcoincidence = 0;//������յ��غϣ�startendcoincidence=1

//·���滮dijkstra
int* dijkstra(float x, float y, float xx, float yy)
{
    //��ʼλ��
    int o_now = 0;
    cv::Point2f point_now((x + 16) * 30, (y + 11) * 30);//Ŀǰλ�� x  y  תΪo_now
    cv::circle(atom_image, point_now, 2, cv::Scalar(255, 255, 255), -1);
    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            if (abs(m * 10 - point_now.x) <= 5 && abs(n * 10 - point_now.y) <= 5) {
                o_now = n * X + m;
                //cout << "m=" << m << " n=" << n << endl;
            }
        }
    }
    //cout << "o_now=" << o_now << endl;


    //�ҿ��ƶ���
    memset(edges, 100, sizeof(edges));//������ ��ʼ����Ϊ100=����
    cv::Point2f point;
    for (int i = 0; i < X; i++) {
        for (int j = 0; j < Y; j++) {
            if ((i != 0) && (i != X - 1) && (j != 0) && (j != (Y - 1))) {

                if (FR[i][j] == 0 && FR[i + 1][j] == 0) {
                    edges[j * X + i][j * X + i + 1] = edges[j * X + i + 1][j * X + i] = 1;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + 1] = edges[j * X + i + 1][j * X + i] = 10000;
                //    point.x = (i + 0.5) * 10;
                //    point.y = j * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i - 1][j] == 0) {
                    edges[j * X + i][j * X + i - 1] = edges[j * X + i - 1][j * X + i] = 1;
                    //point.x = (i - 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - 1] = edges[j * X + i - 1][j * X + i] = 10000;
                //    point.x = (i - 0.5) * 10;
                //    point.y = j * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i][j - 1] == 0) {
                    edges[j * X + i][j * X + i - X] = edges[j * X + i - X][j * X + i] = 1;
                    //point.x = i * 10;
                    //point.y = (j - 0.5) * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - X] = edges[j * X + i - X][j * X + i] = 10000;
                //    point.x = i * 10;
                //    point.y = (j - 0.5) * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i][j + 1] == 0) {
                    edges[j * X + i][j * X + i + X] = edges[j * X + i + X][j * X + i] = 1;
                    //point.x = i * 10;
                    //point.y = (j + 0.5) * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + X] = edges[j * X + i + X][j * X + i] = 10000;
                //    point.x = i * 10;
                //    point.y = (j + 0.5) * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}

                if (FR[i][j] == 0 && FR[i + 1][j + 1] == 0) {//&& FR[i][j + 1] == 0 && FR[i+1][j] == 0
                    edges[j * X + i][j * X + i + 1 + X] = edges[j * X + i + 1 + X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + 1 + X] = edges[j * X + i + 1 + X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i + 1][j - 1] == 0) {//&& FR[i+1][j] == 0 && FR[i][j - 1] == 0
                    edges[j * X + i][j * X + i + 1 - X] = edges[j * X + i + 1 - X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + 1 - X] = edges[j * X + i + 1 - X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i - 1][j - 1] == 0) {//&& FR[i][j - 1] == 0 && FR[i-1][j] == 0
                    edges[j * X + i][j * X + i - 1 - X] = edges[j * X + i - 1 - X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - 1 - X] = edges[j * X + i - 1 - X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i - 1][j + 1] == 0) {// && FR[i-1][j] == 0 && FR[i][j + 1] == 0
                    edges[j * X + i][j * X + i - 1 + X] = edges[j * X + i - 1 + X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - 1 + X] = edges[j * X + i - 1 + X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}


            }
        }
    }

    //�յ�λ��
    int end;
    cv::Point2f point_end((xx + 16) * 30, (yy + 11) * 30);
    circle(atom_image, point_end, 3, cv::Scalar(255, 255, 255), -1);
    for (int i = 0; i <= X; i++) {
        for (int j = 0; j <= Y; j++) {
            if (abs(i * 10 - point_end.x) <= 5 && abs(j * 10 - point_end.y) <= 5) {
                end = j * X + i;
                //cout << "m=" << i << " n=" << j << endl;
            }
        }
    }
    //cout << "end=" << end << endl;

    if (end == o_now) {//�յ������һ��
        startendcoincidence = 1;
        return nullptr;
    }

    /////////////////////////·���滮////////////////////
    float dist[X * Y];  // ��¼��ǰ���е㵽���ľ���
    memset(dist, 100, sizeof(dist));  // ��ʼ��ÿ��dist��ֵΪ����100=��1684300900m�� 
    //memset�ǰ����ֽ������õģ�ÿ���ֽ�Ϊ0x3f, int�ĸ��ֽڣ������ 0x3f3f3f3f
    int visited[X * Y];  // ��ǵ�ǰ�ĵ��Ƿ��߳�
    memset(visited, 0, sizeof(visited)); // ��ʼ�����еĵ㶼û�б��߳���
    string path[X * Y];//ÿһ����·��˳��
    int m = o_now, n = end;///////�õ���ʼ���յ�
    int away[1000] = { 0 };//����·��

    for (int i = 0; i < X * Y; i++) {  // ÿ��ѭ�������޳���1���㣬�����Ҫforѭ������n�Ρ�
        int index = -1;  // index����ǰδ�����ʵľ���ԭ������ĵ�
        //dist[index + X] = dist[index + X+1];
        dist[m] = 0; //����ԭ��   ԭ�㵽ԭ��ľ���Ϊ0���������Ҫ����������forѭ�����е�dist����0x3f3f3f3f,�޷��ҳ�index��

        for (int j = 0; j < X * Y; j++) { // find the index of min distance 
            if (!visited[j] && (index == -1 || dist[j] < dist[index])) { // ��ǰ�ĵ�û�б��߳������ҵ�ǰ��ľ����index��ľ���С�������index��index == -1��ʾ��δ��ʼ�ҵ�dist����Сֵ�����dist[1]���롣
                index = j;
            }
        }

        visited[index] = 1;  //�ҵ���ǰ����ԭ����Сֵ�ĵ㣬��ѵ���б���߳���
        //�������·��
        if (dist[index] + edges[index][index + 1] < dist[index + 1]) { //index������������������е㡣
            dist[index + 1] = dist[index] + edges[index][index + 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + 1] = path[index] + " " + to_string(index + 1);

            int nn = index + 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index - 1] < dist[index - 1]) { //index������������������е㡣
            dist[index - 1] = dist[index] + edges[index][index - 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - 1] = path[index] + " " + to_string(index - 1);

            int nn = index - 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index + X] < dist[index + X]) { //index������������������е㡣
            dist[index + X] = dist[index] + edges[index][index + X];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + X] = path[index] + " " + to_string(index + X);

            int nn = index + X;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index - X] < dist[index - X]) { //index������������������е㡣
            dist[index - X] = dist[index] + edges[index][index - X];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - X] = path[index] + " " + to_string(index - X);

            int nn = index - X;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }

        if (dist[index] + edges[index][index - X - 1] < dist[index - X - 1]) { //index������������������е㡣
            dist[index - X - 1] = dist[index] + edges[index][index - X - 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - X - 1] = path[index] + " " + to_string(index - X - 1);

            int nn = index - X - 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index - X + 1] < dist[index - X + 1]) { //index������������������е㡣
            dist[index - X + 1] = dist[index] + edges[index][index - X + 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - X + 1] = path[index] + " " + to_string(index - X + 1);

            int nn = index - X + 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index + X + 1] < dist[index + X + 1]) { //index������������������е㡣
            dist[index + X + 1] = dist[index] + edges[index][index + X + 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + X + 1] = path[index] + " " + to_string(index + X + 1);

            int nn = index + X + 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index + X - 1] < dist[index + X - 1]) { //index������������������е㡣
            dist[index + X - 1] = dist[index] + edges[index][index + X - 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + X - 1] = path[index] + " " + to_string(index + X - 1);

            int nn = index + X - 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }

        }

        if (i > 5000 || dist[index] > 100) {//���߳���100��   ������ɨ������������   ����Ϊû�п���·��
            cout << "û��·�߿��Ե���" << endl;
            return nullptr;
        }

        if (n == index) {  //�����n���·�����򷵻�
            //cout << path[n] << endl;

            int x = 1, y = 1, p = 0;
            while (1) {//�ҵ��ܹ�����

                while (1) {//�ҵ�ÿһ��������
                    if (path[n][x] == ' ' || path[n][x] == '\0')
                        break;
                    x++;
                }
                string paths = path[n].substr(y, x - y);
                //cout << paths << endl;
                y = ++x;
                away[p] = stoi(paths, 0, 10);
                //cout << away[p] << endl;
                if (path[n][x - 1] == '\0')
                    break;
                p++;
            }
            //return dist[n];
            break;
        }
    }



    //��ͼ��ʾ·��//
    cv::Point2f points;
    for (int i = 0; away[i] != 0; i++) {
        points.x = (int)(away[i] % X) * 10;
        int xxx = away[i] / X;
        points.y = (xxx) * 10;
        circle(atom_image, points, 2, cv::Scalar(150, 150, 255), -1);
    }

    //aways[100]���ֱ�ߵ�
    int aways[100] = { 0 };
    int away_flags = away[1] - away[0];
    int away_flag = 0;
    aways[0] = end;
    for (int i = 2, j = 0; away[i] != 0; i++) {//��ֱ��
        if (away[i] - away[i - 1] == -1)  away_flag = -1;//8������
        if (away[i] - away[i - 1] == -(X + 1))  away_flag = -(X + 1);
        if (away[i] - away[i - 1] == -X)  away_flag = -X;
        if (away[i] - away[i - 1] == -(X - 1))  away_flag = -(X - 1);
        if (away[i] - away[i - 1] == 1)  away_flag = 1;
        if (away[i] - away[i - 1] == (X + 1))  away_flag = (X + 1);
        if (away[i] - away[i - 1] == X)  away_flag = X;
        if (away[i] - away[i - 1] == (X - 1))  away_flag = (X - 1);

        if (away_flag != away_flags) {
            away_flags = away_flag;
            aways[j] = away[i - 1];
            //cout << "xxxxxxxxxxx=" << aways[j] << endl;
            j++;
        }
        aways[j] = end;
    }

    //ֱ�ߵ�תΪ��ͼʵ�ʵ㣨x m��y m��
    for (int i = 0; aways[i] != 0; i++) {//��ֱ��
        //cout << "x=" << (aways[i] % X)/3-16 << "y=" << (aways[i] / X)/3-11 << endl;
        points.x = aways[i] % X * 10;
        points.y = aways[i] / X * 10;
        circle(atom_image, points, 5, cv::Scalar(0, 255, 0), -1);
        float xx, yy;
        xx = ((int)(aways[i] % X)) / 3.0 - 16.0;
        yy = ((int)(aways[i] / X)) / 3.0 - 11.0;
        //printf("x=%f   y=%f\n", xx, yy);
    }
    //cout << "dist[n]=" << dist[n] << endl;
    return aways;
}

//int map_obstacles() {
int map_obstacles(string path) {
    // ��ͼ����
    // �򿪵�ͼ�ļ�
    FILE* file = NULL;
    //file = fopen("map.smap", "r");
    file = fopen(path.c_str(), "r");
    if (file == NULL) {
        printf("Open file fail��\n");
        return 0;
    }
    // ����ļ���С
    struct stat statbuf;
    //stat("map.smap", &statbuf);
    stat(path.c_str(), &statbuf);
    int fileSize = statbuf.st_size;
    printf("�ļ���С��%d\n", fileSize);

    // ��������ļ���С���ڴ�
    char* jsonStr = (char*)malloc(sizeof(char) * fileSize + 1);
    memset(jsonStr, 0, fileSize + 1);

    // ��ȡ�ļ��е�json�ַ���
    int size = fread(jsonStr, sizeof(char), fileSize, file);
    if (size == 0) {
        printf("��ȡ�ļ�ʧ�ܣ�\n");
        fclose(file);
        return 0;
    }
    //printf("%s\n", jsonStr);
    fclose(file);

    // ����ȡ����json�ַ���ת����json����ָ��
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        free(jsonStr);
        return 0;
    }
    free(jsonStr);

    /*************** ���� json***************/
    cJSON* item = NULL;
    int number_map = 0;
    float mapdata[100000][2];
    float maxx = -1000, maxy = -1000, minx = 1000, miny = 1000;
    item = cJSON_GetObjectItem(root, "normalPosList");
    if (item != NULL) {
        cJSON* obj = item->child;	// ��� [1][]
        while (obj) {
            if (obj->type == cJSON_Object) {

                cJSON* objValue = obj->child;	// [1][1]
                while (objValue) {
                    float v_double = objValue->valuedouble;

                    if (number_map % 2 == 0) {//ȡx
                        mapdata[number_map / 2][0] = v_double;
                        //printf("%s = %.2f\n", objValue->string, mapdata[number_map / 2][0]);
                        if (v_double > maxx)   maxx = v_double;
                        if (v_double < minx)   minx = v_double;
                    }
                    else {//ȡy
                        mapdata[number_map / 2][1] = v_double;
                        //printf("%s = %.2f\n", objValue->string, mapdata[number_map / 2][1]);
                        if (v_double > maxy)   maxy = v_double;
                        if (v_double < miny)   miny = v_double;
                    }
                    // ��ȡ��һ��Ԫ��
                    objValue = objValue->next;
                    number_map++;
                }
            }
            // ��ȡ��һ��Ԫ��
            obj = obj->next;
        }
    }
    cJSON_Delete(root);
    printf("x_max=%f   y_max=%f  x_mix=%f   y_mix=%f\n", maxx, maxy, minx, miny);
    printf("��ͼpoint��=%d\n", number_map / 2);

    //��ԭ��ͼ
    cv::Point2f points;
    cv::Point2f point;

    //circle(atom_image, points, 1, Scalar(0, 255, 120), -1);
    points.x = (int)(mapdata[0][0] * 30);
    points.y = (int)(mapdata[0][1] * 30);
    //circle(atom_image, points, 1, Scalar(0, 255, 120), -1);
    for (int pp = 0; pp < number_map / 2; pp++) {
        points.x = (int)(mapdata[pp][0] * 30) + 480;
        points.y = (int)(mapdata[pp][1] * 30) + 330;
        circle(atom_image, points, 1, cv::Scalar(0, 255, 120), -1);//��ɫ
        //printf("x_max=%f   y_max=%f  %d\n", points.x, points.y, number_map);
    }

    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            for (int pp = 0; pp < number_map / 2; pp++) {

                points.x = (int)(mapdata[pp][0] * 30) + 480;
                points.y = (int)(mapdata[pp][1] * 30) + 330;
                if (abs(m * 10 - points.x) < 20 && abs(n * 10 - points.y) < 20) {//�����ϰ���
                    point.x = m * 10;
                    point.y = n * 10;
                    //circle(atom_image, point, 2, Scalar(0, 0, 255), -1);
                    FR1[m][n] = 1;
                }
            }
            if (!FR1[m][n]) {
                point.x = m * 10;
                point.y = n * 10;
                circle(atom_image, point, 1, cv::Scalar(255, 0, 0), -1);
            }


        }
    }

    return 0;
}


int map_Laser_res(SOCKET& client) {

    float x = 0, y = 0, angle0 = 0;
    x = AMRnow_position("x", client);
    y = AMRnow_position("y", client);
    angle0 = AMRnow_position("angle", client);

    char* Laser_res = ReadAMRLaser_res(client);//��ȡ�ɹ����ܼ����״���������  ��Ӧ
    Laser_res = Laser_res + 16;
    cJSON* root = cJSON_Parse(Laser_res);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //free(jsonStr);
        return 0;
    }
    /*************** ���� json***************/
    cJSON* item = NULL;
    int number_map = 0;
    float mapdata[100000][2];
    float maxx = -1000, maxy = -1000, minx = 1000, miny = 1000;
    float dist_double;
    float angle_double;
    item = cJSON_GetObjectItem(root, "lasers");//
    if (item != NULL) {
        cJSON* array_items = cJSON_GetArrayItem(item, 0);//��ȡ��������
        cJSON* array_item = cJSON_GetObjectItem(array_items, "beams");//��ȡbeams����
        cJSON* obj = array_item->child;	// ��� [1][]
        //if(obj)
        //    cout << "�ҵ��˼����״�����" << endl;
        while (obj) {
            if (obj->type == cJSON_Object) {
                cJSON* angle = cJSON_GetObjectItem(obj, "angle");
                if (angle) {
                    angle_double = angle->valuedouble;
                    //printf("angle=%f\n", angle_double);
                }
                cJSON* dist = cJSON_GetObjectItem(obj, "dist");
                if (dist) {
                    dist_double = dist->valuedouble;
                }

                mapdata[number_map][0] = x + 0.553317 * cos(angle0) + dist_double * cos(angle0 + (angle_double)*PI / 180);//0.55�����״�����ڳ���λ��
                mapdata[number_map][1] = y + 0.553317 * sin(angle0) + dist_double * sin(angle0 + (angle_double)*PI / 180);
                //cout << sin(90.0 * PI / 180) << endl;;
                number_map++;

            }
            // ��ȡ��һ��Ԫ��
            obj = obj->next;
        }
    }
    cJSON_Delete(root);

    //printf("x_max=%f   y_max=%f  x_mix=%f   y_mix=%f\n", maxx, maxy, minx, miny);
    //printf("��ͼpoint��=%d\n", number_map / 2);

    //��ԭ�����״�����
    cv::Point2f points;
    for (int pp = 0; pp < number_map - 1; pp++) {
        points.x = (int)(mapdata[pp][0] * 30) + 480;
        points.y = (int)(mapdata[pp][1] * 30) + 330;
        circle(atom_image, points, 1, cv::Scalar(150, 120, 0), -1);//��ɫ
    }

    //�ҵ��ϰ���
    cv::Point2f point;
    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            for (int pp = 0; pp < number_map - 1; pp++) {
                points.x = (int)(mapdata[pp][0] * 30) + 480;
                points.y = (int)(mapdata[pp][1] * 30) + 330;
                if (abs(m * 10 - points.x) < 20 && abs(n * 10 - points.y) < 20) {//�����ϰ���
                    point.x = m * 10;
                    point.y = n * 10;
                    circle(atom_image, point, 2, cv::Scalar(50, 50, 255), -1);
                    FR2[m][n] = 1;
                }
            }
            if (!FR2[m][n]) {
                point.x = m * 10;
                point.y = n * 10;
                //circle(atom_image, point, 1, Scalar(255, 50, 30), -1);
            }
        }
    }

    return 0;
}
void Move2Goal(const Eigen::Vector2d turegoal, const Eigen::Vector2d goal) {
    //��ͼ���ݴ���
    string localMapPath = "./20231104203536530.smap";
    //��ͼ���ݴ���
    map_obstacles(localMapPath);
    cout << "behind_thetas:" << endl;
    //�ϰ�Ѱ��
    memset(FR2, 0, sizeof(FR2));
    map_Laser_res(m_SockClient04);
    for (int m = 0; m < X; m++)
        for (int n = 0; n < Y; n++)
            FR[m][n] = FR1[m][n] + FR2[m][n];
    cout << "behind_thetas:"  << endl;
    AMRLocalInfo currAmrli;
    // ��ȡ��ǰagv���ԭʼλ��
    RequestAMRLocal(currAmrli);
    double yyy = turegoal[1] / 1000., yy = goal[1] / 1000.;
    double xxx = turegoal[0] / 1000., xx = goal[0] / 1000.;
    double x = currAmrli._x, y = currAmrli._y;
    float behind_thetas = asin((yyy - yy) / sqrt((xxx - xx) * (xxx - xx) + (yyy - yy) * (yyy - yy)));
    if ((xxx - xx) > 0)
        behind_thetas;
    else if ((yyy - yy) > 0)
        behind_thetas = 3.1415926 - behind_thetas;
    else
        behind_thetas = -(3.1415926 + behind_thetas);
    cout << "behind_thetas:" << behind_thetas << endl;
    //cout << atan2(yyy - yy, xxx - xx) << endl;
    cout << "x��y��xx��xy��" << x << "  " << y << " " << xx << "    " << yy << "    " << yyy << "   " << xxx << endl;
    int* aways;
    aways = dijkstra(x, y, xx, yy);////////////////·���滮
    if (aways == NULL) {
        cout << "����Ŀ�꣺" << goal.transpose() << "�޿ɴ�·��" << endl;
        return;//ԭ��ͼ ��·�����Ե���         
    }
    cout << "find route!" << endl;
    //// ȡ��AGV�˶��켣
    //vAGVMovePosition.clear();
    //vAGVMovePosition.emplace_back(agi._oriAgvPos); // �����ʼλ��
    ////cout << "AGV��ʼλ�ã�" << agi._oriAgvPos.transpose() << endl;
    ////cout << "robot move pos:" << endl;
    //for (int i = 0; aways[i] != 0; i++) {
    //    Eigen::Vector3d point{
    //        1000 * ((int)(aways[i] % X) / 3.0 - 16.0),
    //        1000 * ((int)(aways[i] / X) / 3.0 - 11.0),
    //        1.
    //    };
    //    vAGVMovePosition.emplace_back(point);
    //    //cout << point.transpose() << endl;
    //}
    //ConvertAGVMove2ArmTra(vAGVMovePosition, vArmMovePosition);

    //while (1) {
    //    AMRLocalInfo currAmrli;
    //    // ��ȡ��ǰagv���ԭʼλ��
    //    RequestAMRLocal(currAmrli);
    //    currAmrli._x *= 1000.;
    //    currAmrli._y *= 1000.;
    //    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

    //    //cout << "main thread!" << endl;
    //    Sleep(10);
    //}
    ////return;

    //cout << "agvλ�ã�x" << x << "    y:" << y << "  ����:" << sta << endl;
    //cout << "cos:" << asin(sinsta1) * 180 / CV_PI << "    sin:" << asin(cossta1) * 180 / CV_PI << endl;
    //cout << "xx:" << xx << "    yy:" << yy << "  " << behind_thetas << endl;
    //cout << "Position x:" << Position.at<double>(0) / 1000.0 << "   y:" << Position.at<double>(1) / 1000.0 << endl;
    //for (int i = 0; aways[i] != 0; i++) {
    //    cout << (int)(aways[i] % X) / 3.0 - 16.0 << "    " << (int)(aways[i] / Y) / 3.0 - 11.0 << endl;
    //}
    //cout << endl;
    //return ;

    int blocked = 0, next_blocked = 0;
    int navigation_status = -1;
    for (int i = 0; aways[i] != 0; i++) {//ֱ������
        //// �ж�AGV�Ƿ��쳣
        //if (agi._move2goal) {
        //    cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl;
        //    AMRstop_Navigation(m_SockClient06);
        //    return;
        //}

        //cout << "-----------------------------------------------1" << endl;
        if (aways[i + 1] == 0) {//��������һ���� ֱ�ӵ����յ�
            //if(toward)
                AMRRobotNavigation(xx, yy, behind_thetas, m_SockClient06);
            //else
            //    AMRRobotNavigation(xx, yy, (behind_thetas+ 3.1415926/2.0), m_SockClient06);
        }
        else
            AMRRobotNavigation(((int)(aways[i] % X)) / 3.0 - 16.0, ((int)(aways[i] / X)) / 3.0 - 11.0, m_SockClient06);

        Sleep(100);//������ʼ

        while (1) {
            AMRLocalInfo currAmrli;
            // ��ȡ��ǰagv���ԭʼλ��
            RequestAMRLocal(currAmrli);
            currAmrli._x *= 1000.;
            currAmrli._y *= 1000.;
            agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

            //cout << "-----------------------------------------------2" << endl;
            //// �ж�AGV�Ƿ��쳣
            //if (agi._move2goal) {
            //    cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl;
            //    AMRstop_Navigation(m_SockClient06);
            //    return;
            //}

            //cout << "-----------------------------------------------2" << endl;
            navigation_status = AMRnavigation_status(m_SockClient04);
            if (navigation_status == 4 || navigation_status == 0)//�˶����||�޵���   ������һ���˶�
            {
                
                //cout << "-----------------------------------------------3" << endl;
                break;
            }

            //cout << "-----------------------------------------------3" << endl;

            blocked = AMRblocked_status(m_SockClient04);//��ȡ�����״�״̬
            //cout << "-----------------------------------------------341" << endl;
            //cout << "daohqk" << blocked << endl;
            memset(FR2, 0, sizeof(FR2));
            map_Laser_res(m_SockClient04);
            //cout << "-----------------------------------------------3" << endl;
            if (blocked == 0)//
                for (int m = 0; m < X; m++)
                    for (int n = 0; n < Y; n++) {
                        if (FR2[m][n]) {
                            int t1 = aways[i - 1];
                            int t2 = aways[i];
                            int x1, y1, x2, y2;
                            x1 = t1 % X;
                            y1 = t1 / X;
                            x2 = t2 % X;
                            y2 = t2 / X;
                            for (; t1 != t2;) {
                                if (((x2 - x1) > 0) && ((y2 - y1) == 0)) {
                                    t1 += 1;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) < 0) && ((y2 - y1) == 0)) {
                                    t1 -= 1;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) == 0) && ((y2 - y1) > 0)) {
                                    t1 += X;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) == 0) && ((y2 - y1) < 0)) {
                                    t1 -= X;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) < 0) && ((y2 - y1) < 0)) {
                                    t1 -= (X + 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) > 0) && ((y2 - y1) < 0)) {
                                    t1 -= (X - 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) < 0) && ((y2 - y1) > 0)) {
                                    t1 += (X - 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) > 0) && ((y2 - y1) > 0)) {
                                    t1 += (X + 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                            }
                        }
                    }
            //cout << "zusai" << blocked << endl;
            //cout << "-----------------------------------------------4" << endl;

            if (blocked == 1) {//���ⱻ����
                //cout << "-----------------------------------------------5" << endl;
                //agi._move2goal = 1;
                //break;
                // ��������ֹͣ����
                AMRstop_Navigation(m_SockClient06);//ֹͣ��ǰ����
                memset(FR2, 0, sizeof(FR2));
                map_Laser_res(m_SockClient04);
                for (int m = 0; m < X; m++)
                    for (int n = 0; n < Y; n++)
                        FR[m][n] = FR1[m][n] + FR2[m][n];

                x = AMRnow_position("x", m_SockClient04);//���赲λ��Ϊ���
                y = AMRnow_position("y", m_SockClient04);

                int* aways;
                aways = dijkstra(x, y, xx, yy);
                //for (int i = 0; aways[i] != 0; i++) {//��ֱ��
                //    cout << "aways[i]:" << aways[i]<< endl;
                //} 
                i = -1;//iҪ�Լ�һ
                while (aways == NULL) {//��·�����Ե���
                    memset(FR2, 0, sizeof(FR2));
                    map_Laser_res(m_SockClient04);
                    for (int m = 0; m < X; m++)
                        for (int n = 0; n < Y; n++)
                            FR[m][n] = FR1[m][n] + FR2[m][n];
                    aways = dijkstra(x, y, xx, yy);
                    if (startendcoincidence) {
                        int awayss[100] = { 0 };
                        awayss[0] = 1;
                        aways = awayss;
                        break;
                    }
                }
                break;//���¿�ʼ
            }
            //cout << "-----------------------------------------------5" << endl;
        }

        //cout << "-----------------------------------------------6" << endl;
    }

    //cout << "-----------------------------------------------7" << endl;
}


Eigen::MatrixXd cloud_ICP(std::shared_ptr<geometry::PointCloud>& source, std::shared_ptr<geometry::PointCloud>& target) {
    if (source == nullptr || target == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        //return ;
    }
    //visualization::DrawGeometries({ source, target },"initial state");

    std::vector<double> voxel_sizes = { 1., 1./4 }; //�²�������դ��ı߳�  0.05  10.5
    std::vector<int> iterations = { 50*4, 14*2 }; //����������
    //std::vector<double> voxel_sizes = { 2., 10.05 / 4 }; //�²�������դ��ı߳�  0.05  10.5
    //std::vector<int> iterations = { 100, 14 * 2 }; //����������
    Eigen::Matrix4d trans_point2plane = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd trans_point2plane1(4, 4);
    for (int i = 0; i < 1; ++i) {
        float voxel_size = voxel_sizes[i];

        auto source_down = source->VoxelDownSample(voxel_size);
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));

        auto target_down = target->VoxelDownSample(voxel_size);
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));

        double max_correspondence_distance = 150.;  //��Ӧ��Ե������룬���ֵ���ڽ��Ӱ��ܴ�   0.07
        auto result_point2plane = open3d::registration::RegistrationICP( //�㵽���ICP
            *source_down, *target_down, max_correspondence_distance, trans_point2plane,
            //open3d::registration::TransformationEstimationPointToPlane(),
            open3d::registration::TransformationEstimationPointToPoint(),
            open3d::registration::ICPConvergenceCriteria(1e-6, 1e-6, iterations[i])
        );
        //double max_correspondence_distance = 150.;
        //auto result_point2plane = open3d::registration::RegistrationICP(*source, *target, max_correspondence_distance);

        trans_point2plane = result_point2plane.transformation_;

        std::cout << "ƥ�䷽ʽ�������" << " " << "inlier_rmse:" << result_point2plane.inlier_rmse_ << endl;
        trans_point2plane1 = trans_point2plane;
        std::cout << trans_point2plane << endl;
        for (int pd = 0; pd < source->points_.size(); pd++) {
            Eigen::MatrixXd bt(4, 1);
            bt << source->points_[pd][0], source->points_[pd][1], source->points_[pd][2], 1;
            source->points_[pd] = trans_point2plane1 * bt;
        }
        //visualization::DrawGeometries({ source, target },"initial state1");
        max_correspondence_distance = 5. / ((i + 1));
    }

    return trans_point2plane1;
}

cv::Mat PointCloudICP(std::shared_ptr<geometry::PointCloud>& source, std::shared_ptr<geometry::PointCloud>& target) {
    cv::Mat result = cv::Mat::zeros(4, 4, CV_64F);
    if (source == nullptr || target == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        return result;
    }

    double voxel_size = 0.05; //�²�������դ��ı߳�  0.05  10.5
    int iteration = 50 * 4; //����������
    Eigen::Matrix4d trans_point2plane = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd trans_point2plane1(4, 4);
    auto source_down = source->VoxelDownSample(voxel_size);
    source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));
    auto target_down = target->VoxelDownSample(voxel_size);
    target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));

    double max_correspondence_distance = 250.;  //��Ӧ��Ե�������mm�����ֵ���ڽ��Ӱ��ܴ�   0.07
    //double max_correspondence_distance = 150.;  //��Ӧ��Ե�������mm�����ֵ���ڽ��Ӱ��ܴ�   0.07
    auto result_point2plane = open3d::registration::RegistrationICP( //�㵽���ICP
        *source_down, *target_down, max_correspondence_distance, trans_point2plane,
        open3d::registration::TransformationEstimationPointToPoint(),
        open3d::registration::ICPConvergenceCriteria(1e-6, 1e-6, iteration)
    );
    trans_point2plane = result_point2plane.transformation_;

    std::cout << "ƥ�䷽ʽ�������" << " " << "inlier_rmse:" << result_point2plane.inlier_rmse_ << endl;
    std::cout << trans_point2plane << endl;
    for (int pd = 0; pd < source->points_.size(); pd++) {
        Eigen::Matrix<double, 4, 1> bt;
        bt << source->points_[pd][0], source->points_[pd][1], source->points_[pd][2], 1;
        Eigen::Matrix<double, 4, 1> converted = trans_point2plane * bt;
        source->points_[pd] = Eigen::Vector3d{ converted[0], converted[1], converted[2] };
    }

    for (int i = 0; i < 4; ++i) 
        for (int j = 0; j < 4; ++j) 
            result.at<double>(i, j) = trans_point2plane(i, j);

    return result;
}


