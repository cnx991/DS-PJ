#include <iostream>
#include <vector>
#include <climits>

using namespace std;

struct car {
    int start;//���
    int end;//Ŀ�ĵ�
    bool state;//�Ƿ񵽴�Ŀ�ĵ�
    int present_position[2];//��ǰ���ڵ�·�Ķ˵�
    int stay_time;//�ڵ�ǰ���ڵ�·�ϻ���ͣ����ʱ��
    vector<int> path;//��ʻ����·�����õص��¼
};

void show(vector<car> x, vector<vector<int>> map_) {//������г�����·������ʱ��
    int time = 0;//��ʼ��
    int pre, p;//��¼�������ڵ�·�������˵�
    for (auto& i : x) {//��������
        p = -1;//��ʼ���ص�
        for (auto& j : i.path) {//����·���еĵص�
            cout << j + 1 << " ";//�������·���ص�
            pre = p;
            p = j;//���³������ڵ�·�˵�
            if (pre != -1) time += map_[pre][p];//����·���ȼ�������ʻʱ�������ʱ��
        }
        cout << endl;
    }
    cout << time;//�����ʱ��
}

void improveddijkstra(vector<vector<int>> map_, vector<vector<int>> limit, vector<car>& cars) {//Ϊ�����滮·��
    int n = map_.size();
    vector<vector<int>> reality(n, vector<int>(n, 0));//��ʼ��ʵ�ʳ�����
    for (auto& car : cars) {
        if (car.stay_time != 0) {//����һ��λʱ��ӳ��������ڵ�ǰ��·��ͣ����ʱ���м�ȥ
            car.stay_time--;
        }
        if (car.stay_time != 0) {//�����������ڵ�ǰ��·����ʻ�����¼��ʵ�ʳ�������
            reality[car.present_position[0]][car.present_position[1]]++;
            reality[car.present_position[1]][car.present_position[0]]++;
            if (reality[car.present_position[0]][car.present_position[1]] == limit[car.present_position[0]][car.present_position[1]])
                //��ʵ�ʳ������ﵽ����������ƣ����ڵ�ǰʱ��ɾȥ�ﵽ����������Ƶ�·��
            {
                map_[car.present_position[0]][car.present_position[1]] = 0;
                map_[car.present_position[1]][car.present_position[0]] = 0;
            }
        }
    }

    for (auto& car : cars) {
        if (car.state == 1) continue;//������ѵ���Ŀ�ĵأ�������
        if (car.stay_time != 0) {//��������ͣ���ڵ�ǰ��·�ĳ���Ϊ�ѵ��ﵱǰ��·�յ�ĳ����¹滮·��
            continue;
        }
        if (car.present_position[1] == car.end - 1) {//���������ĵ�·�յ���Ŀ�ĵأ��򽫳���״̬����Ϊ�ѵ������������
            car.state = 1;
            continue;
        }
        int start = car.start - 1;//��������ӳ���������±�
        int end = car.end - 1;//��Ŀ�ĵ����ӳ���������±�
        vector<bool> s;//Ӧ��dijkstra�㷨����㵽Ŀ�ĵص����·��
        vector<int> dist;
        vector<int> min_path;
        for (int i = 0;i < n;i++) {
            if (map_[start][i] != 0) {
                dist.push_back(map_[start][i]);
            }
            else dist.push_back(INT_MAX);
            s.push_back(0);
            if (i != start && dist[i] < INT_MAX) {
                min_path.push_back(start);
            }
            else min_path.push_back(-1);
        }
        s[start] = 1;
        dist[start] = 0;
        int min;
        int w;
        for (int i = 0;i < n - 1;i++) {
            min = INT_MAX;
            int u = start;
            for (int j = 0;j < n;j++) {
                if (s[j] == 0 && dist[j] < min) {
                    u = j;
                    min = dist[j];
                }
            }
            s[u] = 1;
            for (int k = 0;k < n;k++) {
                if (map_[u][k] != 0) {
                    w = map_[u][k];
                }
                else w = INT_MAX;
                if (s[k] == 0 && w < INT_MAX && dist[u] + w < dist[k]) {
                    dist[k] = dist[u] + w;
                    min_path[k] = u;
                }
            }
        }
        if (min_path[end] != -1) {//����ҵ���㵽Ŀ�ĵص����·��
            for (int j = end;j != start;j = min_path[j]) car.present_position[1] = j;
            car.path.push_back(car.present_position[1]);//����㵽Ŀ�ĵص�·���еĵ�һ�ε�·��¼����ʻ����·����
            car.present_position[0] = start;//����ǰ���ڵ�·����ʼ�˵����Ϊ�������
            car.start = car.present_position[1] + 1;//��������㼴�滮��һ��·�ĳ�ʼ�����Ϊ��ǰ���ڵ�·���յ�
            car.stay_time = map_[car.present_position[0]][car.present_position[1]];//���������ڵ�ǰ��·ͣ����ʱ�����Ϊ��·����
            reality[car.present_position[0]][car.present_position[1]]++;//����ʵ�ʳ�����
            reality[car.present_position[1]][car.present_position[0]]++;
            if (reality[car.present_position[0]][car.present_position[1]] == limit[car.present_position[0]][car.present_position[1]])
                //��ʵ�ʳ������ﵽ����������ƣ����ڵ�ǰʱ��ɾȥ�ﵽ����������Ƶ�·��
            {
                map_[car.present_position[0]][car.present_position[1]] = 0;
                map_[car.present_position[1]][car.present_position[0]] = 0;
            }
        }
        else {//�����㵽Ŀ�ĵص����·��δ�ҵ���˵��·���ؾ���·��ʱ�ﵽ���������
            //��ô��ʱ�ó���ʻ��δ�ﵽ�����������̵�·�ȴ��ؾ���·��ȱ�����¹滮·��
            //���ҵ����·����������³�����Ϣ��ʵ�ʳ��������ж�ʻ���·�Ƿ�ﵽ����������ƣ��ﵽ���ڵ�ǰʱ��ɾȥ
            int j = -1;
            for (int k = 0;k < n;k++) {
                if (dist[k] < INT_MAX && j == -1 && k != start) j = k;
                if (dist[k] < dist[j] && k != start) j = k;
            }
            car.present_position[1] = j;
            car.path.push_back(car.present_position[1]);
            car.present_position[0] = start;
            car.start = car.present_position[1] + 1;
            car.stay_time = map_[car.present_position[0]][car.present_position[1]];
            reality[car.present_position[0]][car.present_position[1]]++;
            reality[car.present_position[1]][car.present_position[0]]++;
            if (reality[car.present_position[0]][car.present_position[1]] == limit[car.present_position[0]][car.present_position[1]])
            {
                map_[car.present_position[0]][car.present_position[1]] = 0;
                map_[car.present_position[1]][car.present_position[0]] = 0;
            }
        }
    }
}


int main() {
    int location_num, car_num;//��ʼ����ͼ������������
    cin >> location_num >> car_num;
    vector<vector<int>> map_(location_num, vector<int>(location_num, 0));
    vector<vector<int>> limit(location_num, vector<int>(location_num, 0));

    for (int i = 0; i < location_num; i++) {//��ͼ
        for (int j = 0; j < location_num; j++) {
            cin >> map_[i][j];
        }
    }

    for (int i = 0; i < location_num; i++) {//�����������
        for (int j = 0; j < location_num; j++) {
            cin >> limit[i][j];
        }
    }
    vector<car> cars;
    for (int i = 0; i < car_num; i++) {
        car car1;
        cars.push_back(car1);
    }
    for (int i = 0; i < car_num; i++) {//��ʼ��������㣬Ŀ�ĵ�
        cin >> cars[i].start >> cars[i].end;
    }

    for (int i = 0; i < car_num; i++) {//��ʼ��������������
        cars[i].state = 0; cars[i].stay_time = 0;
        cars[i].path.push_back(cars[i].start - 1);
        cars[i].present_position[0] = cars[i].start;
        cars[i].present_position[1] = -1;
    }

    while (1) {//ѭ��ֱ�����г�������Ŀ�ĵأ�1��ѭ������1��λʱ��
        improveddijkstra(map_, limit, cars);//ÿʱ������Ϊ�����滮·��
        int i;
        for (i = 0;i < car_num;i++) {
            if (cars[i].state == 0) break;//����г�δ����Ŀ�ĵأ��˳�
        }
        if (i == car_num) break;//�ж��Ƿ����г�������Ŀ�ĵ�
    }
    show(cars, map_);//���
    return 0;
}