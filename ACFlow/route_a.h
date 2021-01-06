#pragma once
#pragma comment(lib,"BasicGraph.lib")
#include <iostream>
#include <vector>
#include <fstream>
#include <time.h>
#include<unordered_map>
#include <windows.h>
#include<algorithm>
#include<numeric>
#include<sstream>
#include <queue>
#include<unordered_set>
#include<set>
#include"path.h"
#include "Graph.h"

#define myType int
/********using extern to declaration ***********/
extern const int m;//ants number
extern const float beta;//heuristic information weight 
extern int G;//��������
extern const float rho;//pheromone violation factor 
extern const float xik;//
extern const float pher_start;
extern const float v_start;
extern const int w;//number of ants allowed to release pheromone 
extern const int cap;//capacity of each road
extern int traffic_time; // ���Լ����ٶȵ�ʱ��
extern const float heur_strat;

extern const int target;//�趨��ʼ�̶�Դ�ڵ�  48//521
extern vector<myType> source;//ÿֻ���ϵ�Դ�ڵ�  521
extern vector<myType> safe_site;//��ȫ�Ľڵ�  �� Ŀ�Ľڵ㼯��
extern vector<myType> sites;//ȫ���ڵ�ļ���
extern vector<bool> ant_dest;//�Ƿ񵽴��յ��ʶ
extern vector<vector<myType> > ant_solution;
extern vector<unordered_map<myType, float> > flow_mtx;//flow hash function

extern vector<vector<pair<myType, float> > > heur_mtx;//heuristic information 
extern vector<vector<pair<myType, float> > > trans_mtx;//Transition matrix
extern vector<vector<pair<myType, float> > > pher_mtx;//Pheromone matrix

extern set<pair<int, int> > cur_global_route;
extern vector<vector<pair<myType, pair<int, int> > > > density_mtx;//density matrix  ��flow ͬʱʵ�� �����Ϊ�ڵ���
extern vector<float> fit_mtx;//��Ӧ�Ⱦ���
extern pair<float, vector<myType>> history_best;
extern vector<vector<myType> > transmit_p;// ���;���
extern vector<vector<myType> > short_p;//short path

extern UndiGraph<myType, float> g;
/********end of Data declaration ***********/

void routeConstricution();
void new_init();
vector<float> fitCompute();
float velocityCompute(myType a, myType b);
void updateLocalPher(myType a, myType b);
void updateGlobalPher();
//float meanCompute();
void readFile(string direction);
//float rand_n(int N);
//int get_pos(myType t);
void updateTrans(bool tag = true);
void updateNodeTrans(myType a);
bool graphPreprocess(int v, int num, vector<vector<myType> >& p, vector<vector<myType> >& res);

inline float meanCompute() {
	float sum = accumulate(fit_mtx.begin(), fit_mtx.end(), 0.0);
	sum /= (float)m;
	return sum;
}
//��ǰ�ڵ��Ƿ�safe
inline bool isSafe(int p) {
	vector<int>::iterator it = find(safe_site.begin(), safe_site.end(), p);
	if (it != safe_site.end()) return true;
	else return false;
}
//num��ʾ����id  p��ʾ��ǰվ��
inline bool isEnd(int p, int num) {
	if (p == safe_site[num]) return true;
	else return false;
}

inline float rand_p() {
	int N = 999;//�������������
	LARGE_INTEGER seed;
	QueryPerformanceFrequency(&seed);
	QueryPerformanceCounter(&seed);
	srand(seed.QuadPart);
	//return rand();
	return rand() % (N + 1) / (float)(N + 1);
}
//����0��N-1֮��������
inline float rand_n(int N) {
	//srand(time(NULL));//������������ӣ�ʹÿ�β�����������в�ͬ	
	LARGE_INTEGER seed;
	QueryPerformanceFrequency(&seed);
	QueryPerformanceCounter(&seed);
	srand(seed.QuadPart);
	return rand() % N;
}
inline int get_pos(myType t) {
	int dis = -1;
	vector<myType>::iterator it;
	it = find(sites.begin(), sites.end(), t);
	dis = distance(sites.begin(), it);
	if (dis == sites.size())
		cerr << "get_pos wrong" << endl;
	return dis;
}