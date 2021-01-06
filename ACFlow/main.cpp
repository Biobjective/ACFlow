#include"path.h"
#include "Graph.h"
#include"route_a.h"
using namespace std;
UndiGraph<myType, float> g(310);

const int m = 100;//ants number
const float beta = 2.0;//heuristic information weight 
int G = 50;//��������
const float rho = 0.3;//pheromone violation factor 
const float xik = 0.1;//
const float pher_start = 0.025;
const float v_start = 100;
const int w = 3;//number of ants allowed to release pheromone 
const int cap = 20;//capacity of each road
int traffic_time = 1; // ���Լ����ٶȵ�ʱ��
const float heur_strat = 0;

const int target = 48;//�趨��ʼ�̶�Դ�ڵ�  48//521
vector<myType> source(m, target);//ÿֻ���ϵ�Դ�ڵ�  521
vector<myType> safe_site;//��ȫ�Ľڵ�  �� Ŀ�Ľڵ㼯��
vector<myType> sites;//ȫ���ڵ�ļ���
vector<bool> ant_dest(m, false);//�Ƿ񵽴��յ��ʶ
vector<vector<myType> > ant_solution(m);
vector<unordered_map<myType, float> > flow_mtx;//flow hash function

vector<vector<pair<myType, float> > > heur_mtx;//heuristic information 
vector<vector<pair<myType, float> > > trans_mtx;//Transition matrix
vector<vector<pair<myType, float> > > pher_mtx;//Pheromone matrix

set<pair<int, int> > cur_global_route;
vector<vector<pair<myType, pair<int, int> > > > density_mtx;//density matrix  ��flow ͬʱʵ�� �����Ϊ�ڵ���
vector<float> fit_mtx(m, 0);//��Ӧ�Ⱦ���
pair<float, vector<myType>> history_best;
vector<vector<myType> > transmit_p;// ���;���
vector<vector<myType> > short_p;//short path

int main() {
	//readFile("F:/Edge_Download/Data/Data/NYC/newtest.csv");
	//SetConsoleTitle("ACEP");
	g.InputfileGraph("Dj.txt");//   C:/Users/shijian/Desktop/   sites.txt(2020/7/15֮ǰ) Dj.txt
	g.Display();
	new_init();
	auto k = g.getedgematrix();
	int num = g.getnumVertexes();
	vector<int> d(num);
	vector<int> p(num, -1);
	dijkstra(num, target, p, d, k);//������С������
	short_p = shortpath_construct(target, num, p);//���·����������ʽ  
	//vector<vector<myType> > res;
	if (graphPreprocess(target, num, short_p, transmit_p))  cout << "graphPreprocess success\n";
	else cout << "graphPreprocess failed\n";
	FILE* stream2;
	freopen_s(&stream2, "res4.txt", "w", stdout);//����ض���
	//g.Display();
	for (int cnts = 0; G; cnts++) {
		//freopen_s(&stream2, "CON", "w", stdout);//����ض���� ����̨
		routeConstricution();
		updateGlobalPher();
		updateTrans();
		float sum = meanCompute();
		//cout << '\a' << endl;
		//freopen_s(&stream2, "res2.txt", "w", stdout);//����ض���
		cout << sum << endl;
		//cout << cnts<<":"<<meanCompute() << endl;
	}
	freopen_s(&stream2, "CON", "w", stdout);//����ض���� ����̨
	cout << '\a';
	return 0;
}
