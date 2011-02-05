#include<bitset>
#include<cstring>
#include<cstdio>
#include<cmath>
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
using namespace std;

const int MAX_N = 100;

int n;
double y[MAX_N], x[MAX_N], dist[MAX_N][MAX_N];

void read(const char* fileName) {
	ifstream inp(fileName);
	inp >> n;
	for(int i = 0; i < n; i++) inp >> y[i] >> x[i];
	for(int i = 0; i < n; i++)
		for(int j = 0; j < n; j++)
			dist[i][j] = hypot(y[i] - y[j], x[i] - x[j]);
}

class Solver {
	public:
		virtual string getName() const { return ""; }
		virtual double solve() { return -1; }
};

class DummySolver: public Solver {
	public:
		virtual string getName() const { return "Dummy"; }
		virtual double solve() { return 0; }
};

class DFSSolver: public Solver {
	public:
		virtual string getName() const { return "DFS"; }

		double dfs(int here, bitset<MAX_N>& visited) {
			if(visited.count() == n) return 0;
			double ret = 1e200;
			for(int there = 0; there < n; ++there) {
				if(visited[there]) continue;
				visited[there].flip();
				ret = min(ret, dist[here][there] + dfs(there, visited));
				visited[there].flip();
			}
			return ret;
		}

		virtual double solve() {
			bitset<MAX_N> visited;

			double ret = 1e200;
			for(int start = 0; start < n; ++start) {
				visited.flip(start);
				ret = min(ret, dfs(start, visited));
				visited.flip(start);
			}
			return ret;
		}
};

Solver* solvers[] = { new DummySolver(), new DFSSolver(), NULL };

void solve(const char* algorithm, const char* fileName) {
	ofstream outp(fileName);
	for(int i = 0; solvers[i]; ++i)
		if(solvers[i]->getName() == algorithm) {
			outp << solvers[i]->solve() << endl;
			return;
		}
	outp << -1 << endl;
}

int main(int argc, char* argv[]) {
	if(argc != 4) {
		printf("Usage) %s [algorithm] [input] [output]\n\n", argv[0]);
		printf("algorithm = ");
		for(int i = 0; solvers[i] != NULL; ++i) {
			if(i > 0) printf(" | ");
			printf("%s", solvers[i]->getName().c_str());
		}
		printf("\n");
		return 0;
	}
	read(argv[2]);
	solve(argv[1], argv[3]);
}

