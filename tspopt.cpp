#include<map>
#include<bitset>
#include<cstring>
#include<cstdio>
#include<cmath>
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
using namespace std;

const int MAX_N = 101;

int n;
double y[MAX_N], x[MAX_N], dist[MAX_N][MAX_N];

void read(const char* fileName) {
	ifstream inp(fileName);
	inp >> n;
	for(int i = 0; i < n; i++) inp >> y[i] >> x[i];
	for(int i = 0; i < n; i++) {
		for(int j = 0; j < n; j++)
			dist[i][j] = hypot(y[i] - y[j], x[i] - x[j]);
		dist[n][i] = 0;
	}
}

class Solver {
	public:
		virtual void init() {}
		virtual double solve() { return -1; }
};

class DummySolver: public Solver {
	public:
		virtual double solve() { return 0; }
};

class DFSSolver;

class Pruner {
	public:
		virtual void init() {}
		virtual bool prune(const DFSSolver& solver, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			return false;
		}
};

class DFSSolver: public Solver {
	public:

		double minLength;
		vector<int> minPath;
		vector<Pruner*> pruners;

		virtual void init() {
			for(int i = 0; i < pruners.size(); ++i)
				pruners[i]->init();
		}

		void addPruner(Pruner* pruner) {
			pruners.push_back(pruner);
		}

		bool prune(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			for(int i = 0; i < pruners.size(); i++)
				if(pruners[i]->prune(*this, path, visited, length))
					return true;
			return false;
		}

		void dfs(vector<int>& path, bitset<MAX_N>& visited, double length) {
			if(prune(path, visited, length)) return;
			if(path.size() == n) {
				if(length < minLength) {
					minLength = length;
					minPath = path;
				}
				return;
			}

			int here = path.back();
			for(int next = 0; next < n; ++next) {
				if(visited[next]) continue;

				visited[next].flip();
				path.push_back(next);

				dfs(path, visited, length + dist[here][next]);

				path.pop_back();
				visited[next].flip();
			}
		}

		virtual double solve() {
			bitset<MAX_N> visited;

			vector<int> path;
			path.reserve(n);

			minLength = 1e200;
			for(int start = 0; start < n; ++start) {
				visited.flip(start);
				path.push_back(start);
				dfs(path, visited, 0.0);
				path.pop_back();
				visited.flip(start);
			}
			return minLength;
		}
};

class NaivePruner : public Pruner {
	public:
		virtual bool prune(const DFSSolver& solver, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			return length >= solver.minLength;
		}
};

class PathSwapPruner : public Pruner {
	public:
		virtual bool prune(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			if(path.size() < 4) return false;
			const int a = path[path.size()-4];
			const int b = path[path.size()-3];
			const int c = path[path.size()-2];
			const int d = path[path.size()-1];
			return dist[a][b] + dist[c][d] > dist[a][c] + dist[b][d];
		}
};

class LowerBoundPruner : public Pruner {
	public:
		vector<double> minEdge;
		virtual void init() {
			minEdge.resize(n);
			for(int i = 0; i < n; i++) {
				minEdge[i] = 1e200;
				for(int j = 0; j < n; j++)
					if(i != j)
						minEdge[i] = min(minEdge[i], dist[i][j]);
			}
		}

		virtual bool prune(const DFSSolver& solver, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			double lowerBound = length;
			for(int i = 0; i < n; ++i)
				if(!visited[i])
					lowerBound += minEdge[i];

			return lowerBound >= solver.minLength;
		}
};

map<string, Solver*> solvers;

void setupSolvers() {
	solvers["Dummy"] = new DummySolver();

	vector<string> names;
	vector<Pruner*> pruners;

	names.push_back("Naive");
	pruners.push_back(new NaivePruner());

	names.push_back("Path");
	pruners.push_back(new PathSwapPruner());

	names.push_back("LowerBound");
	pruners.push_back(new LowerBoundPruner());

	int m = pruners.size();
	for(int i = 0; i < (1<<m); ++i) {
		string name = "DFS";
		for(int j = 0; j < m; j++) if(i & (1<<j)) name += names[j];
		DFSSolver* solver = new DFSSolver();
		for(int j = 0; j < m; j++) if(i & (1<<j)) solver->addPruner(pruners[j]);
		solvers[name] = solver;
	}
}

void solve(const string& algorithm, const char* fileName) {
	ofstream outp(fileName);
	if(solvers.count(algorithm) == 0)
		outp << -1 << endl;
	else {
		Solver* s = solvers[algorithm];
		s->init();
		outp << s->solve() << endl;
	}
}

int main(int argc, char* argv[]) {
	setupSolvers();

	if(argc != 4) {
		printf("Usage) %s [algorithm] [input] [output]\n\n", argv[0]);
		printf("algorithm = ");
		bool first = true;
		for(map<string,Solver*>::iterator it = solvers.begin(); it != solvers.end(); ++it) {
			if(!first) printf(" | ");
			first = false;
			printf("%s", it->first.c_str());
		}
		printf("\n");
		return 0;
	}
	read(argv[2]);
	solve(argv[1], argv[3]);
}

