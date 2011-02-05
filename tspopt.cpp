#include<cstring>
#include<cstdio>
#include<cmath>
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
using namespace std;

int n;
double y[100], x[100], dist[100][100];

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

Solver* solvers[] = { new DummySolver(), NULL };

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

