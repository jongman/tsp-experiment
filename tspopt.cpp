#include<cassert>
#include<algorithm>
#include<map>
#include<bitset>
#include<cstring>
#include<cstdio>
#include<cmath>
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
#include<limits>
using namespace std;

const int INT_MAX = numeric_limits<int>::max();
const int MAX_N = 101;


// 여행하는 외판원 문제의 입력을 정의한다
struct TSPProblem {
	int n;
	double dist[MAX_N][MAX_N];
};

TSPProblem read(istream& inp) {
	TSPProblem problem;
	inp >> problem.n;
	vector<double> y(problem.n), x(problem.n);
	for(int i = 0; i < problem.n; i++)
		inp >> y[i] >> x[i];
	for(int i = 0; i < problem.n; i++) {
		for(int j = 0; j < problem.n; j++)
			problem.dist[i][j] = hypot(y[i] - y[j], x[i] - x[j]);
	}
	return problem;
}


// 현재 상태가 주어질 때 종료 상태까지 거리의 하한을 계산한다
struct Estimator {
	virtual void init(const TSPProblem& problem) {
	}
	virtual double estimate(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) = 0;
};

// 현재 상태가 주어질 때 더 탐색할 필요가 있는지 없는지를 반환한다
struct Pruner {
	virtual void init(const TSPProblem& problem) {
	}
	virtual bool prune(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		return false;
	}
};


// 현재 상태가 주어질 때 어느 순서로 다음 정점을 시도해야 할 지를 반환한다
// 기본 구현은 0 부터 n-1 까지 순서대로 방문하는 것
struct OrderSelector {
	vector<vector<int> > ret;
	virtual void init(const TSPProblem& problem) {
		ret.clear();
		ret.resize(problem.n);
		for(int i = 0; i < problem.n; i++)
			for(int j = 0; j < problem.n; j++)
				if(i != j)
					ret[i].push_back(j);
	}
	virtual vector<int> getOrder(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length, double minLength) {
		return ret[path.back()];
	}
};

// 탐색의 끝에 도달했는지를 확인하고, 끝에 도달했다면 완전한 경로의 길이를 반환한다
struct FinishChecker {
	virtual void init(const TSPProblem& problem) {}
	virtual pair<bool,double> isFinished(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		if(path.size() == problem.n) return make_pair(true, length);
		return make_pair(false, length);
	}
};

struct BaseSolver {

	vector<Estimator*> estimators;
	OrderSelector* orderSelector;
	FinishChecker* finishChecker;
	virtual void init(const TSPProblem& problem) {
		for(int i = 0; i < estimators.size(); ++i)
			estimators[i]->init(problem);

		if(!orderSelector)
			orderSelector = new OrderSelector();
		orderSelector->init(problem);

		if(!finishChecker)
			finishChecker = new FinishChecker();
		finishChecker->init(problem);
	}

	void setOrderSelector(OrderSelector* selector) {
		orderSelector = selector;
	}

	void addEstimator(Estimator* estimator) {
		estimators.push_back(estimator);
	}

	void setFinishChecker(FinishChecker* checker) {
		finishChecker = checker;
	}


	virtual double solve(const TSPProblem& problem) = 0;

};

struct DepthFirstSolver: public BaseSolver {
	double minLength;

	bool prune(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		for(int i = 0; i < estimators.size(); i++)
			if(estimators[i]->estimate(problem, path, visited, length) >= minLength)
				return true;
		return false;
	}

	void dfs(const TSPProblem& problem, vector<int>& path, bitset<MAX_N>& visited, double length) {
		if(prune(problem, path, visited, length)) return;
		pair<bool, double> isFinished = finishChecker->isFinished(problem, path, visited, length);
		if(isFinished.first) {
			minLength = min(minLength, isFinished.second);
			return;
		}

		int here = path.back();
		vector<int> order = orderSelector->getOrder(problem, path, visited, length, minLength);
		for(int i = 0; i < order.size(); ++i) {
			int next = order[i];
			if(visited[next]) continue;

			visited[next].flip();
			path.push_back(next);

			dfs(problem, path, visited, length + problem.dist[here][next]);

			path.pop_back();
			visited[next].flip();
		}
	}

	virtual double solve(const TSPProblem& problem) {
		bitset<MAX_N> visited;

		vector<int> path;
		path.reserve(problem.n);

		minLength = 1e200;
		for(int start = 0; start < problem.n; ++start) {
			visited.flip(start);
			path.push_back(start);
			dfs(problem, path, visited, 0.0);
			path.pop_back();
			visited.flip(start);
		}
		return minLength;
	}
};
/*

struct IDSSolver: public DepthFirstSolver {


	virtual double solve() {
		bitset<MAX_N> visited;

		vector<int> path;
		path.reserve(n);

		double initialGuess = 1;
		//for(int i = 0; i < estimators.size(); ++i)
		//	initialGuess = max(initialGuess, estimators[i]->estimate(path, visited, 0));
		for(double guess = initialGuess; ; guess *= 2) {
			minLength = guess;
			for(int start = 0; start < n; ++start) {
				visited.flip(start);
				path.push_back(start);
				dfs(path, visited, 0.0);
				path.pop_back();
				visited.flip(start);
			}
			if(minLength < guess) break;
		}
		return minLength;
	}
};
struct IDAStarSolver: public BaseSolver {

	double estimate(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		double ret = 0;
		for(int i = 0; i < estimators.size(); ++i)
			ret = max(ret, estimators[i]->estimate(path, visited, length));
		return ret;
	}

	pair<double,double> dfs(vector<int>& path, bitset<MAX_N>& visited, double pathLimit, double length) {
		double lowerBound = estimate(path, visited, length);
		if(lowerBound > pathLimit) return make_pair(INFINITY, lowerBound);
		if(path.size() == n) return make_pair(length, pathLimit);

		double best = INFINITY, nextPathLimit = INFINITY;
		int here = path.back();

		vector<int> order = orderSelector->getOrder(path, visited, length, pathLimit);
		for(int i = 0; i < order.size(); ++i) {
			int next = order[i];
			if(visited[next]) continue;

			visited[next].flip();
			path.push_back(next);

			pair<double,double> cand = dfs(path, visited, pathLimit, length + dist[here][next]);
			best = min(best, cand.first);
			nextPathLimit = min(nextPathLimit, cand.second);

			path.pop_back();
			visited[next].flip();
		}
		return make_pair(best, nextPathLimit);
	}

	virtual double solve() {
		bitset<MAX_N> visited;

		vector<int> path;
		path.reserve(n);

		double pathLimit = 0;
		while(true) {
			double best = INFINITY, nextPathLimit = INFINITY;
			for(int start = 0; start < n; start++) {
				visited.flip(start);
				path.push_back(start);

				pair<double,double> cand = dfs(path, visited, pathLimit, 0.0);
				if(cand.first < 1e200) return cand.first;
				pathLimit = cand.second;
				path.pop_back();
				visited.flip(start);
			}
		}
	}
};
*/

struct Optimizer {
	virtual double optimize(const TSPProblem& problem, vector<int>& path, double curLength) = 0;
};

struct TwoOptimizer : public Optimizer {

	virtual double optimize(const TSPProblem& problem, vector<int>& path, double curLength) {
		//printf("Optimizing from %g ..\n", curLength);
		while(true) {
			bool improved = false;
			for(int i = 0; i < path.size(); i++) {
				for(int j = i+2; j+1 < path.size(); j++) {
					// (.. path[i]) + (path[j] .. path[i+1]) + (path[j+1] ..)
					double delta = problem.dist[path[i]][path[j]] + problem.dist[path[i+1]][path[j+1]]
						- problem.dist[path[i]][path[i+1]] - problem.dist[path[j]][path[j+1]];
					if(delta < -1e-9) {
						//printf("went down by %g\n", delta);
						curLength += delta;
						improved = true;
						reverse(path.begin() + i + 1, path.begin() + j + 1);
						break;
					}
				}
				if(improved) break;
			}
			if(!improved) break;
		}
		//printf("Resulted %g\n", curLength);
		return curLength;
	}

};

struct SwapOptimizer: public Optimizer {

	virtual double optimize(const TSPProblem& problem, vector<int>& path, double curLength) {
		//printf("Optimizing from %g ..\n", curLength);
		while(true) {
			bool improved = false;
			for(int i = 0; i < path.size(); i++) {
				for(int j = i+1; j < path.size(); j++) {
					int A = path[i], B = path[j];
					double delta = 0;
					if(i > 0)
						delta = delta - problem.dist[path[i-1]][A] + problem.dist[path[i-1]][B];
					if(j + 1 < path.size())
						delta = delta - problem.dist[B][path[j+1]] + problem.dist[A][path[j+1]];
					if(i + 1 < j)
						delta = delta - problem.dist[A][path[i+1]] + problem.dist[B][path[i+1]]
							- problem.dist[path[j-1]][B] + problem.dist[path[j-1]][A];
					if(delta < 0) {
						//printf("went down by %g\n", delta);
						curLength += delta;
						improved = true;
						swap(path[i], path[j]);
						break;
					}
				}
				if(improved) break;
			}
			if(!improved) break;
		}
		//printf("Resulted %g\n", curLength);
		return curLength;
	}

};

struct OptimizingFinishChecker: public FinishChecker {


	vector<Optimizer*> optimizers;

	void addOptimizer(Optimizer* optimizer) {
		optimizers.push_back(optimizer);
	}

	virtual pair<bool,double> isFinished(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		if(path.size() < problem.n) return make_pair(false, length);
		vector<int> optimized = path;
		for(int i = 0; i < optimizers.size(); i++) {
			length = optimizers[i]->optimize(problem, optimized, length);
		}

		return make_pair(true, length);
	}

};

struct MemoizingFinishChecker : public FinishChecker {

	int bino[MAX_N][MAX_N];
	int stateLimit, cacheDepth;
	vector<vector<double> > cache[MAX_N];

	MemoizingFinishChecker(int stateLimit) : stateLimit(stateLimit) {
	}

	void calcBino(const TSPProblem& problem) {
		memset(bino, 0, sizeof(bino));
		for(int i = 0; i < MAX_N; ++i) {
			bino[i][0] = 1;
			for(int j = 1; j < i+1; j++) {
				bino[i][j] = min(stateLimit/problem.n+1, bino[i-1][j-1] + bino[i-1][j]);
			}
		}
	}

	void determineCacheDepth(const TSPProblem& problem) {
		cacheDepth = 0;
		int stateCount = problem.n;

		// determine depth of cache
		while(cacheDepth < problem.n-1) {
			int newStates = bino[problem.n][cacheDepth+1] * problem.n;
			if(stateCount + newStates > stateLimit) break;
			stateCount += newStates;
			++cacheDepth;
		}
		//fprintf(stderr, "Determined a cache depth of %d StateCount = %d\n", cacheDepth, stateCount);
	}

	int calcIndex(const TSPProblem& problem, const vector<int>& selected) {
		int ret = 0;
		for(int i = 0; i < selected.size(); ++i)
			ret += bino[problem.n - selected[i] - 1][selected.size() - i];
		return ret;
	}

	virtual void init(const TSPProblem& problem) {
		calcBino(problem);
		determineCacheDepth(problem);

		//test();

		for(int i = 0; i < problem.n; i++) {
			cache[i].clear();
			cache[i].resize(cacheDepth + 1);
			for(int j = 1; j <= cacheDepth; j++) {
				cache[i][j].resize(bino[problem.n][j], -1);
			}
		}
	}

	double solve(const TSPProblem& problem, int here, const vector<int>& toVisit) {
		if(toVisit.empty()) return 0;
		int idx = calcIndex(problem, toVisit);
		double& ret = cache[here][toVisit.size()][idx];
		if(ret >= 0) return ret;
		ret = 1e200;
		for(int i = 0; i < toVisit.size(); ++i) {
			vector<int> toVisit2 = toVisit;
			toVisit2.erase(toVisit2.begin() + i);
			ret = min(ret, problem.dist[here][toVisit[i]] + solve(problem, toVisit[i], toVisit2));
		}
		/*
		   printf("solve(here=%d, toVisit=", here);
		   for(int i = 0; i < toVisit.size(); i++) {
		   printf("%s%d", (i ? "," : ""), toVisit[i]);
		   }
		   printf(") = %g\n", ret);
		   */
		return ret;
	}

	virtual pair<bool,double> isFinished(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		if(problem.n == path.size()) return make_pair(true, length);
		if(problem.n - path.size() == cacheDepth) {
			vector<int> toVisit;
			for(int i = 0; i < problem.n; i++)
				if(!visited[i])
					toVisit.push_back(i);
			return make_pair(true, length + solve(problem, path.back(), toVisit));
		}
		return make_pair(false, length);
	}
};

struct NearestNeighborOrderSelector : public OrderSelector {

	virtual void init(const TSPProblem& problem) {
		ret.clear();
		ret.resize(problem.n);
		for(int i = 0; i < problem.n; i++) {
			vector<pair<double,int> > ord;
			for(int j = 0; j < problem.n; j++)
				if(i != j)
					ord.push_back(make_pair(problem.dist[i][j], j));
			sort(ord.begin(), ord.end());
			for(int j = 0; j < ord.size(); j++)
				ret[i].push_back(ord[j].second);
		}
	}
	virtual vector<int> getOrder(const vector<int>& path, const bitset<MAX_N>& visited, double length, double minLength) {
		return ret[path.back()];
	}
};

struct PathSwapPruner : public Pruner {

	virtual bool prune(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		if(path.size() < 4) return false;
		const int a = path[path.size()-4];
		const int b = path[path.size()-3];
		const int c = path[path.size()-2];
		const int d = path[path.size()-1];
		if(problem.dist[a][b] + problem.dist[c][d] > problem.dist[a][c] + problem.dist[b][d]) return true;
	}
};

struct PathReversePruner: public Pruner {

	virtual bool prune(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		if(path.size() < 3) return false;
		int c = path[path.size()-2];
		int d = path.back();
		for(int i = 0; i+2 < path.size(); ++i) {
			int a = path[i], b = path[i+1];
			if(problem.dist[a][c] + problem.dist[b][d] < problem.dist[a][b] + problem.dist[c][d]) return true;
		}
		return false;
	}
};

struct NaiveEstimator : public Estimator {
	virtual double estimate(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		return length;
	}
};

struct IncomingEdgeEstimator : public Estimator {

	vector<double> minEdge;
	virtual void init(const TSPProblem& problem) {
		minEdge.resize(problem.n);
		for(int i = 0; i < problem.n; i++) {
			minEdge[i] = 1e200;
			for(int j = 0; j < problem.n; j++)
				if(i != j)
					minEdge[i] = min(minEdge[i], problem.dist[i][j]);
		}
	}

	virtual double estimate(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		double lowerBound = length;
		for(int i = 0; i < problem.n; ++i)
			if(!visited[i])
				lowerBound += minEdge[i];
		return lowerBound;
	}
};

struct UnionFind
{
	int n, components;
	vector<int> parent, rank;
	UnionFind(int n) : n(n), components(n), parent(n), rank(n)
	{ for(int i = 0; i < n; ++i) { parent[i] = i; rank[i] = 0; } }
	int find(int here) { return parent[here] == here ? here : (parent[here] = find(parent[here])); }
	bool join(int a, int b)
	{
		a = find(a); b = find(b);
		if(a == b) return false;
		if(rank[a] > rank[b])
			parent[b] = a;
		else
		{
			parent[a] = b;
			if(rank[a] == rank[b]) rank[b]++;
		}
		--components;
		return true;
	}
};

struct MSTEstimator: public Estimator {

	vector<pair<double,pair<int,int> > > edges;

	virtual void init(const TSPProblem& problem) {
		edges.clear();
		for(int i = 0; i < problem.n; ++i)
			for(int j = i+1; j < problem.n; j++)
				edges.push_back(make_pair(problem.dist[i][j], make_pair(i, j)));
		sort(edges.begin(), edges.end());
	}

	double getLowerBound(const TSPProblem& problem, int here, const bitset<MAX_N>& visited) {
		UnionFind* uf = new UnionFind(problem.n);
		double taken = 0;
		vector<vector<int> > adj(problem.n);
		for(int i = 0; i < edges.size(); i++) {
			int a = edges[i].second.first, b = edges[i].second.second;
			if(a != here && visited[a]) continue;
			if(b != here && visited[b]) continue;
			a = uf->find(a); b = uf->find(b);
			if(a != b) {
				taken += edges[i].first;
				adj[a].push_back(b);
				adj[b].push_back(a);
				uf->join(a, b);
			}
		}
		delete uf;
		return taken;
	}

	virtual double estimate(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length) {
		return length + getLowerBound(problem, path.empty() ? -1 : path.back(), visited);
	}
};

struct EstimatingOrderSelector : public OrderSelector {

	Estimator *estimator;

	EstimatingOrderSelector(Estimator* estimator) : estimator(estimator) {
	}

	virtual void init(const TSPProblem& problem) {
		estimator->init(problem);
	}

	virtual vector<int> getOrder(const TSPProblem& problem, const vector<int>& path, const bitset<MAX_N>& visited, double length, double minLength) {
		vector<pair<double,int> > ord;
		vector<int> newPath = path;
		newPath.push_back(-1);
		bitset<MAX_N> newVisited = visited;
		for(int i = 0; i < problem.n; i++) {
			if(visited[i]) continue;
			newPath.back() = i;
			newVisited.flip(i);

			ord.push_back(make_pair(estimator->estimate(problem, newPath, newVisited, length + problem.dist[path.back()][i]), i));

			newVisited.flip(i);
		}
		sort(ord.begin(), ord.end());
		vector<int> ret;
		for(int i = 0; i < ord.size(); i++) {
			if(ord[i].first < minLength)
				ret.push_back(ord[i].second);
		}
		return ret;
	}
};

map<string, BaseSolver*> solvers;

void setupSolvers() {

	// SETUP SELECTORS
	vector<string> selectorNames;
	vector<OrderSelector*> selectors;

	selectorNames.push_back("Default");
	selectors.push_back(new OrderSelector());

	selectorNames.push_back("Nearest");
	selectors.push_back(new NearestNeighborOrderSelector());

	selectorNames.push_back("IncomingEdge");
	selectors.push_back(new EstimatingOrderSelector( new IncomingEdgeEstimator()));

	selectorNames.push_back("MST");
	selectors.push_back(new EstimatingOrderSelector( new MSTEstimator()));

	// SETUP FINISHCHECKERS
	vector<string> checkerNames;
	vector<FinishChecker*> checkers;

	checkerNames.push_back("Default");
	checkers.push_back(new FinishChecker());

	checkerNames.push_back("Memoization");
	checkers.push_back(new MemoizingFinishChecker(500000));

	{
		checkerNames.push_back("Optimizing1");
		OptimizingFinishChecker* opt = new OptimizingFinishChecker();
		opt->addOptimizer(new SwapOptimizer());
		//opt->addOptimizer(new TwoOptimizer());
		checkers.push_back(opt);
	}

	{
		checkerNames.push_back("Optimizing2");
		OptimizingFinishChecker* opt = new OptimizingFinishChecker();
		opt->addOptimizer(new SwapOptimizer());
		opt->addOptimizer(new TwoOptimizer());
		checkers.push_back(opt);
	}

	// SETUP ESTIMATORS
	vector<string> estimatorNames;
	vector<Estimator*> estimators;

	estimatorNames.push_back("Naive");
	estimators.push_back(new NaiveEstimator());

	estimatorNames.push_back("IncomingEdge");
	estimators.push_back(new IncomingEdgeEstimator());

	estimatorNames.push_back("MST");
	estimators.push_back(new MSTEstimator());

	int s = selectors.size();
	int c = checkers.size();
	int p = estimators.size();
	for(int selector = 0; selector < s; ++selector) {
		string base = ":" + selectorNames[selector] + ":";
		for(int checker = 0; checker < c; ++checker) {
			string base2 = base + checkerNames[checker] + ":";
			for(int estimatorSet = 0; estimatorSet < (1<<p); ++estimatorSet) {
				{
					/* Setup DepthFirstSolver */
					string name = base2;
					DepthFirstSolver* solver = new DepthFirstSolver();
					solver->setOrderSelector(selectors[selector]);
					solver->setFinishChecker(checkers[checker]);
					for(int i = 0; i < p; i++) if(estimatorSet & (1<<i)) {
						if(name[name.size()-1] != ':')
							name += ',';
						name += estimatorNames[i];
						solver->addEstimator(estimators[i]);
					}
					solvers["DFS" + name] = solver;
				}
				/*
				{
					/* Setup IDSSolver *
					string name = base2;
					IDSSolver* solver = new IDSSolver();
					solver->setOrderSelector(selectors[selector]);
					solver->setFinishChecker(checkers[checker]);
					for(int i = 0; i < p; i++) if(estimatorSet & (1<<i)) {
						if(name[name.size()-1] != ':')
							name += ',';
						name += estimatorNames[i];
						solver->addEstimator(estimators[i]);
					}
					solvers["IDS" + name] = solver;
				}
				*/
			}
		}
	}
}

double solve(const string& algorithm, const TSPProblem& problem) {
	if(solvers.count(algorithm) == 0)
		return -1;
	BaseSolver* s = solvers[algorithm];
	s->init(problem);
	return s->solve(problem);
}

int main(int argc, char* argv[]) {
	setupSolvers();

	if(argc != 4) {
		printf("Usage) %s [algorithm] [input] [output]\n\n", argv[0]);
		printf("algorithm = ");
		bool first = true;
		for(map<string,BaseSolver*>::iterator it = solvers.begin(); it != solvers.end(); ++it) {
			if(!first) printf(" | ");
			first = false;
			printf("%s", it->first.c_str());
		}
		printf("\n");
		return 0;
	}
	ifstream inp(argv[2]);
	ofstream outp(argv[3]);
	int cases;
	inp >> cases;
	for(int i = 0; i < cases; ++i) {
		outp << solve(argv[1], read(inp)) << endl;
	}
}

