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

int n;
double y[MAX_N], x[MAX_N], dist[MAX_N][MAX_N];

void read(istream& inp) {
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
		virtual bool prune(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			return false;
		}
};

class Estimator {
	public:
		virtual void init() {}
		virtual double estimate(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			return 0;
		}
};

class OrderSelector {
	public:
		vector<vector<int> > ret;
		virtual void init() {
			ret.clear();
			ret.resize(n);
			for(int i = 0; i < n; i++)
				for(int j = 0; j < n; j++)
					if(i != j)
						ret[i].push_back(j);
		}
		virtual vector<int> getOrder(const vector<int>& path, const bitset<MAX_N>& visited, double length, double minLength) {
			return ret[path.back()];
		}
};

class FinishChecker {
	public:
		virtual void init() {}
		virtual pair<bool,double> isFinished(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			if(path.size() == n) return make_pair(true, length);
			return make_pair(false, length);
		}
};

class BaseSolver: public Solver {
	public:
		vector<Estimator*> estimators;
		OrderSelector* orderSelector;
		FinishChecker* finishChecker;
		virtual void init() {
			for(int i = 0; i < estimators.size(); ++i)
				estimators[i]->init();

			if(!orderSelector)
				orderSelector = new OrderSelector();
			orderSelector->init();

			if(!finishChecker)
				finishChecker = new FinishChecker();
            finishChecker->init();
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


};

class DFSSolver: public BaseSolver {
	public:

		int renewed;
		double minLength;

		bool prune(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			for(int i = 0; i < estimators.size(); i++)
				if(estimators[i]->estimate(path, visited, length) >= minLength)
					return true;
			return false;
		}

		void dfs(vector<int>& path, bitset<MAX_N>& visited, double length) {
			if(prune(path, visited, length)) return;
			pair<bool, double> isFinished = finishChecker->isFinished(path, visited, length);
			if(isFinished.first) {
				++renewed;
				minLength = min(minLength, isFinished.second);
				return;
			}

			int here = path.back();
			vector<int> order = orderSelector->getOrder(path, visited, length, minLength);
			for(int i = 0; i < order.size(); ++i) {
				int next = order[i];
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

			renewed = 0;
			minLength = 1e200;
			for(int start = 0; start < n; ++start) {
				visited.flip(start);
				path.push_back(start);
				dfs(path, visited, 0.0);
				path.pop_back();
				visited.flip(start);
			}
			//fprintf(stderr, "Optimal solution was renewed %d times\n", renewed);
			return minLength;
		}
};

class IDSSolver: public DFSSolver {
	public:

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
class IDAStarSolver: public BaseSolver {
	public:
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

class Optimizer {
	public:
		virtual double optimize(vector<int>& path, double curLength) = 0;
};

class TwoOptimizer : public Optimizer {
	public:
		virtual double optimize(vector<int>& path, double curLength) {
			//printf("Optimizing from %g ..\n", curLength);
			while(true) {
				bool improved = false;
				for(int i = 0; i < path.size(); i++) {
					for(int j = i+2; j+1 < path.size(); j++) {
						// (.. path[i]) + (path[j] .. path[i+1]) + (path[j+1] ..)
						double delta = dist[path[i]][path[j]] + dist[path[i+1]][path[j+1]]
							- dist[path[i]][path[i+1]] - dist[path[j]][path[j+1]];
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

class SwapOptimizer: public Optimizer {
	public:
		virtual double optimize(vector<int>& path, double curLength) {
			//printf("Optimizing from %g ..\n", curLength);
			while(true) {
				bool improved = false;
				for(int i = 0; i < path.size(); i++) {
					for(int j = i+1; j < path.size(); j++) {
						int A = path[i], B = path[j];
						double delta = 0;
						if(i > 0)
							delta = delta - dist[path[i-1]][A] + dist[path[i-1]][B];
						if(j + 1 < path.size())
							delta = delta - dist[B][path[j+1]] + dist[A][path[j+1]];
						if(i + 1 < j)
							delta = delta - dist[A][path[i+1]] + dist[B][path[i+1]]
								- dist[path[j-1]][B] + dist[path[j-1]][A];
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

class OptimizingFinishChecker: public FinishChecker {
	public:

		vector<Optimizer*> optimizers;

		void addOptimizer(Optimizer* optimizer) {
			optimizers.push_back(optimizer);
		}

		virtual pair<bool,double> isFinished(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			if(path.size() < n) return make_pair(false, length);
			vector<int> optimized = path;
			for(int i = 0; i < optimizers.size(); i++) {
				length = optimizers[i]->optimize(optimized, length);
			}

			return make_pair(true, length);
		}

};

class MemoizingFinishChecker : public FinishChecker {
	public:
		int bino[MAX_N][MAX_N];
		int stateLimit, cacheDepth;
		vector<vector<double> > cache[MAX_N];

		MemoizingFinishChecker(int stateLimit) : stateLimit(stateLimit) {
		}

		void calcBino() {
			memset(bino, 0, sizeof(bino));
			for(int i = 0; i < MAX_N; ++i) {
				bino[i][0] = 1;
				for(int j = 1; j < i+1; j++) {
					bino[i][j] = min(stateLimit/n+1, bino[i-1][j-1] + bino[i-1][j]);
				}
			}
		}

		void determineCacheDepth() {
			cacheDepth = 0;
			int stateCount = n;

			// determine depth of cache
			while(cacheDepth < n-1) {
				int newStates = bino[n][cacheDepth+1] * n;
				if(stateCount + newStates > stateLimit) break;
				stateCount += newStates;
				++cacheDepth;
			}
		    //fprintf(stderr, "Determined a cache depth of %d StateCount = %d\n", cacheDepth, stateCount);
		}

		int calcIndex(const vector<int>& selected) {
			int ret = 0;
			for(int i = 0; i < selected.size(); ++i)
				ret += bino[n - selected[i] - 1][selected.size() - i];
			return ret;
		}

		void test() {
			for(int levels = 1; levels <= cacheDepth; ++levels) {
				map<int,int> seen;
				for(int mask = (1LL<<n)-1; mask >= 0; --mask) {
					if(__builtin_popcount(mask) == levels) {
						vector<int> selected;
						for(int i = 0; i < n; i++) {
							if(mask & (1LL<<i))
								selected.push_back(i);
						}
						printf("level = %d mask = %d selected =", levels, mask);
						for(int i = 0; i < selected.size(); i++) {
							printf(" %d", selected[i]);
						}
						int got = calcIndex(selected);
						printf(" got = %d\n", got);
						if(got < 0 || got >= bino[n][levels]) {
							printf("OVERFLOW!!\n");
							printf("bino[%d][%d] = %d, got = %d\n", n, levels, bino[n][levels], got);
							throw 1;
						}
						if(seen.count(got)) {
							printf("CLASH!!!\n");
							printf("Seen[%d] is already taken by mask=%d\n", got, seen[got]);
							throw 1;
						}
						seen[got] = mask;
					}
				}
			}
		}

		virtual void init() {
			calcBino();
			determineCacheDepth();

			//test();

			for(int i = 0; i < n; i++) {
				cache[i].clear();
				cache[i].resize(cacheDepth + 1);
				for(int j = 1; j <= cacheDepth; j++) {
					cache[i][j].resize(bino[n][j], -1);
				}
			}
		}

		double solve(int here, const vector<int>& toVisit) {
			if(toVisit.empty()) return 0;
			int idx = calcIndex(toVisit);
			double& ret = cache[here][toVisit.size()][idx];
			if(ret >= 0) return ret;
			ret = 1e200;
			for(int i = 0; i < toVisit.size(); ++i) {
				vector<int> toVisit2 = toVisit;
				toVisit2.erase(toVisit2.begin() + i);
				ret = min(ret, dist[here][toVisit[i]] + solve(toVisit[i], toVisit2));
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

		virtual pair<bool,double> isFinished(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			if(n == path.size()) return make_pair(true, length);
			if(n - path.size() == cacheDepth) {
				vector<int> toVisit;
				for(int i = 0; i < n; i++)
					if(!visited[i])
						toVisit.push_back(i);
				return make_pair(true, length + solve(path.back(), toVisit));
			}
			return make_pair(false, length);
		}
};

class NearestNeighborOrderSelector : public OrderSelector {
	public:
		virtual void init() {
			ret.clear();
			ret.resize(n);
			for(int i = 0; i < n; i++) {
				vector<pair<double,int> > ord;
				for(int j = 0; j < n; j++)
					if(i != j)
						ord.push_back(make_pair(dist[i][j], j));
				sort(ord.begin(), ord.end());
				for(int j = 0; j < ord.size(); j++)
					ret[i].push_back(ord[j].second);
			}
		}
		virtual vector<int> getOrder(const vector<int>& path, const bitset<MAX_N>& visited, double length, double minLength) {
			return ret[path.back()];
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
			if(dist[a][b] + dist[c][d] > dist[a][c] + dist[b][d]) return true;
		}
};

class PathReversePruner: public Pruner {
	public:
		virtual bool prune(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			if(path.size() < 3) return false;
			int c = path[path.size()-2];
			int d = path.back();
			for(int i = 0; i+2 < path.size(); ++i) {
				int a = path[i], b = path[i+1];
				if(dist[a][c] + dist[b][d] < dist[a][b] + dist[c][d]) return true;
			}
			return false;
		}
};

class NaiveEstimator : public Estimator {
	public:
		virtual double estimate(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			return length;
		}
};

class IncomingEdgeEstimator : public Estimator {
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

		virtual double estimate(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			double lowerBound = length;
			for(int i = 0; i < n; ++i)
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

class MSTEstimator: public Estimator {
	public:
		vector<pair<double,pair<int,int> > > edges;

		virtual void init() {
			edges.clear();
			for(int i = 0; i < n; ++i)
				for(int j = i+1; j < n; j++)
					edges.push_back(make_pair(dist[i][j], make_pair(i, j)));
			sort(edges.begin(), edges.end());
		}

		double getLowerBound(int here, const bitset<MAX_N>& visited) {
			UnionFind* uf = new UnionFind(n);
			double taken = 0;
			vector<vector<int> > adj(n);
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

		virtual double estimate(const vector<int>& path, const bitset<MAX_N>& visited, double length) {
			return length + getLowerBound(path.empty() ? -1 : path.back(), visited);
		}
};

class EstimatingOrderSelector : public OrderSelector {
	public:
		Estimator *estimator;

		EstimatingOrderSelector(Estimator* estimator) : estimator(estimator) {
		}

		virtual void init() {
			estimator->init();
		}

		virtual vector<int> getOrder(const vector<int>& path, const bitset<MAX_N>& visited, double length, double minLength) {
			vector<pair<double,int> > ord;
			vector<int> newPath = path;
			newPath.push_back(-1);
			bitset<MAX_N> newVisited = visited;
			for(int i = 0; i < n; i++) {
				if(visited[i]) continue;
				newPath.back() = i;
				newVisited.flip(i);

				ord.push_back(make_pair(estimator->estimate(newPath, newVisited, length + dist[path.back()][i]), i));

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

map<string, Solver*> solvers;

void setupSolvers() {
	solvers["Dummy"] = new DummySolver();


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
					/* Setup DFSSolver */
					string name = base2;
					DFSSolver* solver = new DFSSolver();
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
				{
					/* Setup IDSSolver */
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
			}
		}
	}
}

double solve(const string& algorithm) {
	if(solvers.count(algorithm) == 0)
		return -1;
	Solver* s = solvers[algorithm];
	s->init();
	return s->solve();
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
	ifstream inp(argv[2]);
	ofstream outp(argv[3]);
	int cases;
	inp >> cases;
	for(int i = 0; i < cases; ++i) {
		read(inp);
		outp << solve(argv[1]) << endl;
	}
}

