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
#define ONLINE_JUDGE

using namespace std;

const int INT_MAX = numeric_limits<int>::max();
const int MAX_N = 30;


// 여행하는 외판원 문제의 입력을 정의한다
struct TSPProblem {
	int n;
	double dist[MAX_N][MAX_N];
};

// 여행하는 외판원 문제의 상태를 정의한다
struct TSPState {
	// 문제 입력
	const TSPProblem& problem;
	// 지금까지 방문한 (부분) 경로
	vector<int> path;
	// 각 정점을 방문했는지 여부
	bitset<MAX_N> visited;
	// 현재 경로의 길이
	double length;

	TSPState(const TSPProblem& problem) : problem(problem) {
		path.reserve(problem.n);
		visited.reset();
		length = 0;
	}

	// vertex 를 현재 경로 뒤에 추가한다
	void push(int vertex) {
		if(!path.empty()) length += problem.dist[path.back()][vertex];
		path.push_back(vertex);
		visited.set(vertex);
	}

	// 현재 경로 맨 뒤의 정점을 삭제한다
	void pop() {
		int vertex = path.back();
		path.pop_back();

		if(!path.empty()) length -= problem.dist[path.back()][vertex];
		visited.reset(vertex);
	}
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

#ifdef ONLINE_JUDGE

TSPProblem readAOJ(istream& inp) {
	TSPProblem problem;
	inp >> problem.n;
	for(int i = 0; i < problem.n; i++)
		for(int j = 0; j < problem.n; j++)
			inp >> problem.dist[i][j];
	return problem;
}

#endif

// 현재 상태가 주어질 때 종료 상태까지 거리의 하한을 계산한다
struct Estimator {
	virtual void init(const TSPProblem& problem) {
	}

	virtual double estimate(const TSPState& state) = 0;
};

// 현재 상태가 주어질 때 더 탐색할 필요가 있는지 없는지를 반환한다.
struct Pruner {
	virtual void init(const TSPProblem& problem) {
	}

	virtual bool prune(const TSPState& state, double minLength) = 0;
};


// 현재 상태가 주어질 때 어느 순서로 다음 정점을 시도해야 할 지를 반환한다.
// 기본 구현은 0 부터 n-1 까지 순서대로 방문하는 순서를 항상 반환한다.
struct OrderSelector {
	vector<int> defaultOrder;
	virtual void init(const TSPProblem& problem) {
		defaultOrder.resize(problem.n);
		for(int i = 0; i < problem.n; ++i)
			defaultOrder[i] = i;
	}
	virtual vector<int> getOrder(const TSPState& state, double minLength) {
		return defaultOrder;
	}
};

// 탐색의 끝에 도달했는지를 확인하고, 끝에 도달했다면 완전한 경로의 길이를 반환한다.
// 기본 구현은 path 의 길이가 n 과 같을 경우 탐색을 종료한다.
struct FinishChecker {
	virtual void init(const TSPProblem& problem) {
	}
	// (탐색 종료 여부, 경로의 최종 길이) 를 반환한다.
	virtual pair<bool,double> isFinished(const TSPState& state) {
		if(state.path.size() == state.problem.n)
			return make_pair(true, state.length);
		return make_pair(false, state.length);
	}
};

// TSP 문제를 해결하는 깊이 우선 탐색을 구현하는 클래스.
struct DepthFirstSolver {
	// 지금까지 찾은 최단 경로의 길이. 탐색 중 갱신된다.
	double minLength;

	// 최적화를 구현하는 객체들
	vector<Pruner*> pruners;
	OrderSelector* orderSelector;
	FinishChecker* finishChecker;

	virtual void init(const TSPProblem& problem) {
		for(int i = 0; i < pruners.size(); ++i)
			pruners[i]->init(problem);

		if(!orderSelector)
			orderSelector = new OrderSelector();
		orderSelector->init(problem);

		if(!finishChecker)
			finishChecker = new FinishChecker();
		finishChecker->init(problem);
	}

	void addPruner(Pruner* pruner) {
		pruners.push_back(pruner);
	}

	void setOrderSelector(OrderSelector* selector) {
		orderSelector = selector;
	}

	void setFinishChecker(FinishChecker* checker) {
		finishChecker = checker;
	}

	// Pruner 들 중 하나라도 탐색을 중단하라고 하는지 확인한다
	bool prune(const TSPState& state) {
		for(int i = 0; i < pruners.size(); ++i)
			if(pruners[i]->prune(state, minLength))
				return true;
		return false;
	}

	void dfs(TSPState& state) {
		// 탐색을 중단할지 확인한다
		if(prune(state)) return;
		// 기저 사례를 확인한다
		pair<bool, double> isFinished = finishChecker->isFinished(state);
		if(isFinished.first) {
			minLength = min(minLength, isFinished.second);
			return;
		}

		// 다음 정점을 결정한다
		vector<int> order = orderSelector->getOrder(state, minLength);
		for(int i = 0; i < order.size(); ++i) {
			int next = order[i];
			if(state.visited[next]) continue;

			state.push(next);
			dfs(state);
			state.pop();
		}
	}

	virtual double solve(const TSPProblem& problem) {
		// 빈 상태를 생성한다
		TSPState state(problem);

		minLength = 1e200;

		// 모든 시작점을 하나씩 시도해 본다
		for(int start = 0; start < problem.n; ++start) {
			state.push(start);
			dfs(state);
			state.pop();
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

// 완성된 답이 주어질 때 이 답을 적절히 개선한다.
struct Optimizer {
	virtual void optimize(TSPState& state) = 0;
};

struct LocalSearchOptimizer: public Optimizer {
	virtual void optimize(TSPState& s) {
		while(true) {
			bool improved = false;
			for(int i = 0; i < s.path.size(); i++) {
				for(int j = i+1; j < s.path.size(); j++) {
					int A = s.path[i], B = s.path[j];
					double delta = 0;
					if(i > 0)
						delta = delta - s.problem.dist[s.path[i-1]][A] + s.problem.dist[s.path[i-1]][B];
					if(j + 1 < s.path.size())
						delta = delta - s.problem.dist[B][s.path[j+1]] + s.problem.dist[A][s.path[j+1]];
					if(i + 1 < j)
						delta = delta - s.problem.dist[A][s.path[i+1]] + s.problem.dist[B][s.path[i+1]]
							- s.problem.dist[s.path[j-1]][B] + s.problem.dist[s.path[j-1]][A];
					if(delta < 0) {
						s.length += delta;
						improved = true;
						swap(s.path[i], s.path[j]);
						break;
					}
				}
				if(improved) break;
			}
			if(improved) continue;
			for(int i = 0; i < s.path.size(); i++) {
				for(int j = i+2; j+1 < s.path.size(); j++) {
					//           A  +   B               C        D
					// (.. path[i]) + (path[j] .. path[i+1]) + (path[j+1] ..)
					int A = s.path[i], B = s.path[j], C = s.path[i+1], D = s.path[j+1];
					double delta = s.problem.dist[A][B] + s.problem.dist[C][D]
						- s.problem.dist[A][C] - s.problem.dist[B][D];
					if(delta < -1e-9) {
						s.length += delta;
						improved = true;
						reverse(s.path.begin() + i + 1, s.path.begin() + j + 1);
						break;
					}
				}
				if(improved) break;
			}
			if(!improved) break;
		}
	}
};

struct TwoOptimizer : public Optimizer {

	virtual void optimize(TSPState& s) {
		while(true) {
			bool improved = false;
			for(int i = 0; i < s.path.size(); i++) {
				for(int j = i+2; j+1 < s.path.size(); j++) {
					//           A  +   B               C        D
					// (.. path[i]) + (path[j] .. path[i+1]) + (path[j+1] ..)
					int A = s.path[i], B = s.path[j], C = s.path[i+1], D = s.path[j+1];
					double delta = s.problem.dist[A][B] + s.problem.dist[C][D]
						- s.problem.dist[A][C] - s.problem.dist[B][D];
					if(delta < -1e-9) {
						s.length += delta;
						improved = true;
						reverse(s.path.begin() + i + 1, s.path.begin() + j + 1);
						break;
					}
				}
				if(improved) break;
			}
			if(!improved) break;
		}
	}

};

// 임의의 두 정점의 순서를 바꿨을 때 경로가 더 짧아지면 바꾸는 것을 반복하는
// 국소 탐색을 구현한다.
struct SwapOptimizer: public Optimizer {
	virtual void optimize(TSPState& s) {
		while(true) {
			bool improved = false;
			for(int i = 0; i < s.path.size(); i++) {
				for(int j = i+1; j < s.path.size(); j++) {
					int A = s.path[i], B = s.path[j];
					double delta = 0;
					if(i > 0)
						delta = delta - s.problem.dist[s.path[i-1]][A] + s.problem.dist[s.path[i-1]][B];
					if(j + 1 < s.path.size())
						delta = delta - s.problem.dist[B][s.path[j+1]] + s.problem.dist[A][s.path[j+1]];
					if(i + 1 < j)
						delta = delta - s.problem.dist[A][s.path[i+1]] + s.problem.dist[B][s.path[i+1]]
							- s.problem.dist[s.path[j-1]][B] + s.problem.dist[s.path[j-1]][A];
					if(delta < 0) {
						s.length += delta;
						improved = true;
						swap(s.path[i], s.path[j]);
						break;
					}
				}
				if(improved) break;
			}
			if(!improved) break;
		}
	}

};

// 최적해가 갱신될 때마다 Optimizer 로 개선해 본다
struct OptimizingFinishChecker: public FinishChecker {
	vector<Optimizer*> optimizers;

	void addOptimizer(Optimizer* optimizer) {
		optimizers.push_back(optimizer);
	}

	virtual pair<bool,double> isFinished(const TSPState& state) {
		if(state.path.size() < state.problem.n) return make_pair(false, state.length);
		TSPState optimized = state;
		for(int i = 0; i < optimizers.size(); i++)
			optimizers[i]->optimize(optimized);
		return make_pair(true, optimized.length);
	}

};

// 방문할 정점이 cacheDepth 이하로 남은 문제들에 대해 메모이제이션을 수행한다
struct MemoizingFinishChecker : public FinishChecker {
	int bino[MAX_N][MAX_N];
	int stateLimit;

	// 마지막 cacheDepth 개의 정점에 대해 메모이제이션을 수행한다
	int cacheDepth;

	// solve() 의 반환값을 저장하는 캐시 배열
	vector<vector<double> > cache[MAX_N];

	// 생성자의 인자로 사용할 캐시 배열의 총 크기 제한을 지정한다.
	MemoizingFinishChecker(int stateLimit) : stateLimit(stateLimit) {
	}

	// 일대일 대응 함수에 사용할 이항계수를 미리 계산해 둔다
	void calcBino(const TSPProblem& problem) {
		memset(bino, 0, sizeof(bino));
		for(int i = 0; i < MAX_N; ++i) {
			bino[i][0] = 1;
			for(int j = 1; j < i+1; j++) {
				bino[i][j] = min(stateLimit/problem.n+1, bino[i-1][j-1] + bino[i-1][j]);
			}
		}
	}

	virtual void init(const TSPProblem& problem) {
		calcBino(problem);
		determineCacheDepth(problem);

		// 캐시 배열을 초기화한다
		for(int i = 0; i < problem.n; i++) {
			cache[i].clear();
			cache[i].resize(cacheDepth + 1);
			for(int j = 1; j <= cacheDepth; j++) {
				cache[i][j].resize(bino[problem.n][j], -1);
			}
		}
	}

	// cache 배열의 크기가 stateLimit 을 초과하지 않는 최대의 깊이를 계산한다.
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
	}

	// solve() 에서 쓰는 일대일 대응 함수.
	int calcIndex(const TSPState& state) {
		int ret = 0;
		int visitedCount = state.visited.count();
		for(int i = 0; i < state.problem.n; ++i)
			if(state.visited[i]) {
				ret += bino[state.problem.n - i - 1][visitedCount];
				visitedCount--;
			}
		return ret;
	}


	// state 에서 종료 상태까지 도달하기 위한 최단 경로를 반환한다.
	// 이 때 아직 방문하지 않은 정점의 수는 cacheDepth 이하여야 한다.
	double solve(TSPState& state) {
		// 기저 사례 확인
		if(state.path.size() == state.problem.n) return 0;
		// 일대일 대응 함수를 써서 toVisit 을 정수로 변환한다
		int idx = calcIndex(state);

		// 메모이제이션
		double& ret = cache[state.path.back()][state.problem.n - state.visited.count()][idx];
		if(ret >= 0) return ret;

		ret = 1e200;
		for(int next = 0; next < state.problem.n; ++next) {
			if(state.visited[next]) continue;
			double cand = state.problem.dist[state.path.back()][next];
			state.push(next);
			cand += solve(state);
			state.pop();
			ret = min(ret, cand);
		}
		return ret;
	}

	virtual pair<bool,double> isFinished(const TSPState& state) {
		if(state.problem.n == state.path.size()) return make_pair(true, state.length);

		// 남은 정점의 수가 캐시 깊이와 같다면 메모이제이션을 사용해 문제를 해결한다
		if(state.problem.n - state.path.size() == cacheDepth) {
			TSPState copy = state;
			return make_pair(true, state.length + solve(copy));
		}
		return make_pair(false, state.length);
	}
};

struct NearestNeighborOrderSelector : public OrderSelector {

	vector<vector<int> > ret;
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
	virtual vector<int> getOrder(const TSPState& state, double minLength) {
		return ret[state.path.back()];
	}
};

// 답의 하한을 예상하는 estimator 들을 이용한 가지치기를 구현한다.
struct EstimatingPruner : public Pruner {
	vector<Estimator*> estimators;

	void addEstimator(Estimator* estimator) {
		estimators.push_back(estimator);
	}

	virtual void init(const TSPProblem& problem) {
		for(int i = 0; i < estimators.size(); ++i)
			estimators[i]->init(problem);
	}

	// estimator 중 하나라도 현재 답 이상의 하한을 예상하면 걸러낸다.
	virtual bool prune(const TSPState& state, double minLength) {
		for(int i = 0; i < estimators.size(); ++i)
			if(estimators[i]->estimate(state) >= minLength)
				return true;
		return false;
	}

};

// 현재 정점 이전 두 개의 정점의 순서를 바꿔 보고 더 좋아지면 탐색을 중단한다
struct PathSwapPruner : public Pruner {

	virtual bool prune(const TSPState& state, double minLength) {
		if(state.path.size() < 4) return false;
		const int a = state.path[state.path.size()-4];
		const int b = state.path[state.path.size()-3];
		const int c = state.path[state.path.size()-2];
		const int d = state.path[state.path.size()-1];
		return state.problem.dist[a][b] + state.problem.dist[c][d] >
			state.problem.dist[a][c] + state.problem.dist[b][d];
	}
};

// 경로의 일부분을 뒤집어 보고 더 좋아지면 탐색을 중단한다
struct PathReversePruner: public Pruner {

	virtual bool prune(const TSPState& state, double minLength) {
		if(state.path.size() < 3) return false;
		int c = state.path[state.path.size()-2];
		int d = state.path.back();
		for(int i = 0; i+2 < state.path.size(); ++i) {
			int a = state.path[i], b = state.path[i+1];
			if(state.problem.dist[a][c] + state.problem.dist[b][d] <
					state.problem.dist[a][b] + state.problem.dist[c][d])
				return true;
		}
		return false;
	}
};

// 별다른 일을 하지 않는 Estimator 의 구현
struct NaiveEstimator : public Estimator {
	virtual double estimate(const TSPState& state) {
		return state.length;
	}
};

// 남아 있는 정점에 대해 인접한 최소 간선의 합을 반환한다.
struct IncomingEdgeEstimator : public Estimator {

	vector<double> minEdge;

	// 각 정점에 인접한 간선 중 가장 짧은 것을 미리 찾아 둔다.
	virtual void init(const TSPProblem& problem) {
		minEdge.resize(problem.n);
		for(int i = 0; i < problem.n; i++) {
			minEdge[i] = 1e200;
			for(int j = 0; j < problem.n; j++)
				if(i != j)
					minEdge[i] = min(minEdge[i], problem.dist[i][j]);
		}
	}

	virtual double estimate(const TSPState& state) {
		double lowerBound = state.length;
		for(int i = 0; i < state.problem.n; ++i)
			if(!state.visited[i])
				lowerBound += minEdge[i];
		return lowerBound;
	}
};

// Union-Find 상호 배제적 집합 자료 구조를 구현한다
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

// 현재 위치와 방문하지 않은 정점들간을 연결하는 간선 중 가장 짧은 x개의
// 합을 반환하는 휴리스틱
struct SmallestEdgesEstimator: public Estimator {

	vector<pair<double,pair<int,int> > > edges;

	// 각 간선의 목록을 가중치별로 정렬해 edges 에 저장해 둔다
	virtual void init(const TSPProblem& problem) {
		edges.clear();
		for(int i = 0; i < problem.n; ++i)
			for(int j = i+1; j < problem.n; j++)
				edges.push_back(make_pair(problem.dist[i][j], make_pair(i, j)));
		sort(edges.begin(), edges.end());
	}
	virtual double estimate(const TSPState& state) {
		double lowerBound = state.length;
		int need = state.problem.n - state.path.size();
		int here = state.path.empty() ? -1 : state.path.back();
		for(int i = 0; i < edges.size(); i++) {
			// 간선은 아직 방문하지 않은 점이나 현재 경로의 마지막 점을 연결해야 한다
			int a = edges[i].second.first, b = edges[i].second.second;
			if(a != here && state.visited[a]) continue;
			if(b != here && state.visited[b]) continue;
			lowerBound += edges[i].first;
			if(--need == 0) break;
		}
		return lowerBound;
	}
};

// 현재 위치와 방문하지 않은 정점들을 모두 잇는 MST 를 구하는 휴리스틱
struct MSTEstimator: public Estimator {

	vector<pair<double,pair<int,int> > > edges;

	// Kruskal 을 위해 각 간선의 목록을 가중치별로 정렬해 edges 에 저장해 둔다
	virtual void init(const TSPProblem& problem) {
		edges.clear();
		for(int i = 0; i < problem.n; ++i)
			for(int j = i+1; j < problem.n; j++)
				edges.push_back(make_pair(problem.dist[i][j], make_pair(i, j)));
		sort(edges.begin(), edges.end());
	}

	// Kruskal's MST
	virtual double estimate(const TSPState& state) {
		int here = state.path.empty() ? -1 : state.path.back();
		UnionFind* uf = new UnionFind(state.problem.n);
		double taken = 0;
		for(int i = 0; i < edges.size(); i++) {
			int a = edges[i].second.first, b = edges[i].second.second;
			if(a != here && state.visited[a]) continue;
			if(b != here && state.visited[b]) continue;
			a = uf->find(a); b = uf->find(b);
			if(a != b) {
				taken += edges[i].first;
				uf->join(a, b);
			}
		}
		delete uf;
		return taken + state.length;
	}
};

// Estimator 가 낮은 하한을 반환하는 선택지부터 시도한다
struct EstimatingOrderSelector : public OrderSelector {

	Estimator *estimator;

	EstimatingOrderSelector(Estimator* estimator) : estimator(estimator) {
	}

	virtual void init(const TSPProblem& problem) {
		estimator->init(problem);
	}

	virtual vector<int> getOrder(const TSPState& state, double minLength) {
		vector<pair<double,int> > ord;
		TSPState newState = state;
		for(int i = 0; i < state.problem.n; i++) {
			if(newState.visited[i]) continue;
			newState.push(i);
			ord.push_back(make_pair(estimator->estimate(newState), i));
			newState.pop();
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

map<string, DepthFirstSolver*> solvers;

void setupSolvers() {

	// SETUP SELECTORS
	vector<string> selectorNames;
	vector<OrderSelector*> selectors;

	selectorNames.push_back("Default");
	selectors.push_back(new OrderSelector());

	selectorNames.push_back("Nearest");
	selectors.push_back(new NearestNeighborOrderSelector());

	/*
	selectorNames.push_back("IncomingEdge");
	selectors.push_back(new EstimatingOrderSelector( new IncomingEdgeEstimator()));

	selectorNames.push_back("MST");
	selectors.push_back(new EstimatingOrderSelector( new MSTEstimator()));*/

	// SETUP FINISHCHECKERS
	vector<string> checkerNames;
	vector<FinishChecker*> checkers;

	checkerNames.push_back("Default");
	checkers.push_back(new FinishChecker());

	checkerNames.push_back("Memoization");
	checkers.push_back(new MemoizingFinishChecker(500000));
	/*
	{
		checkerNames.push_back("OptimizingLocalSearch");
		OptimizingFinishChecker* opt = new OptimizingFinishChecker();
		opt->addOptimizer(new LocalSearchOptimizer());
		//opt->addOptimizer(new TwoOptimizer());
		checkers.push_back(opt);
	}*/


	{
		checkerNames.push_back("OptimizingSwap");
		OptimizingFinishChecker* opt = new OptimizingFinishChecker();
		opt->addOptimizer(new SwapOptimizer());
		//opt->addOptimizer(new TwoOptimizer());
		checkers.push_back(opt);
	}

	/*
	{
		checkerNames.push_back("OptimizingTwo");
		OptimizingFinishChecker* opt = new OptimizingFinishChecker();
		opt->addOptimizer(new TwoOptimizer());
		checkers.push_back(opt);
	}

	{
		checkerNames.push_back("OptimizingSwapTwo");
		OptimizingFinishChecker* opt = new OptimizingFinishChecker();
		opt->addOptimizer(new SwapOptimizer());
		opt->addOptimizer(new TwoOptimizer());
		checkers.push_back(opt);
	}*/

	// SETUP PRUNERS
	vector<string> prunerNames;
	vector<Pruner*> pruners;

	//prunerNames.push_back("PathSwap");
	//pruners.push_back(new PathSwapPruner());

	prunerNames.push_back("PathReverse");
	pruners.push_back(new PathReversePruner());

	// SETUP ESTIMATORS
	vector<string> estimatorNames;
	vector<Estimator*> estimators;

	estimatorNames.push_back("Naive");
	estimators.push_back(new NaiveEstimator());

	estimatorNames.push_back("IncomingEdge");
	estimators.push_back(new IncomingEdgeEstimator());

	//estimatorNames.push_back("SmallestEdges");
	//estimators.push_back(new SmallestEdgesEstimator());

	estimatorNames.push_back("MST");
	estimators.push_back(new MSTEstimator());

	int s = selectors.size();
	int c = checkers.size();
	int p = pruners.size();
	int e = estimators.size();
	for(int selector = 0; selector < s; ++selector) {
		string base = ":" + selectorNames[selector] + ":";
		for(int checker = 0; checker < c; ++checker) {
			string base2 = base + checkerNames[checker] + ":";
			for(int prunerSet = 0; prunerSet < (1<<p); ++prunerSet) {
				for(int estimatorSet = 0; estimatorSet < (1<<e); ++estimatorSet) {
					{
						/* Setup DepthFirstSolver */
						string name = base2;
						DepthFirstSolver* solver = new DepthFirstSolver();
						solver->setOrderSelector(selectors[selector]);
						solver->setFinishChecker(checkers[checker]);
						if(prunerSet) {
							for(int i = 0; i < p; ++i)
								if(prunerSet & (1<<i)) {
									solver->addPruner(pruners[i]);
									if(name[name.size()-1] != ':')
										name += ',';
									name += prunerNames[i];
								}
						}
						name += ":";
						if(estimatorSet) {
							EstimatingPruner* pruner = new EstimatingPruner();
							for(int i = 0; i < e; i++) if(estimatorSet & (1<<i)) {
								if(name[name.size()-1] != ':')
									name += ',';
								name += estimatorNames[i];
								pruner->addEstimator(estimators[i]);
							}
							solver->addPruner(pruner);
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
}

double solve(const string& algorithm, const TSPProblem& problem) {
	if(solvers.count(algorithm) == 0)
		return -1;
	DepthFirstSolver* s = solvers[algorithm];
	s->init(problem);
	return s->solve(problem);
}

int main(int argc, char* argv[]) {
	setupSolvers();

#ifndef ONLINE_JUDGE
	if(argc != 4) {
		printf("Usage) %s [algorithm] [input] [output]\n\n", argv[0]);
		printf("algorithm = ");
		bool first = true;
		for(map<string,DepthFirstSolver*>::iterator it = solvers.begin(); it != solvers.end(); ++it) {
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
#else
	int cases;
	cin >> cases;
	for(int i = 0; i < cases; ++i)
		printf("%.10lf\n", solve("DFS:Nearest:Memoization:PathReverse:IncomingEdge,MST", readAOJ(cin)));


#endif
}

