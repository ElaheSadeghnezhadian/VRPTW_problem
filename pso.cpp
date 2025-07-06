#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <random>
#include <unordered_map>
#include <functional>   
#include <utility>
#include <deque>
using namespace std;

int evaluationCounter = 0;
struct Customer {
    int id;
    double x, y;
    int demand;
    int readyTime;
    int dueTime;
    int serviceTime;
};

using Solution = vector<vector<int>>;

vector<Customer> customers;
vector<vector<double>> dist;
int vehicleCount, vehicleCapacity, numCustomers;
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

// ------------------ pso parameters ----------------------------
struct Particle {
    Solution position;
    Solution bestPosition;
    double bestFitness;
};

const int SWARM_SIZE = 50;
const long long MAX_ITER = 10000000000000;

double W_start = 0.9;
double W_end   = 0.4;

double C1_start = 2.5;
double C1_end   = 1.5;

double C2_start = 1.5;
double C2_end   = 2.5;

// ===== Euclidean =====
inline double euclidean(const Customer &a, const Customer &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// ==== distance matrix ====
void buildDistanceMatrix() {
    int n = customers.size();
    dist.assign(n, vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist[i][j] = euclidean(customers[i], customers[j]);
}

// ==== Read instance ====
void readInstance(const string &filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }

    string line;
    getline(infile, line);

    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            infile >> vehicleCount >> vehicleCapacity;
            break;
        }
    }

    while (getline(infile, line)) {
        if (line.find("CUSTOMER") != string::npos) {
            getline(infile, line);
            break;
        }
    }

    int id;
    double x, y;
    int demand, ready_time, due_date, service_time;
    while (infile >> id >> x >> y >> demand >> ready_time >> due_date >> service_time) {
        customers.push_back({id, x, y, demand, ready_time, due_date, service_time});
    }

    infile.close();
    numCustomers = customers.size();
    buildDistanceMatrix();
}

// ===== validation ====
bool validRoute(const vector<int>& route) {
    int load = 0;
    double time = 0;
    if (route.front()!=0 || route.back()!=0) return false;
    for (int i = 1; i < (int)route.size(); ++i) {
        int u = route[i-1], v = route[i];
        time += dist[u][v];
        time = max(time, (double)customers[v].readyTime);
        if (time > customers[v].dueTime) return false;
        if (v!=0) {
            time += customers[v].serviceTime;
            load += customers[v].demand;
            if (load > vehicleCapacity) return false;
        }
    }
    return true;
}

bool isFeasible(const Solution &sol) {
    if (sol.size()>vehicleCount) return false;
    vector<bool> seen(numCustomers,false);
    seen[0]=true;
    for (auto &r:sol) {
        if (!validRoute(r)) return false;
        for (int i=1;i+1<r.size();i++) {
            if (seen[r[i]]) return false;
            seen[r[i]] = true;
        }
    }
    for (int i=0;i<numCustomers;i++) if (!seen[i]) return false;
    return true;
}

// ===== objective ====
double routeCost(const vector<int>& route) {
    double c = 0.0;
    for (int i = 1; i < route.size(); ++i) {
        c += dist[route[i-1]][route[i]];
    }
    return c;
}

double totalCost(const vector<vector<int>>& sol) {
    double cost = 0.0;
    for (const auto& r : sol)
        cost += routeCost(r);
    return cost;
}

double penaltyTerm(const Solution& sol) {
    double penalty = 0.0;

    vector<bool> visited(numCustomers, false);
    visited[0] = true; // Depot

    for (const auto& route : sol) {
        int load = 0;
        double time = 0;

        if (route.empty()) continue;

        for (int i = 1; i < route.size(); ++i) {
            int u = route[i - 1], v = route[i];
            time += dist[u][v];
            time = max(time, (double)customers[v].readyTime);

            if (time > customers[v].dueTime) {
                penalty += (time - customers[v].dueTime); // Time window violation
            }

            if (v != 0) {
                load += customers[v].demand;
                time += customers[v].serviceTime;

                if (load > vehicleCapacity) {
                    penalty += (load - vehicleCapacity) * 1000; // Capacity violation
                }

                if (visited[v]) {
                    penalty += 1000; // Visit duplication
                } else {
                    visited[v] = true;
                }
            }
        }
    }

    for (int i = 1; i < numCustomers; ++i)
        if (!visited[i]) penalty += 1000; // Customer not visited

    return penalty;
}

double objective(const Solution &sol) {
    evaluationCounter++;
    int used = 0;
    double distance = 0;
    for (const auto& r : sol) {
        if (r.size() > 2) {
            used++;
            distance += routeCost(r);
        }
    }

    double penalty = penaltyTerm(sol);
    return used * 10000 + distance + penalty * 100; 
}

// ===== Random solution generation =====
// good but not valid
Solution randomSolution1() {
    // Sort customers: larger demand first
    vector<int> customers_to_assign(numCustomers - 1);
    iota(customers_to_assign.begin(), customers_to_assign.end(), 1);
    sort(customers_to_assign.begin(), customers_to_assign.end(),
        [](int a, int b) {
            return customers[a].demand > customers[b].demand;
        });

    Solution sol;
    sol.emplace_back(vector<int>{0, 0});  // first route

    vector<int> routeLoads(1, 0);  // load per route

    for (int cust : customers_to_assign) {
        double bestDelta = numeric_limits<double>::max();
        int bestRouteIdx = -1, bestPos = -1;

        // Try inserting into existing routes
        for (int r = 0; r < (int)sol.size(); ++r) {
            auto& route = sol[r];
            if (routeLoads[r] + customers[cust].demand > vehicleCapacity)
                continue;

            for (int pos = 1; pos < (int)route.size(); ++pos) {
                int prev = route[pos - 1], next = route[pos];
                double delta =
                    dist[prev][cust] +
                    dist[cust][next] -
                    dist[prev][next];

                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestRouteIdx = r;
                    bestPos = pos;
                }
            }
        }

        // Compare with opening a new route
        if ((int)sol.size() < vehicleCount) {
            double newRouteCost = 2 * dist[0][cust];
            if (newRouteCost < bestDelta) {
                sol.emplace_back(vector<int>{0, cust, 0});
                routeLoads.push_back(customers[cust].demand);
                continue;
            }
        }

        if (bestRouteIdx != -1) {
            // Insert into best existing route
            sol[bestRouteIdx].insert(sol[bestRouteIdx].begin() + bestPos, cust);
            routeLoads[bestRouteIdx] += customers[cust].demand;
        } else {
            // Fallback: force into first route
            sol[0].insert(sol[0].begin() + 1, cust);
            routeLoads[0] += customers[cust].demand;
        }
    }

    return sol;
}

// valid
Solution randomSolution() {
    // مشتریان غیر-دپو
    vector<int> customers_to_assign(numCustomers - 1);
    iota(customers_to_assign.begin(), customers_to_assign.end(), 1);
    shuffle(customers_to_assign.begin(), customers_to_assign.end(), rng);

    Solution sol;
    sol.emplace_back(vector<int>{0, 0});  // مسیر اول خالی

    auto currentLoad = [](const vector<int>& route) {
        int load = 0;
        for (int i = 1; i + 1 < route.size(); ++i)
            load += customers[route[i]].demand;
        return load;
    };

    for (int cust : customers_to_assign) {
        double bestCost = numeric_limits<double>::max();
        int bestRoute = -1, bestPos = -1;

        // 🔷 سعی می‌کنیم در یکی از مسیرهای موجود درج کنیم
        for (int r = 0; r < (int)sol.size(); ++r) {
            auto& route = sol[r];
            int load = currentLoad(route);

            if (load + customers[cust].demand > vehicleCapacity)
                continue;

            for (int pos = 1; pos < (int)route.size(); ++pos) {
                route.insert(route.begin() + pos, cust);
                if (validRoute(route)) {
                    double cost = routeCost(route);
                    if (cost < bestCost) {
                        bestCost = cost;
                        bestRoute = r;
                        bestPos = pos;
                        if (cost == 0.0) {
                            route.erase(route.begin() + pos);
                            goto assign;  // سریع‌ترین حالت
                        }
                    }
                }
                route.erase(route.begin() + pos);
            }
        }

    assign:
        if (bestRoute != -1) {
            // ✅ پیدا شد
            sol[bestRoute].insert(sol[bestRoute].begin() + bestPos, cust);
        } else if ((int)sol.size() < vehicleCount) {
            // ✅ مسیر جدید (اگر ظرفیت داریم)
            sol.emplace_back(vector<int>{0, cust, 0});
        } else {
            // 🔷 fallback: سعی کن در مسیری جا بدهی
            bool inserted = false;
            for (auto& route : sol) {
                int load = currentLoad(route);
                if (load + customers[cust].demand <= vehicleCapacity) {
                    route.insert(route.end() - 1, cust);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                // ❌ در هیچ مسیری جا نشد → اضافه به مسیر اول (حتی اگر infeasible)
                sol[0].insert(sol[0].begin() + 1, cust);
            }
        }
    }

    return sol;
}
// ===== Move towards best solution =====
Solution moveTowards(const Solution& current, const Solution& pbest, const Solution& gbest,
                     double w, double c1, double c2) {
    Solution next = current;
    uniform_real_distribution<> dist01(0.0, 1.0);

    auto flatten = [](const Solution& sol) {
        vector<int> seq;
        for (const auto& r : sol) {
            for (int c : r) if (c != 0) seq.push_back(c);
        }
        return seq;
    };

    auto rebuildSolutionFeasible = [](const vector<int>& seq) {
        Solution rebuilt;
        int idx = 0;
        while (idx < seq.size()) {
            vector<int> route = {0};
            int load = 0;
            while (idx < seq.size()) {
                int cust = seq[idx];
                if (load + customers[cust].demand <= vehicleCapacity) {
                    route.push_back(cust);
                    load += customers[cust].demand;
                    idx++;
                } else break;
            }
            route.push_back(0);
            rebuilt.push_back(route);
        }
        return rebuilt;
    };

    auto solutionCostWithPenalty = [](const Solution& sol) {
        double total = 0.0;
        for (const auto& r : sol) {
            int load = 0;
            for (size_t i = 0; i + 1 < r.size(); ++i) {
                total += dist[r[i]][r[i+1]];
                if (r[i] != 0) load += customers[r[i]].demand;
            }
            if (load > vehicleCapacity) total += 1e6 * (load - vehicleCapacity);  // heavy penalty
        }
        return total;
    };

    auto moveTowardTarget = [&](const Solution& target) {
        vector<int> currSeq = flatten(next);
        vector<int> targetSeq = flatten(target);

        vector<size_t> diffPositions;
        size_t len = min(currSeq.size(), targetSeq.size());
        for (size_t i = 0; i < len; ++i) {
            if (currSeq[i] != targetSeq[i]) {
                diffPositions.push_back(i);
            }
        }
        if (diffPositions.empty()) return;

        int maxSwaps = max(1, (int)diffPositions.size() / 4);  // more aggressive
        uniform_int_distribution<int> swapCountDist(1, maxSwaps);
        int swapCount = swapCountDist(rng);

        for (int s = 0; s < swapCount && !diffPositions.empty(); ++s) {
            uniform_int_distribution<size_t> diffDist(0, diffPositions.size() - 1);
            size_t idx = diffDist(rng);
            size_t pos = diffPositions[idx];

            auto it = find(currSeq.begin() + pos + 1, currSeq.end(), targetSeq[pos]);
            if (it != currSeq.end()) {
                iter_swap(currSeq.begin() + pos, it);
            }

            diffPositions.erase(diffPositions.begin() + idx);
        }

        Solution candidate = rebuildSolutionFeasible(currSeq);
        if (solutionCostWithPenalty(candidate) < solutionCostWithPenalty(next)) {
            next = std::move(candidate);
        }
    };

    double r = dist01(rng);
    if (r < c1) {
        moveTowardTarget(pbest);
    } else if (r < c1 + c2) {
        moveTowardTarget(gbest);
    } else {
        // Smart random move: relocate a customer from overloaded route to underloaded route
        Solution candidate = next;

        // Compute loads
        vector<int> loads(candidate.size(), 0);
        for (size_t i = 0; i < candidate.size(); ++i) {
            for (int cust : candidate[i]) {
                if (cust != 0) loads[i] += customers[cust].demand;
            }
        }

        // Find overloaded and underloaded routes
        vector<int> over, under;
        for (size_t i = 0; i < candidate.size(); ++i) {
            if (loads[i] > vehicleCapacity) over.push_back(i);
            else if (loads[i] < vehicleCapacity * 0.9) under.push_back(i);
        }

        if (!over.empty() && !under.empty()) {
            uniform_int_distribution<int> overDist(0, over.size() - 1);
            uniform_int_distribution<int> underDist(0, under.size() - 1);
            int fromRoute = over[overDist(rng)];
            int toRoute = under[underDist(rng)];

            auto& from = candidate[fromRoute];
            auto& to = candidate[toRoute];

            if (from.size() > 2) {
                uniform_int_distribution<int> posDist(1, from.size() - 2);
                int pos = posDist(rng);
                int cust = from[pos];

                from.erase(from.begin() + pos);

                // Insert in best position in target route
                int bestPos = 1;
                double bestDelta = 1e9;
                for (size_t i = 1; i < to.size(); ++i) {
                    double delta = dist[to[i-1]][cust] + dist[cust][to[i]] - dist[to[i-1]][to[i]];
                    if (loads[toRoute] + customers[cust].demand <= vehicleCapacity &&
                        delta < bestDelta) {
                        bestDelta = delta;
                        bestPos = i;
                    }
                }
                to.insert(to.begin() + bestPos, cust);
            }
        }

        if (solutionCostWithPenalty(candidate) < solutionCostWithPenalty(next)) {
            next = std::move(candidate);
        }
    }

    return next;
}

// ===== Particle Swarm Optimization (PSO) =====
Solution particleSwarmOptimization(int maxTime, int maxEvaluations) {
    auto t_start = chrono::steady_clock::now();

    vector<Particle> swarm(SWARM_SIZE);
    Solution gbestPosition;
    Solution bestFeasible;
    double bestFeasibleFitness = numeric_limits<double>::max();

    double gbestFitness = numeric_limits<double>::max();

    for (auto &p : swarm) {
        p.position = randomSolution();
        p.bestPosition = p.position;
        p.bestFitness = objective(p.position);

        if (p.bestFitness < gbestFitness) {
            gbestFitness = p.bestFitness;
            gbestPosition = p.bestPosition;
        }
    }

    int iteration = 0;

    while (true) {
        auto t_now = chrono::steady_clock::now();
        int elapsed = chrono::duration_cast<chrono::seconds>(t_now - t_start).count();

        bool timeLimitReached = (maxTime > 0) && (elapsed >= maxTime);
        bool evalLimitReached = (maxEvaluations > 0) && (evaluationCounter >= maxEvaluations);

        if (timeLimitReached || evalLimitReached) break;

        double w = W_start - ((W_start - W_end) * iteration / MAX_ITER);
        double c1 = C1_start - ((C1_start - C1_end) * iteration / MAX_ITER);
        double c2 = C2_start + ((C2_end - C2_start) * iteration / MAX_ITER);


        for (auto &p : swarm) {
            Solution candidate = moveTowards(p.position, p.bestPosition, gbestPosition, w, c1, c2);

            double fit = objective(candidate);
            if (fit < p.bestFitness) {
                p.bestPosition = candidate;
                p.bestFitness = fit;
            }

            if (fit < gbestFitness) {
                gbestFitness = fit;
                gbestPosition = candidate;
            }

            // اضافه: اگر candidate معتبر است و بهتر از bestFeasible است
            if (isFeasible(candidate) && fit < bestFeasibleFitness) {
                bestFeasible = candidate;
                bestFeasibleFitness = fit;
            }


            p.position = candidate;
        }
        const double DIVERSITY_THRESHOLD = 1e-3;

        iteration++;
    }

    cout << "Best fitness found: " << gbestFitness << "\n";
    if (!bestFeasible.empty()) {
    cout << "Best feasible fitness found: " << bestFeasibleFitness << "\n";
    return bestFeasible;} 
    else {
        cout << "No feasible solution found, returning best found (possibly infeasible).\n";
        return gbestPosition;
    }

}

// ===== Output solution =====
void outputSolution(const vector<vector<int>>& sol, const string& inputFilename) {
    double cost = totalCost(sol);

    cout << "Vehicles used: " << sol.size() << "\n";
    cout << "Total cost: " << fixed << setprecision(2) << cost << "\n";

    for (size_t i = 0; i < sol.size(); ++i) {
        cout << "Route " << i + 1 << ": ";
        for (size_t j = 1; j < sol[i].size() - 1; ++j)
            cout << sol[i][j] << " ";
        cout << "| Cost: " << fixed << setprecision(2) << routeCost(sol[i]) << "\n";
    }

    // تولید نام فایل خروجی
    string outputFile;
    size_t lastSlash = inputFilename.find_last_of("/\\");
    string base = (lastSlash == string::npos) ? inputFilename : inputFilename.substr(lastSlash + 1);
    size_t dot = base.find_last_of('.');
    if (dot != string::npos) {
        base = base.substr(0, dot);
    }
    outputFile = base + "_output.txt";

    ofstream fout(outputFile);
    if (!fout) {
        cerr << "Error: cannot write to output file " << outputFile << endl;
        return;
    }

    // نوشتن مسیرها در فایل
    for (size_t i = 0; i < sol.size(); ++i) {
        fout << "Route " << i + 1 << ": ";
        for (size_t j = 1; j < sol[i].size() - 1; ++j) {
            fout << sol[i][j];
            if (j < sol[i].size() - 2) fout << " ";
        }
        fout << "\n";
    }

    // نوشتن تعداد خودرو و هزینه کل
    fout << "Vehicles: " << sol.size() << "\n";
    fout << "Distance: " << fixed << setprecision(2) << cost << "\n";

    fout.close();

    cout << "Solution written to " << outputFile << "\n";
}

int main(int argc, char* argv[]) {

    if (argc != 4) {
        cerr << "Usage: " << argv[0] 
             << " [instance-file] [max-time-sec] [max-evaluations]\n";
        return 1;
    }


    string file          = argv[1];
    int max_time = atoi(argv[2]);    
    int max_evaluations = atoi(argv[3]);

    readInstance(file);

    auto t0 = chrono::steady_clock::now();

    auto best = particleSwarmOptimization(max_time, max_evaluations);

    auto t1 = chrono::steady_clock::now();

    outputSolution(best, file);

    cout << "\n========== Execution Summary ==========";
    cout << "\n Total Runtime: " 
         << chrono::duration_cast<chrono::seconds>(t1-t0).count() 
         << " seconds\n";
    cout << " Max Time Allowed: " << max_time << " seconds\n";
    cout << " Max Evaluations Allowed: " << max_evaluations << "\n";
    cout << " Evaluations: " << evaluationCounter << "\n";

    if (isFeasible(best)) {
        cout << "Solution is valid and feasible.\n";
    } else {
        cout << "Solution is NOT valid!\n";
    }


    return 0;
}
