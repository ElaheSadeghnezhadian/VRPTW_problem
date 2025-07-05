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
    // velocity: برای مسائل گسسته می‌توانیم به جای سرعت واقعی از یک «لیست تغییرات پیشنهادی» استفاده کنیم
};

const int SWARM_SIZE = 30;
const int MAX_ITER = 500;

const double W = 0.7;       // اینرسی
const double C1 = 1.4;      // cognitive
const double C2 = 1.4;      // social

int elitist_weight = 5;
double q0_dynamic = 0.1;

vector<vector<double>> IM;
const double ALPHA_IM = 0.1;  // ضریب زمان در IM
unordered_map<int, double> CM;  // customer id → cost of insertion


// === logging streams ===


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

// ==== matrixes ====
/*void buildInformationMatrix() {
    int n = customers.size();
    IM.assign(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            IM[i][j] = dist[i][j] + ALPHA_IM * abs(customers[i].readyTime - customers[j].readyTime);
        }
    }
}*/

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
    // buildInformationMatrix();

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
Solution randomSolution() {
    vector<int> customers_to_assign(numCustomers - 1);
    iota(customers_to_assign.begin(), customers_to_assign.end(), 1);
    shuffle(customers_to_assign.begin(), customers_to_assign.end(), rng);

    Solution sol;
    sol.push_back({0, 0});  // شروع فقط با یک مسیر (depot -> depot)

    for (int cust : customers_to_assign) {
        double bestDelta = numeric_limits<double>::max();
        int bestRoute = -1, bestPos = -1;

        // 🔷 سعی می‌کنیم در یکی از مسیرهای فعلی جا بدهیم
        for (int r = 0; r < (int)sol.size(); ++r) {
            auto& route = sol[r];

            // محاسبه بار فعلی
            int load = 0;
            for (size_t i = 1; i + 1 < route.size(); ++i)
                load += customers[route[i]].demand;

            // ظرفیت اجازه نمی‌دهد → رد
            if (load + customers[cust].demand > vehicleCapacity)
                continue;

            // بهترین جایگاه برای درج در همین مسیر
            for (int pos = 1; pos < (int)route.size(); ++pos) {
                route.insert(route.begin() + pos, cust);

                if (validRoute(route)) {
                    double delta = routeCost(route);
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestRoute = r;
                        bestPos = pos;

                        // ✳️ early exit (اختیاری: چون greedy هستیم و مسیر جدید هزینه دارد)
                        if (delta == 0.0) break;
                    }
                }

                route.erase(route.begin() + pos);
            }
        }

        if (bestRoute != -1) {
            // در بهترین مسیر موجود درج می‌کنیم
            sol[bestRoute].insert(sol[bestRoute].begin() + bestPos, cust);
        } else if ((int)sol.size() < vehicleCount) {
            // 🔷 اگر امکان ساخت مسیر جدید هست، می‌سازیم
            sol.push_back({0, cust, 0});
        } else {
            // 🔷 fallback: مجبوریم به مسیر اول اضافه کنیم
            sol[0].insert(sol[0].begin() + 1, cust);
        }
    }

    return sol;
}

// ===== Move towards best solution =====
Solution moveTowards1(const Solution& current, const Solution& pbest, const Solution& gbest) {
    Solution next = current;

    uniform_real_distribution<> dist01(0.0, 1.0);

    if (dist01(rng) < C1) {
        // حرکت به سمت pbest
        int r = rand() % next.size();
        vector<int> &route = next[r];

        vector<int> targetRoute;
        for (const auto& r : pbest) {
            for (int c : r) {
                if (c != 0) targetRoute.push_back(c);
            }
        }

        vector<int> currSeq;
        for (const auto& r : next) {
            for (int c : r) {
                if (c != 0) currSeq.push_back(c);
            }
        }

        // تلاش برای نزدیک کردن ترتیب به pbest
        for (size_t i = 0; i + 1 < currSeq.size(); ++i) {
            if (currSeq[i] != targetRoute[i]) {
                auto it = find(currSeq.begin() + i + 1, currSeq.end(), targetRoute[i]);
                if (it != currSeq.end()) {
                    iter_swap(currSeq.begin() + i, it);
                    break;
                }
            }
        }

        // بازسازی next از currSeq
        int idx = 0;
        next.clear();
        while (idx < currSeq.size()) {
            vector<int> route = {0};
            int load = 0;
            double time = 0;

            while (idx < currSeq.size()) {
                int cust = currSeq[idx];
                if (load + customers[cust].demand <= vehicleCapacity) {
                    route.push_back(cust);
                    load += customers[cust].demand;
                    idx++;
                } else {
                    break;
                }
            }
            route.push_back(0);
            next.push_back(route);
        }

    } else if (dist01(rng) < C2) {
        // حرکت به سمت gbest
        int r = rand() % next.size();
        vector<int> &route = next[r];

        vector<int> targetRoute;
        for (const auto& r : gbest) {
            for (int c : r) {
                if (c != 0) targetRoute.push_back(c);
            }
        }

        vector<int> currSeq;
        for (const auto& r : next) {
            for (int c : r) {
                if (c != 0) currSeq.push_back(c);
            }
        }

        for (size_t i = 0; i + 1 < currSeq.size(); ++i) {
            if (currSeq[i] != targetRoute[i]) {
                auto it = find(currSeq.begin() + i + 1, currSeq.end(), targetRoute[i]);
                if (it != currSeq.end()) {
                    iter_swap(currSeq.begin() + i, it);
                    break;
                }
            }
        }

        int idx = 0;
        next.clear();
        while (idx < currSeq.size()) {
            vector<int> route = {0};
            int load = 0;
            double time = 0;

            while (idx < currSeq.size()) {
                int cust = currSeq[idx];
                if (load + customers[cust].demand <= vehicleCapacity) {
                    route.push_back(cust);
                    load += customers[cust].demand;
                    idx++;
                } else {
                    break;
                }
            }
            route.push_back(0);
            next.push_back(route);
        }

    } else {
        // حرکت تصادفی (اکتشاف)
        int r1 = rand() % next.size();
        int r2 = rand() % next[r1].size();
        int r3 = rand() % next.size();
        int r4 = rand() % next[r3].size();
        if (r1 != r3 || r2 != r4) {
            swap(next[r1][r2], next[r3][r4]);
        }
    }

    return next;
}

Solution moveTowards(const Solution& current, const Solution& pbest, const Solution& gbest) {
    Solution next = current;

    uniform_real_distribution<> dist01(0.0, 1.0);
    double rand_val = dist01(rng);

    auto flattenSolution = [](const Solution& sol) {
        vector<int> seq;
        for (const auto& route : sol) {
            for (int c : route) {
                if (c != 0) seq.push_back(c);
            }
        }
        return seq;
    };

    auto rebuildSolution = [](const vector<int>& seq) {
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
                } else {
                    break;
                }
            }
            route.push_back(0);
            rebuilt.push_back(route);
        }
        return rebuilt;
    };

    auto moveTowardsTarget = [&](const Solution& target) {
        vector<int> targetSeq = flattenSolution(target);
        vector<int> currSeq = flattenSolution(next);

        for (size_t i = 0; i + 1 < currSeq.size(); ++i) {
            if (currSeq[i] != targetSeq[i]) {
                auto it = find(currSeq.begin() + i + 1, currSeq.end(), targetSeq[i]);
                if (it != currSeq.end()) {
                    iter_swap(currSeq.begin() + i, it);
                    break;
                }
            }
        }
        next = rebuildSolution(currSeq);
    };

    if (rand_val < C1) {
        // حرکت به سمت pbest
        moveTowardsTarget(pbest);

    } else if (rand_val < C1 + C2) {
        // حرکت به سمت gbest
        moveTowardsTarget(gbest);

    } else {
        // حرکت تصادفی (اکتشاف)
        int r1 = rand() % next.size();
        int r2 = 1 + rand() % (next[r1].size() - 2); // از 1 تا size-2 چون 0 ها سر و ته هستند

        int r3 = rand() % next.size();
        int r4 = 1 + rand() % (next[r3].size() - 2);

        if (r1 != r3 || r2 != r4) {
            swap(next[r1][r2], next[r3][r4]);
        }
    }

    return next;
}

// ===== Longest Common Subsequence (LCS) =====
/*vector<int> LCS(const vector<int>& seq1, const vector<int>& seq2) {
    int m = seq1.size(), n = seq2.size();
    vector<vector<int>> dp(m+1, vector<int>(n+1,0));
    for (int i=1;i<=m;++i)
        for (int j=1;j<=n;++j)
            if (seq1[i-1]==seq2[j-1]) dp[i][j]=dp[i-1][j-1]+1;
            else dp[i][j]=max(dp[i-1][j],dp[i][j-1]);

    vector<int> lcs;
    int i=m, j=n;
    while(i>0 && j>0) {
        if(seq1[i-1]==seq2[j-1]) {
            lcs.push_back(seq1[i-1]);
            i--; j--;
        } else if(dp[i-1][j]>dp[i][j-1]) i--;
        else j--;
    }
    reverse(lcs.begin(), lcs.end());
    return lcs;
}*/

// ===== Build cost matrix for insertion =====
/*void buildCostMatrix(const Solution& sol) {
    CM.clear();
    for (int cust = 1; cust < numCustomers; ++cust) {
        double bestCost = numeric_limits<double>::max();
        for (const auto& route : sol) {
            for (size_t pos = 1; pos < route.size(); ++pos) {
                vector<int> newRoute = route;
                newRoute.insert(newRoute.begin() + pos, cust);
                if (validRoute(newRoute)) {
                    double c = routeCost(newRoute) - routeCost(route);
                    if (c < bestCost) bestCost = c;
                }
            }
        }
        CM[cust] = bestCost;
    }
}*/

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

        for (auto &p : swarm) {
            Solution candidate = moveTowards(p.position, p.bestPosition, gbestPosition);

            double fit = objective(candidate);
            if (fit < p.bestFitness) {
                p.bestPosition = candidate;
                p.bestFitness = fit;
            }

            if (fit < gbestFitness) {
                gbestFitness = fit;
                gbestPosition = candidate;
                // buildCostMatrix(gbestPosition);
            }

            // اضافه: اگر candidate معتبر است و بهتر از bestFeasible است
            if (isFeasible(candidate) && fit < bestFeasibleFitness) {
                bestFeasible = candidate;
                bestFeasibleFitness = fit;
            }


            p.position = candidate;
        }
        if (iteration % 50 == 0) {
            sort(swarm.begin(), swarm.end(), [](const Particle& a, const Particle& b) {
                return a.bestFitness < b.bestFitness;
            });
            // vector<int> lcsSeq = LCS(swarm[0].bestPosition[0], swarm[1].bestPosition[0]);
            // cout << "Iteration " << iteration << ": LCS length = " << lcsSeq.size() << "\n";
        }
        // if (iteration % 100 == 0) {
        //     cout << "Diversifying population...\n";
        //     for (int i = SWARM_SIZE/2; i < SWARM_SIZE; ++i) {
        //         swarm[i].position = randomSolution();
        //         swarm[i].bestPosition = swarm[i].position;
        //         swarm[i].bestFitness = objective(swarm[i].position);
        //     }
        // }

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
