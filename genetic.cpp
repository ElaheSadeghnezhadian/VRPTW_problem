#define _USE_MATH_DEFINES
#include <bits/stdc++.h>
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

// Genetic parameters
// double MUTATION_RATE = 0.4;
const int TOURNAMENT_K = 3;
const double MUTATION_INNER_PROB = 0.1;
const double MUTATION_INTER_PROB = 0.1;
const double CROSSOVER_RATE = 0.8;
const int ELITISM_COUNT = 1;
const int POP_SIZE = 50;
bool useRoulette = true;

// Logging streams
ofstream logEvents("logEvents.txt");
ofstream logPopulation("logPopulation.txt");
ofstream logBest("logBest.txt");
// ofstream logErrors("error_log.txt");

// Compute Euclidean distance
inline double euclidean(const Customer &a, const Customer &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// Build distance matrix
void buildDistanceMatrix() {
    int n = customers.size();
    dist.assign(n, vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist[i][j] = euclidean(customers[i], customers[j]);
}

// Read VRPTW instance
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
    evaluationCounter++;  // فرض بر اینکه این متغیر از قبل تعریف شده است
    int used = 0; 
    double totalDistance = 0.0;
    
    // محاسبه تعداد وسایل نقلیه استفاده‌شده و هزینه کل سفر
    for (const auto& r : sol) {
        if (r.size() > 2) {  // اگر مسیر دارای حداقل یک مشتری باشد
            used++;
            totalDistance += routeCost(r);  // جمع مسافت‌های مسیر
        }
    }
    
    // گرد کردن مسافت به دو رقم اعشار
    totalDistance = round(totalDistance * 100.0) / 100.0;
    
    // هدف: اولویت با تعداد وسایل نقلیه، سپس مسافت
    return used * 10000 + totalDistance;
}

// double objective(const Solution &sol) {
//     evaluationCounter++;
//     int used = 0;
//     double distance = 0;
//     for (const auto& r : sol) {
//         if (r.size() > 2) {
//             used++;
//             distance += routeCost(r);
//         }
//     }

//     double penalty = penaltyTerm(sol);
//     return used * 10000 + distance + penalty * 100; // وزن penalty قابل تنظیم است
// }

// ===== log =====
void logSolution(const Solution &sol, int solId) {
    logPopulation << "Solution " << solId << ": Objective=" << objective(sol) << " Vehicles=";
    int v=0; for(auto &r:sol) if(r.size()>2) v++;
    logPopulation << v << "\n";
    for (int i=0; i<sol.size(); ++i) {
        logPopulation << " Route " << i << ":";
        for (int j=0; j<sol[i].size(); ++j) logPopulation << " " << sol[i][j];
        logPopulation << "\n";
    }
}

// ===== Create solution  =====
Solution randomSolution() {
    static int counter = 0;
    vector<bool> used(numCustomers, false);
    used[0] = true;  // دپو

    // ۱) پرموتیشن مشتری‌ها (به جز دپو)
    vector<int> perm;
    for (int i = 1; i < numCustomers; ++i) perm.push_back(i);
    shuffle(perm.begin(), perm.end(), rng);

    Solution sol;
    ofstream logEvents("logEvents.txt", ios::app);
    logEvents << "=== Random-Insert Solution #" << counter << " ===\n";

    // ۲) برای هر مشتری در perm
    for (int c : perm) {
        bool inserted = false;

        // ۲-۱) شافل کردن ترتیب روت‌ها برای تنوع
        vector<int> order(sol.size());
        iota(order.begin(), order.end(), 0);
        shuffle(order.begin(), order.end(), rng);

        // ۲-۲) تلاش برای درج در هر روت موجود
        for (int ri : order) {
            auto &route = sol[ri];
            int L = route.size();

            // سعی می‌کنیم در یک جای تصادفی از 1 تا L-1 درج کنیم
            vector<int> positions(L - 1);
            iota(positions.begin(), positions.end(), 1);
            shuffle(positions.begin(), positions.end(), rng);

            for (int pos : positions) {
                // درج موقت
                route.insert(route.begin() + pos, c);
                if (validRoute(route)) {
                    used[c] = true;
                    inserted = true;
                    logEvents << "  Inserted cust " << c
                              << " into route#" << ri
                              << " at pos " << pos << "\n";
                    break;
                }
                // اگر نشد، undo
                route.erase(route.begin() + pos);
            }
            if (inserted) break;
        }

        // ۲-۳) اگر هیچ‌کدوم جا نشد، مسیر جدید بساز
        if (!inserted) {
            sol.push_back({0, c, 0});
            used[c] = true;
            logEvents << "  Created new route for cust " << c << "\n";
        }
    }

    logSolution(sol, counter++);
    return sol;
}

Solution solomonInitialSolution() {
    vector<bool> inserted(numCustomers, false);
    inserted[0] = true;

    Solution sol;

    for (int v = 0; v < vehicleCount; ++v) {
        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            int best = -1;
            double minCost = 1e9;

            for (int i = 1; i < numCustomers; ++i) {
                if (inserted[i]) continue;

                double arrival = time + dist[current][i];
                arrival = max(arrival, (double)customers[i].readyTime);
                double finish = arrival + customers[i].serviceTime;

                if (finish > customers[i].dueTime || load + customers[i].demand > vehicleCapacity)
                    continue;

                double cost = arrival; // می‌تونی معیار بهتر تعریف کنی
                if (cost < minCost) {
                    minCost = cost;
                    best = i;
                }
            }

            if (best == -1) break;

            route.push_back(best);
            inserted[best] = true;
            time = max(time + dist[current][best], (double)customers[best].readyTime) + customers[best].serviceTime;
            load += customers[best].demand;
            current = best;
        }

        if (route.size() > 1) {
            route.push_back(0);
            sol.push_back(route);
        }
    }

    for (int i = 1; i < numCustomers; ++i) {
        if (!inserted[i]) {
            sol.push_back({0, i, 0});
        }
    }

    return sol;
}

Solution hybridInitialSolution() {
    vector<bool> inserted(numCustomers, false);
    inserted[0] = true;
    Solution sol;

    for (int v = 0; v < vehicleCount; ++v) {
        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            int best = -1;
            double minCost = 1e9;

            for (int i = 1; i < numCustomers; ++i) {
                if (inserted[i]) continue;

                double arrival = time + dist[current][i];
                arrival = max(arrival, (double)customers[i].readyTime);
                double finish = arrival + customers[i].serviceTime;

                if (finish > customers[i].dueTime || load + customers[i].demand > vehicleCapacity)
                    continue;

                double cost = arrival;
                if (cost < minCost) {
                    minCost = cost;
                    best = i;
                }
            }

            if (best == -1) break;

            route.push_back(best);
            inserted[best] = true;
            time = max(time + dist[current][best], (double)customers[best].readyTime) + customers[best].serviceTime;
            load += customers[best].demand;
            current = best;
        }

        if (route.size() > 1) {
            route.push_back(0);
            if (validRoute(route)) {
                sol.push_back(route);
            }
        }
    }

    vector<pair<double, int>> polarSorted;
    double depotX = customers[0].x, depotY = customers[0].y;

    for (int i = 1; i < numCustomers; ++i) {
        if (inserted[i]) continue;
        double dx = customers[i].x - depotX;
        double dy = customers[i].y - depotY;
        double angle = atan2(dy, dx);
        polarSorted.emplace_back(angle, i);
    }

    sort(polarSorted.begin(), polarSorted.end());

    for (auto& [_, cust] : polarSorted) {
        int bestRouteIdx = -1;
        int bestInsertPos = -1;
        double bestCostIncrease = 1e9;
        int bestLoad = 0;

        for (size_t i = 0; i < sol.size(); ++i) {
            auto& route = sol[i];
            for (size_t pos = 1; pos < route.size(); ++pos) {
                vector<int> temp = route;
                temp.insert(temp.begin() + pos, cust);
                if (validRoute(temp)) {
                    sol[i] = temp;
                    inserted[cust] = true;
                    break;
                }
            }
        }
    }

    return sol;
}

Solution greedySolution() {
    // ====== ۱. آماده‌سازی ======
    vector<bool> inserted(numCustomers, false);
    inserted[0] = true;     // دپو از ابتدا در نظر گرفته شده
    Solution sol;

    // ====== ۲. ساخت مسیرها با حریصانهٔ سولومون ======
    for (int v = 0; v < vehicleCount; ++v) {
        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            int bestCust = -1;
            double bestArrival = 1e18;

            // جستجوی مشتری بعدی با کمترین زمان ورود (Solomon)
            for (int i = 1; i < numCustomers; ++i) {
                if (inserted[i]) continue;
                double arrival = time + dist[current][i];
                arrival = max(arrival, (double)customers[i].readyTime);
                double finish = arrival + customers[i].serviceTime;

                if (finish > customers[i].dueTime ||
                    load + customers[i].demand > vehicleCapacity)
                    continue;

                if (arrival < bestArrival) {
                    bestArrival = arrival;
                    bestCust    = i;
                }
            }

            if (bestCust < 0) break;  // دیگر مشتری قابل‌درجی نیست

            // درج مشتری انتخابی
            route.push_back(bestCust);
            inserted[bestCust] = true;
            time    = bestArrival + customers[bestCust].serviceTime;
            load   += customers[bestCust].demand;
            current = bestCust;
        }

        // اگر حداقل یک مشتری اضافه شده، مسیر را ببند
        if (route.size() > 1) {
            route.push_back(0);
            sol.push_back(route);
        }
    }

    // ====== ۳. درج باقی‌مانده با روش قطبی–گرِیدی ======
    // ۳-۱) مرتب‌سازی قطبی بر اساس زاویه
    vector<pair<double,int>> polar;
    double dx0 = customers[0].x, dy0 = customers[0].y;
    for (int i = 1; i < numCustomers; ++i) {
        if (inserted[i]) continue;
        double ang = atan2(customers[i].y - dy0, customers[i].x - dx0);
        polar.emplace_back(ang, i);
    }
    sort(polar.begin(), polar.end());

    // ۳-۲) برای هر مشتری باقیمانده، بهترین درج را پیدا کن
    for (auto &[_, cust] : polar) {
        int  bestRoute = -1;
        int  bestPos   = -1;
        double bestInc = 1e18;

        // جستجو در هر مسیر و هر موقعیت ممکن
        for (int r = 0; r < (int)sol.size(); ++r) {
            auto &route = sol[r];
            int L = route.size();
            for (int pos = 1; pos < L; ++pos) {
                // هزینه افزایش مسافت
                int   u = route[pos-1];
                int   v = route[pos];
                double inc = dist[u][cust] + dist[cust][v] - dist[u][v];

                // بررسی اعتبار
                vector<int> tmp = route;
                tmp.insert(tmp.begin() + pos, cust);
                if (!validRoute(tmp)) continue;

                if (inc < bestInc) {
                    bestInc   = inc;
                    bestRoute = r;
                    bestPos   = pos;
                }
            }
        }

        if (bestRoute >= 0) {
            // درج در بهترین مکان پیدا شده
            sol[bestRoute].insert(sol[bestRoute].begin() + bestPos, cust);
            inserted[cust] = true;
        } else {
            // اگر هیچ‌کجا جا نشد، مسیر جدید بساز
            sol.push_back({0, cust, 0});
            inserted[cust] = true;
        }
    }

    // ====== ۴. اطمینان از سرویس همهٔ مشتری‌ها ======
    for (int i = 1; i < numCustomers; ++i) {
        if (!inserted[i]) {
            sol.push_back({0, i, 0});
        }
    }

    return sol;
}

vector<vector<int>> generateInitialSolution() {
    vector<vector<int>> solution;
    vector<bool> visited(numCustomers, false);
    visited[0] = true; // depot always visited

        vector<pair<double, int>> polarSorted;
        double depotX = customers[0].x;
        double depotY = customers[0].y;

        for (int i = 1; i < numCustomers; ++i) {
            double dx = customers[i].x - depotX;
            double dy = customers[i].y - depotY;
            double angle = atan2(dy, dx); 
            polarSorted.emplace_back(angle, i);
        }

        sort(polarSorted.begin(), polarSorted.end());


        for (auto& [_, cust] : polarSorted) {
            if (visited[cust]) continue;

            int bestRouteIdx = -1;
            int bestInsertPos = -1;
            double bestCostIncrease = 1e9;
            int bestLoad = 0;

            for (size_t i = 0; i < solution.size(); ++i) {
                auto& route = solution[i];
                for (size_t pos = 1; pos < route.size(); ++pos) {
                    vector<int> temp = route;
                    temp.insert(temp.begin() + pos, cust);
                    int load = 0;
                    if (validRoute(temp)) {
                        double costBefore = 0.0, costAfter = 0.0;
                        for (size_t j = 1; j < route.size(); ++j)
                            costBefore += dist[route[j - 1]][route[j]];
                        for (size_t j = 1; j < temp.size(); ++j)
                            costAfter += dist[temp[j - 1]][temp[j]];
                        double costDiff = costAfter - costBefore;
                        if (costDiff < bestCostIncrease) {
                            bestCostIncrease = costDiff;
                            bestRouteIdx = i;
                            bestInsertPos = pos;
                            bestLoad = load;
                        }
                    }
                }
            }

            if (bestRouteIdx != -1) {
                solution[bestRouteIdx].insert(solution[bestRouteIdx].begin() + bestInsertPos, cust);
                visited[cust] = true;
            } else {
   
                vector<int> newRoute = {0, cust, 0};
                int load = 0;
                if (validRoute(newRoute)) {
                    visited[cust] = true;
                    solution.push_back(newRoute);
                }
            }
        }
    return solution;
}

// ===== Population initialization =====
vector<Solution> initPopulation() {
    const double P_RANDOM = 0.6;
    vector<Solution> pop;
    for (int i = 0; i < POP_SIZE; ++i) {
        Solution s;
        if (uniform_real_distribution<>(0,1)(rng) < P_RANDOM) {
            s = generateInitialSolution();
            logEvents << "[Init] Using Solomon Initial Solution for Individual " << i << "\n";
        } else {
            s = randomSolution();
            logEvents << "[Init] Using Random Solution for Individual " << i << "\n";
        }
        logSolution(s, i);
        pop.push_back(s);
    }
    return pop;
}

// ===== selection =====
Solution tournament(const vector<Solution> &pop) {
    int k=3;
    Solution best;
    double bestFit = numeric_limits<double>::infinity();
    uniform_int_distribution<int> distPop(0, pop.size()-1);
    for (int i=0;i<k;i++) {
        int idx = distPop(rng);
        double f = objective(pop[idx]);
        if (f<bestFit) { bestFit=f; best=pop[idx]; }
    }
    logEvents << "Tournament selected solution with objective "<<bestFit<<"\n";
    return best;
}

Solution rouletteSelect(const vector<Solution>& pop, const vector<double>& fit) {
    double sum = accumulate(fit.begin(), fit.end(), 0.0);
    double r = uniform_real_distribution<>(0,sum)(rng);
    double acc=0;
    for(int i=0;i<pop.size();i++){
        acc += (sum - fit[i]); // چون کمترین fitness بهتر است
        if(acc >= r) return pop[i];
    }
    return pop.back();
}

Solution eliteSelect(const vector<Solution>& pop, const vector<double>& fit) {
    // برترین k را انتخاب کن
    int k = 3;
    vector<int> idx(pop.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&](int a,int b){return fit[a]<fit[b];});
    int pick = idx[ uniform_int_distribution<>(0, k-1)(rng) ];
    return pop[pick];
}

// ===== Crossover =====
pair<Solution, Solution> crossover(const Solution &a, const Solution &b) {
    int r1 = uniform_int_distribution<int>(0, a.size() - 1)(rng);
    int r2 = uniform_int_distribution<int>(0, b.size() - 1)(rng);

    Solution child1, child2;
    set<int> used1 = {0}, used2 = {0};

    // Copy selected routes
    child1.push_back(a[r1]);
    for (int i : a[r1]) used1.insert(i);

    child2.push_back(b[r2]);
    for (int i : b[r2]) used2.insert(i);

    // Add remaining routes from other parent
    auto fillChild = [&](const Solution &parent, Solution &child, set<int> &used) {
        for (auto &r : parent) {
            vector<int> cleanRoute;
            for (int c : r) {
                if (used.count(c) == 0) {
                    cleanRoute.push_back(c);
                    used.insert(c);
                }
            }
            if (!cleanRoute.empty()) {
                cleanRoute.insert(cleanRoute.begin(), 0);
                cleanRoute.push_back(0);
                child.push_back(cleanRoute);
            }
        }
    };

    fillChild(b, child1, used1);
    fillChild(a, child2, used2);

    return {child1, child2};
}

// ===== Mutation (Inversion) =====
void mutate(Solution &sol) {
    // احتمال جهش در هر روت
    const double pm = 0.3;
    for (auto &route : sol) {
        int L = route.size();
        if (L > 4 && uniform_real_distribution<>(0,1)(rng) < pm) {
            // انتخاب دو نقطه i < j در داخل روت (بدون 0ها)
            uniform_int_distribution<int> pick(1, L - 2);
            int i = pick(rng), j = pick(rng);
            if (i > j) swap(i, j);
            // معکوس‌سازی زیررشته [i..j]
            reverse(route.begin() + i, route.begin() + j + 1);
            // بررسی اعتبار
            if (!validRoute(route)) {
                reverse(route.begin() + i, route.begin() + j + 1);  // undo
            }
        }
    }
}

// ===== Local Search =====
double routeDistance(const vector<int>& route) {
    double total = 0.0;
    for (int i = 0; i < (int)route.size() - 1; ++i)
        total += dist[route[i]][route[i + 1]];
    return total;
}

vector<int> twoOptSwap(const vector<int>& route) {
    int n = route.size();
    if (n <= 4) return route;  // خیلی کوتاهه، تغییر نمی‌خواد

    vector<int> bestRoute = route;
    double bestDist = routeDistance(route);

    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 1; i < n - 2; ++i) {
            for (int j = i + 1; j < n - 1; ++j) {
                vector<int> newRoute = bestRoute;
                reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
                double newDist = routeDistance(newRoute);
                if (newDist < bestDist) {
                    bestRoute = newRoute;
                    bestDist = newDist;
                    improved = true;
                }
            }
        }
    }

    return bestRoute;
}

Solution applyLocalSearch(const Solution& sol) {
    Solution newSol;
    for (const auto& route : sol) {
        newSol.push_back(twoOptSwap(route));
    }
    return newSol;
}

// ===== Genetic Algorithm Main =====
Solution geneticAlgorithm(int max_time, int max_evaluations) {
    logBest << "Gen, BestObjective\n";
    auto population = initPopulation();
    Solution best = population[0];
    double bestFit = objective(best);
    // logGeneration << 0 << "," << bestFit << "\n";

    auto t_start = chrono::steady_clock::now();
    int gen = 1;
    // ب) ارزیابی اولیه
    vector<double> fitness(POP_SIZE);
    for(int i=0;i<POP_SIZE;i++){
        fitness[i] = objective(population[i]);
    }

    // ج) حلقه نسل‌ها
    while (true) {
        // 1) چک زمان
        // cout<<"[GA] Loop start gen="<<gen<<"\n"; cout.flush();
        auto t_now = chrono::steady_clock::now();
        double elapsed_sec = chrono::duration<double>(t_now - t_start).count();
        if (max_time > 0 && elapsed_sec >= max_time) {
            cout << "Stopping: time limit reached after " << elapsed_sec << "s\n";
            break;
        }
        // 2) چک تعداد ارزیابی
        if (max_evaluations > 0 && evaluationCounter >= max_evaluations) {
            cout << "Stopping: evaluation limit reached (" 
                 << evaluationCounter << ")\n";
            break;
        }

        // ۱) انتخاب والدین (Roulette یا Elite)
        vector<Solution> matingPool;
        for(int i=0;i<POP_SIZE;i++){
            if (useRoulette)
                matingPool.push_back(rouletteSelect(population, fitness));
            else
                matingPool.push_back(eliteSelect(population, fitness));
        }

        // ۲) تولید فرزندان با crossover+mutate
        vector<Solution> offspring;
        for(int i=0;i<POP_SIZE/2;i++){
            auto p1 = matingPool[2*i], p2 = matingPool[2*i+1];
            Solution c1, c2;
            if (uniform_real_distribution<>(0,1)(rng) < CROSSOVER_RATE)
                tie(c1,c2) = crossover(p1,p2);
            else
                c1=p1, c2=p2;
            mutate(c1); mutate(c2);

            c1 = applyLocalSearch(c1);
            c2 = applyLocalSearch(c2);

            offspring.push_back(c1);
            offspring.push_back(c2);
        }

        // ۳) ارزیابی فرزندان و ترکیب (μ+λ) برای نسل بعد
        vector<pair<double,Solution>> combined;
        combined.reserve(POP_SIZE + offspring.size());
        for(int i=0;i<POP_SIZE;i++)
            combined.emplace_back(fitness[i], population[i]);
        for(auto &c : offspring)
            combined.emplace_back(objective(c), c);

        sort(combined.begin(), combined.end(),
             [](auto &a, auto &b){ return a.first < b.first; });

        population.clear();
        fitness.clear();
        for(int i=0;i<POP_SIZE;i++){
            population.push_back(combined[i].second);
            fitness.push_back(combined[i].first);
        }
        // پس از مرتب‌سازی `combined`
        int elitismCount = 1;  // تعداد نخبه‌هایی که مستقیماً حفظ می‌شوند
        for (int i = 0; i < elitismCount; ++i) {
            population.push_back(combined[i].second);
            fitness.push_back(combined[i].first);
        }
        for (int i = elitismCount; i < POP_SIZE; ++i) {
            population.push_back(combined[i].second);
            fitness.push_back(combined[i].first);
        }


        if (fitness[0] < bestFit) {
        bestFit = fitness[0];
        best = population[0];
        }

        cout << "Gen " << gen << ", best=" << fitness[0] << ", evaluations=" << evaluationCounter << "\n";
        logBest << gen << "," << bestFit << "," << evaluationCounter << endl;
        gen++;
    }

    return population[0];
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
    // Derive output file name from input file name
    string outputFile;
    size_t lastSlash = inputFilename.find_last_of("/\\");
    string base = (lastSlash == string::npos) ? inputFilename : inputFilename.substr(lastSlash + 1);
    size_t dot = base.find_last_of('.');
    if (dot != string::npos) {
        base = base.substr(0, dot);
    }
    outputFile = base + "_output.txt";

    ofstream fout(outputFile);
    for (size_t i = 0; i < sol.size(); ++i) {
        fout << "Route " << i + 1 << ": ";
        for (size_t j = 1; j < sol[i].size() - 1; ++j)
            fout << sol[i][j] << " ";
        fout << "\n";
    }
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

    auto best = geneticAlgorithm(max_time,max_evaluations);

    auto t1 = chrono::steady_clock::now();

    outputSolution(best, file);

    cout << "\n========== Execution Summary ==========";
    cout << "\n Total Runtime: " 
         << chrono::duration_cast<chrono::seconds>(t1-t0).count() 
         << " seconds\n";
    cout << " Max Time Allowed: " << max_time << " seconds\n";
    cout << " Max Evaluations Allowed: " << max_evaluations << "\n";

    if (isFeasible(best)) {
        cout << "Solution is valid and feasible.\n";
    } else {
        cout << "Solution is NOT valid!\n";
    }

    // close logs
    logPopulation.close();
    logBest.close();
    logEvents.close();

    return 0;
}
