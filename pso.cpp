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
struct Move {
    int from;  // index of customer in sequence
    int to;    // index to insert
};
struct Velocity {
    double strength; // چقدر به هدف نزدیک شویم
    double diversification; // شدت حرکات تصادفی

    Velocity(double s = 1.0, double d = 1.0)
        : strength(s), diversification(d) {}
};
struct Particle {
    Solution position;
    Solution bestPosition;
    double bestFitness;
    Velocity velocity;
};

const int SWARM_SIZE = 20;
const long long MAX_ITER = 10000000000000;

double W_start = 0.9;
double W_end   = 0.4;

double C1_start = 2.5;
double C1_end   = 1.5;

double C2_start = 1.5;
double C2_end   = 2.5;


// ofstream logFile("solution_validation.log");
ofstream logValidation("validation.log");
ofstream logIterations("pso_iterations.log");
ofstream logBest("best_solutions.log");
ofstream logInfeasible("infeasible_attempts.log");


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

// ===== validation ====
bool validRoute(const vector<int>& route, ostream& logStream) {
    int load = 0;
    double time = 0;
    if (route.front() != 0 || route.back() != 0) {
        logStream << "Route does not start and end at depot (0).\n";
        return false;
    }
    for (int i = 1; i < (int)route.size(); ++i) {
        int u = route[i - 1], v = route[i];
        time += dist[u][v];
        time = max(time, (double)customers[v].readyTime);
        if (time > customers[v].dueTime) {
            logStream << "Time window violated at customer " << v
                      << ". Arrival time: " << time
                      << ", Due time: " << customers[v].dueTime << "\n";
            return false;
        }
        if (v != 0) {
            time += customers[v].serviceTime;
            load += customers[v].demand;
            if (load > vehicleCapacity) {
                logStream << "Capacity exceeded in route at customer " << v
                          << ". Load: " << load
                          << ", Capacity: " << vehicleCapacity << "\n";
                return false;
            }
        }
    }
    return true;
}

bool isFeasible(const Solution& sol, ostream& logStream) {
    if ((int)sol.size() > vehicleCount) {
        logStream << "Number of routes (" << sol.size() << ") exceeds vehicle count (" << vehicleCount << ").\n";
        return false;
    }

    vector<bool> seen(numCustomers, false);
    seen[0] = true;

    for (size_t r = 0; r < sol.size(); ++r) {
        const auto& route = sol[r];
        logStream << "Checking route " << r + 1 << ":\n";

        if (!validRoute(route, logStream)) {
            logStream << "Route " << r + 1 << " is invalid.\n";
            return false;
        }

        for (int i = 1; i + 1 < (int)route.size(); ++i) {
            int cust = route[i];
            if (seen[cust]) {
                logStream << "Customer " << cust << " visited more than once.\n";
                return false;
            }
            seen[cust] = true;
        }
    }

    for (int i = 0; i < numCustomers; ++i) {
        if (!seen[i]) {
            logStream << "Customer " << i << " is not visited.\n";
            return false;
        }
    }

    return true;
}

// ===== Random solution generation =====
bool canInsertCustomer(const vector<int>& route, int cust, int pos, int currentLoad) {
    if (currentLoad + customers[cust].demand > vehicleCapacity)
        return false;

    // مسیر جدید با مشتری درج شده
    vector<int> newRoute = route;
    newRoute.insert(newRoute.begin() + pos, cust);

    double currentTime = 0;
    for (int i = 1; i < (int)newRoute.size(); ++i) {
        int prev = newRoute[i - 1];
        int curr = newRoute[i];
        currentTime += dist[prev][curr];
        currentTime = max(currentTime, (double)customers[curr].readyTime);
        if (currentTime > customers[curr].dueTime)
            return false;
        currentTime += customers[curr].serviceTime;
    }
    return true;
}

Solution randomSolution() {
    vector<int> customers_to_assign(numCustomers - 1);
    iota(customers_to_assign.begin(), customers_to_assign.end(), 1);

    // مرتب سازی نزولی بر اساس تقاضا
    sort(customers_to_assign.begin(), customers_to_assign.end(),
        [](int a, int b) {
            return customers[a].demand > customers[b].demand;
        });

    // افزودن کمی تصادف در ترتیب
    for (int i = 0; i < (int)customers_to_assign.size() - 1; ++i) {
        if ((rand() % 100) < 20) {
            int j = i + rand() % (customers_to_assign.size() - i);
            swap(customers_to_assign[i], customers_to_assign[j]);
        }
    }

    Solution sol;
    sol.emplace_back(vector<int>{0, 0});  // مسیر اول با فقط انبار
    vector<int> routeLoads(1, 0);

    for (int cust : customers_to_assign) {
        vector<tuple<double, int, int>> candidates;

        for (int r = 0; r < (int)sol.size(); ++r) {
            if (routeLoads[r] + customers[cust].demand > vehicleCapacity)
                continue;

            auto& route = sol[r];
            int currentLoad = routeLoads[r];

            for (int pos = 1; pos < (int)route.size(); ++pos) {
                if (!canInsertCustomer(route, cust, pos, currentLoad))
                    continue;

                int prev = route[pos - 1], next = route[pos];
                double delta =
                    dist[prev][cust] +
                    dist[cust][next] - dist[prev][next];
                candidates.emplace_back(delta, r, pos);
            }
        }

        // امکان باز کردن مسیر جدید
        if ((int)sol.size() < vehicleCount) {
            // مسیر جدید: 0 -> cust -> 0
            double newRouteCost = 2 * dist[0][cust];
            candidates.emplace_back(newRouteCost, -1, -1);
        }

        if (!candidates.empty()) {
            sort(candidates.begin(), candidates.end());
            int pick = 0;
            if (candidates.size() > 3 && (rand() % 100) < 30)
                pick = rand() % min(3, (int)candidates.size());

            auto [_, r, pos] = candidates[pick];

            if (r == -1) {
                sol.emplace_back(vector<int>{0, cust, 0});
                routeLoads.push_back(customers[cust].demand);
            } else {
                sol[r].insert(sol[r].begin() + pos, cust);
                routeLoads[r] += customers[cust].demand;
            }
        } else {
            // اگر هیچ جایی مناسب نبود، مسیر جدید می‌سازیم
            sol.emplace_back(vector<int>{0, cust, 0});
            routeLoads.push_back(customers[cust].demand);
        }
    }

    return sol;
}

Solution reduceVehicles(Solution sol) {
    bool merged = true;

    auto routeLoad = [](const vector<int>& route) {
        int load = 0;
        for (int i = 1; i + 1 < route.size(); ++i)
            load += customers[route[i]].demand;
        return load;
    };

    while (merged) {
        merged = false;

        double bestSaving = 0.0;
        int bestI = -1, bestJ = -1;
        vector<int> bestMerged;

        for (int i = 0; i < (int)sol.size(); ++i) {
            for (int j = i + 1; j < (int)sol.size(); ++j) {
                auto& route1 = sol[i];
                auto& route2 = sol[j];
                int totalLoad = routeLoad(route1) + routeLoad(route2);
                if (totalLoad > vehicleCapacity) continue;

                vector<int> customers2(route2.begin() + 1, route2.end() - 1);
                vector<int> mergedRoute = route1;
                mergedRoute.insert(mergedRoute.end() - 1, customers2.begin(), customers2.end());

                if (validRoute(mergedRoute,logValidation)) {
                    double saving = routeCost(route1) + routeCost(route2) - routeCost(mergedRoute);
                    if (saving > bestSaving) {
                        bestSaving = saving;
                        bestI = i;
                        bestJ = j;
                        bestMerged = mergedRoute;
                    }
                }
            }
        }

        if (bestI != -1) {
            sol[bestI] = std::move(bestMerged);
            sol.erase(sol.begin() + bestJ);
            merged = true;
        }
    }

    return sol;
}

Solution repairTimeWindows(Solution sol) {
    vector<int> removedCustomers;

    // حذف مشتریانی که تایم ویندو نقض شده
    for (auto& route : sol) {
        double currentTime = 0;
        for (int pos = 1; pos + 1 < (int)route.size();) {
            int prev = route[pos - 1];
            int curr = route[pos];
            currentTime += dist[prev][curr];
            currentTime = max(currentTime, (double)customers[curr].readyTime);

            if (currentTime > customers[curr].dueTime) {
                // حذف مشتری از مسیر و افزودنش به لیست حذف‌شده‌ها
                removedCustomers.push_back(curr);
                route.erase(route.begin() + pos);
                currentTime -= dist[prev][curr];  // چون حذف کردیم، از زمان کمش می‌کنیم
                // pos ثابت می‌مونه چون ایندکس‌ها عقب کشیده شدن
            } else {
                currentTime += customers[curr].serviceTime;
                pos++;
            }
        }
    }

    // تلاش برای افزودن دوباره مشتری‌های حذف شده
    for (int cust : removedCustomers) {
        double bestCost = numeric_limits<double>::max();
        int bestRoute = -1, bestPos = -1;

        for (int r = 0; r < (int)sol.size(); ++r) {
            auto& route = sol[r];
            int load = 0;
            for (int i = 1; i + 1 < (int)route.size(); ++i)
                load += customers[route[i]].demand;
            if (load + customers[cust].demand > vehicleCapacity) continue;

            for (int pos = 1; pos < (int)route.size(); ++pos) {
                if (canInsertCustomer(route, cust, pos, load)) {
                    route.insert(route.begin() + pos, cust);
                    double cost = routeCost(route);
                    if (cost < bestCost) {
                        bestCost = cost;
                        bestRoute = r;
                        bestPos = pos;
                    }
                    route.erase(route.begin() + pos);
                }
            }
        }

        if (bestRoute != -1) {
            sol[bestRoute].insert(sol[bestRoute].begin() + bestPos, cust);
        } else {
            // اگر هیچ جای خالی نبود، مسیر جدید باز کن
            sol.emplace_back(vector<int>{0, cust, 0});
        }
    }

    return sol;
}

// ===== Move towards best solution =====

vector<int> flatten(const Solution& sol) {
    vector<int> seq;
    for (const auto& r : sol)
        copy_if(r.begin(), r.end(), back_inserter(seq),
                [](int c) { return c != 0; });
    return seq;
}

double solutionCostWithPenalty(const Solution& sol) {
    double total = 0.0;
    for (const auto& r : sol) {
        int load = 0;
        for (size_t i = 0; i + 1 < r.size(); ++i) {
            total += dist[r[i]][r[i+1]];
            if (r[i] != 0) load += customers[r[i]].demand;
        }
        if (load > vehicleCapacity)
            total += 1e6 * (load - vehicleCapacity);
            
    }
    return total;
}

void applyDiversifiedMove(Solution& sol, double intensity = 1.0) {
    uniform_int_distribution<int> moveTypeDist(0, 2);
    int moveType = moveTypeDist(rng);

    if (moveType == 0) { // 2-opt like reversal
        int r = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        if (sol[r].size() > 4) {
            int i = uniform_int_distribution<int>(1, sol[r].size() - 3)(rng);
            int j = i + 1 + uniform_int_distribution<int>(0, sol[r].size() - i - 3)(rng);
            reverse(sol[r].begin() + i, sol[r].begin() + j + 1);
        }
    } else if (moveType == 1 && sol.size() >= 2) { // swap between routes
        int r1 = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        int r2 = r1;
        while (r2 == r1) r2 = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        if (sol[r1].size() > 2 && sol[r2].size() > 2) {
            int i = uniform_int_distribution<int>(1, sol[r1].size() - 2)(rng);
            int j = uniform_int_distribution<int>(1, sol[r2].size() - 2)(rng);
            swap(sol[r1][i], sol[r2][j]);
        }
    } else if (moveType == 2 && sol.size() >= 2) { // segment exchange
        int r1 = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        int r2 = r1;
        while (r2 == r1) r2 = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        if (sol[r1].size() > 4 && sol[r2].size() > 4) {
            int i1 = uniform_int_distribution<int>(1, sol[r1].size() - 2)(rng);
            int i2 = uniform_int_distribution<int>(1, sol[r2].size() - 2)(rng);
            vector<int> seg1(sol[r1].begin() + i1, sol[r1].end() - 1);
            vector<int> seg2(sol[r2].begin() + i2, sol[r2].end() - 1);
            sol[r1].erase(sol[r1].begin() + i1, sol[r1].end() - 1);
            sol[r2].erase(sol[r2].begin() + i2, sol[r2].end() - 1);
            sol[r1].insert(sol[r1].end() - 1, seg2.begin(), seg2.end());
            sol[r2].insert(sol[r2].end() - 1, seg1.begin(), seg1.end());
        }
    }
}

void moveTowardTarget(Solution& next, const Solution& target, const Velocity& v = Velocity{}) {
    auto currSeq = flatten(next);
    auto targetSeq = flatten(target);

    vector<size_t> diffPositions;
    for (size_t i = 0; i < currSeq.size(); ++i)
        if (currSeq[i] != targetSeq[i])
            diffPositions.push_back(i);

    if (!diffPositions.empty()) {
        int swapCount = uniform_int_distribution<int>(
            1, max(1, int(v.strength * diffPositions.size() / 4))
        )(rng);

        for (int s = 0; s < swapCount && !diffPositions.empty(); ++s) {
            size_t idx = uniform_int_distribution<size_t>(0, diffPositions.size() - 1)(rng);
            size_t pos = diffPositions[idx];
            auto it = find(currSeq.begin() + pos + 1, currSeq.end(), targetSeq[pos]);
            if (it != currSeq.end()) iter_swap(currSeq.begin() + pos, it);
            diffPositions.erase(diffPositions.begin() + idx);
        }
    }

    applyDiversifiedMove(next, v.diversification);
}

Solution moveTowards(const Solution& current, const Solution& pbest, const Solution& gbest,
                     double w, double c1, double c2) {
    Solution next = current;
    uniform_real_distribution<> dist01(0.0, 1.0);

    if (dist01(rng) < c1) moveTowardTarget(next, pbest);
    if (dist01(rng) < c2) moveTowardTarget(next, gbest);

    if (solutionCostWithPenalty(next) >= solutionCostWithPenalty(current)) {
        applyDiversifiedMove(next);
    }

    return next;
}

// ===== Particle Swarm Optimization (PSO) =====
void updateVelocity(Velocity &v, double w, double c1, double c2) {
    double r1 = uniform_real_distribution<double>(0, 1)(rng);
    double r2 = uniform_real_distribution<double>(0, 1)(rng);

    v.strength = w * v.strength + c1 * r1 + c2 * r2;
    v.diversification = w * v.diversification + c1 * (1.0 - r1) + c2 * (1.0 - r2);
}

Solution particleSwarmOptimization(int maxTime, int maxEvaluations) {
    auto t_start = chrono::steady_clock::now();

    vector<Particle> swarm(SWARM_SIZE);
    Solution gbestPosition;
    Solution bestFeasible;
    double bestFeasibleFitness = numeric_limits<double>::max();

    double gbestFitness = numeric_limits<double>::max();
    
    for (auto &p : swarm) {
        p.position = randomSolution();
        p.position = reduceVehicles(p.position);
        p.position = repairTimeWindows(p.position);
        p.bestPosition = p.position;
        p.bestFitness = objective(p.position);
        p.velocity = Velocity(1.0, 1.0);
        
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

            updateVelocity(p.velocity, w, c1, c2);

            Solution candidate = p.position;

            moveTowardTarget(candidate, gbestPosition, p.velocity);

            // Solution candidate = moveTowards(p.position, p.bestPosition, gbestPosition, w, c1, c2);

            double fit = objective(candidate);
            if (fit < p.bestFitness) {
                p.bestPosition = candidate;
                p.bestFitness = fit;
            }

            if (fit < gbestFitness) {
                gbestFitness = fit;
                gbestPosition = candidate;
                logBest << "Iteration: " << iteration
                        << ", New gbestFitness: " << gbestFitness << "\n";
            }

            // اضافه: اگر candidate معتبر است و بهتر از bestFeasible است
            if (isFeasible(candidate, logValidation) && fit < bestFeasibleFitness) {
                bestFeasible = candidate;
                bestFeasibleFitness = fit;
            }


            p.position = candidate;
            p.position = reduceVehicles(p.position);  
                      
        }

        logIterations << "Iteration: " << iteration
                    << ", gbestFitness: " << gbestFitness
                    << ", vehicles: " << gbestPosition.size() << "\n";


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

    if (isFeasible(best, logValidation)) {
        cout << "Solution is valid and feasible.\n";
    } else {
        cout << "Solution is NOT valid!\n";
    }

    logValidation.close();
    logIterations.close();
    logBest.close();
    logInfeasible.close();

    return 0;
}
