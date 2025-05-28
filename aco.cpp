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

// === aco ===

const int POP_SIZE = 30; // تعداد مورچه‌ها
const double ALPHA = 1.0;    // وزن فرومون
const double BETA = 2.0;     // وزن تابع اکتشافی (جاذبه)
const double EVAPORATION = 0.5;  // نرخ تبخیر فرومون
const double Q = 100.0;      // ضریب تقویت فرومون

vector<vector<double>> pheromone;


// Logging streams
ofstream logEvents("logEvents.txt");
ofstream logPopulation("logPopulation.txt");
ofstream logBest("logBest.txt");

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

// ===== log =====
void logSolution(const Solution &sol, int solId) {
    logPopulation << "Solution " << solId << ": Objective=" << objective(sol) << "total cost:" << totalCost(sol) << " Vehicles=";
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
    used[0] = true;

    logEvents << "[INFO] === Start RandomSolution #" << counter << " ===" << endl;

    vector<int> perm(numCustomers - 1);
    iota(perm.begin(), perm.end(), 1);
    shuffle(perm.begin(), perm.end(), rng);

    Solution sol;
    int attempts = 0;
    int insertedCount = 0;

    for (int cust : perm) {
        bool inserted = false;

        vector<int> order(sol.size());
        iota(order.begin(), order.end(), 0);
        shuffle(order.begin(), order.end(), rng);

        for (int ri : order) {
            auto& route = sol[ri];
            int L = route.size();
            vector<int> pos(L - 1);
            iota(pos.begin(), pos.end(), 1);
            shuffle(pos.begin(), pos.end(), rng);

            for (int p : pos) {
                attempts++;
                route.insert(route.begin() + p, cust);

                if (validRoute(route)) {
                    used[cust] = true;
                    inserted = true;
                    insertedCount++;
                    logEvents << "[INFO] Inserted customer " << cust << " into route " << ri << " at position " << p << endl;
                    break;
                }

                route.erase(route.begin() + p);
                logEvents << "[WARN] Failed to insert customer " << cust << " into route " << ri << " at position " << p << endl;
            }

            if (inserted) break;
        }

        if (!inserted) {
            sol.push_back({0, cust, 0});
            used[cust] = true;
            insertedCount++;
            logEvents << "[INFO] Created new route for customer " << cust << endl;
        }
    }

    double cost = totalCost(sol);
    logEvents << "[INFO] RandomSolution #" << counter << " completed. Attempts=" << attempts
              << ", Inserted=" << insertedCount << ", Routes=" << sol.size()
              << ", TotalCost=" << cost << endl;

    logSolution(sol, counter++);
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
    const double P_RANDOM = 0.3;
    vector<Solution> pop;

    for (int i = 0; i < POP_SIZE; ++i) {
        double r = uniform_real_distribution<>(0, 1)(rng);

        if (r < P_RANDOM) {
            logEvents << "[INFO] Init individual " << i << " using Solomon InitialSolution." << endl;
            pop.push_back(generateInitialSolution());
        } else {
            logEvents << "[INFO] Init individual " << i << " using RandomSolution." << endl;
            pop.push_back(randomSolution());
        }

        logSolution(pop.back(), i);
    }

    logEvents << "[INFO] Population initialized with size " << pop.size() << endl;
    return pop;
}

// === 
void initializePheromone() {
    int n = customers.size();
    pheromone.assign(n, vector<double>(n, 1.0));
}

Solution constructAntSolution() {
    vector<bool> visited(numCustomers, false);
    visited[0] = true;
    Solution sol;

    while (any_of(visited.begin() + 1, visited.end(), [](bool v){ return !v; })) {
        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            vector<pair<int, double>> probabilities;
            double sumProb = 0.0;

            for (int i = 1; i < numCustomers; ++i) {
                if (visited[i]) continue;

                int demand = customers[i].demand;
                if (load + demand > vehicleCapacity) continue;

                double arrival = time + dist[current][i];
                if (arrival > customers[i].dueTime) continue;

                double tau = pow(pheromone[current][i], ALPHA);
                double eta = pow(1.0 / (1.0 + dist[current][i]), BETA);
                double prob = tau * eta;

                probabilities.emplace_back(i, prob);
                sumProb += prob;
            }

            if (probabilities.empty()) break;

            // انتخاب مشتری بعدی بر اساس احتمال
            uniform_real_distribution<> dis(0, sumProb);
            double r = dis(rng);
            double accum = 0;
            int selected = -1;

            for (auto& [i, p] : probabilities) {
                accum += p;
                if (r <= accum) {
                    selected = i;
                    break;
                }
            }

            if (selected == -1) break;

            route.push_back(selected);
            visited[selected] = true;
            time = max(time + dist[current][selected], (double)customers[selected].readyTime) + customers[selected].serviceTime;
            load += customers[selected].demand;
            current = selected;
        }

        route.push_back(0);
        sol.push_back(route);
    }

    return sol;
}

void updatePheromone(const vector<Solution>& ants, const Solution& best) {
    // تبخیر
    for (auto& row : pheromone)
        for (auto& p : row)
            p *= (1.0 - EVAPORATION);

    // تقویت توسط همه مورچه‌ها
    for (const auto& sol : ants) {
        double quality = Q / (totalCost(sol) + penaltyTerm(sol));

        for (const auto& route : sol) {
            for (size_t i = 1; i < route.size(); ++i) {
                int u = route[i - 1];
                int v = route[i];
                pheromone[u][v] += quality;
                pheromone[v][u] += quality;
            }
        }
    }

    // تقویت بیشتر توسط بهترین
    double bestQ = Q * 2.0 / (totalCost(best) + penaltyTerm(best));
    for (const auto& route : best) {
        for (size_t i = 1; i < route.size(); ++i) {
            int u = route[i - 1];
            int v = route[i];
            pheromone[u][v] += bestQ;
            pheromone[v][u] += bestQ;
        }
    }
}

Solution antColonyOptimization(int maxTime, int maxEvaluations) {
    initializePheromone();
    Solution globalBest;
    double bestObj = numeric_limits<double>::max();

    int evals = 0;
    auto start = chrono::steady_clock::now();

    while(true) {
        // time/eval check
        auto now = chrono::steady_clock::now();
        if(maxTime>0 && chrono::duration_cast<chrono::seconds>(now-start).count() >= maxTime) break;
        if(maxEvaluations>0 && evals>=maxEvaluations) break;

        vector<Solution> ants;
        for (int i = 0; i < POP_SIZE; ++i) {
            Solution antSol = constructAntSolution();
            ants.push_back(antSol);

            double obj = objective(antSol);
            evals++;

            if (obj < bestObj) {
                bestObj = obj;
                globalBest = antSol;
                logBest << "[NEW BEST] Eval=" << evals << ", Obj=" << obj << "\n";
            }

            if (evals >= maxEvaluations) return globalBest;

            auto now = chrono::steady_clock::now();
            double elapsed = chrono::duration_cast<chrono::seconds>(now - start).count();
            if (elapsed >= maxTime) return globalBest;
        }

        updatePheromone(ants, globalBest);
    }

    return globalBest;
}

// --- ACO Core ---
// Solution constructSolution() {
//     vector<bool> visited(numCustomers, false);
//     Solution sol;
//     for(int ant=0; ant<numAnts; ++ant) {
//         Solution antSol;
//         visited.assign(numCustomers,false);
//         visited[0]=true;

//         // keep building routes
//         while(true) {
//             vector<int> route = {0};
//             double load=0, time=0;
//             int current=0;
//             // build one route
//             while(true) {
//                 // build candidate list
//                 vector<int> cand;
//                 vector<double> prob;
//                 for(int j=1;j<numCustomers;j++) if(!visited[j]) {
//                     // check capacity and time feasibility heuristically
//                     if(load+customers[j].demand<=vehicleCapacity) {
//                         double arrival = max(time+dist[current][j], (double)customers[j].readyTime);
//                         if(arrival <= customers[j].dueTime) {
//                             cand.push_back(j);
//                             double tau = pow(pheromone[current][j], alpha);
//                             double eta = pow(1.0/(dist[current][j]+1e-6), beta);
//                             prob.push_back(tau*eta);
//                         }
//                     }
//                 }
//                 if(cand.empty()) break;
//                 // roulette wheel
//                 double sum=accumulate(prob.begin(),prob.end(),0.0);
//                 uniform_real_distribution<> dist01(0, sum);
//                 double r = dist01(rng);
//                 int idx=0;
//                 double cum=0;
//                 for(;idx<prob.size();idx++){
//                     cum += prob[idx];
//                     if(cum>=r) break;
//                 }
//                 int next = cand[idx];
//                 // move
//                 route.push_back(next);
//                 visited[next]=true;
//                 time = max(time+dist[current][next], (double)customers[next].readyTime)
//                        + customers[next].serviceTime;
//                 load += customers[next].demand;
//                 current = next;
//             }
//             route.push_back(0);
//             sol.push_back(route);
//             // if all customers visited, break
//             bool done=true;
//             for(int i=1;i<numCustomers;i++) if(!visited[i]) { done=false; break; }
//             if(done) break;
//         }
//         return sol;
//     }
//     return sol;
// }

// void updatePheromone(const vector<Solution> &population) {
//     // evaporation
//     for(int i=0;i<numCustomers;i++)
//         for(int j=0;j<numCustomers;j++)
//             pheromone[i][j] *= (1.0 - rho);
//     // deposit
//     for(const auto &sol : population) {
//         int used;
//         double cost = totalCost(sol, used);
//         double deposit = Q / cost;
//         for(const auto &route : sol) {
//             for(int i=1;i<route.size();i++) {
//                 int u=route[i-1], v=route[i];
//                 pheromone[u][v] += deposit;
//                 pheromone[v][u] += deposit;
//             }
//         }
//     }
// }

// Solution antColonyOptimization(int maxTime, int maxEvals) {
//     // init pheromone
//     pheromone.assign(numCustomers, vector<double>(numCustomers, 1.0));

//     auto start = chrono::steady_clock::now();
//     int evals = 0;
//     vector<Solution> population;

//     while(true) {
//         // time/eval check
//         auto now = chrono::steady_clock::now();
//         if(maxTime>0 && chrono::duration_cast<chrono::seconds>(now-start).count() >= maxTime) break;
//         if(maxEvals>0 && evals>=maxEvals) break;

//         // construct solutions
//         population.clear();
//         for(int k=0;k<numAnts;k++) {
//             Solution sol = constructSolution();
//             int u;
//             double cost = totalCost(sol,u);
//             evals++;
//             population.push_back(sol);
//             if(cost < bestCost) {
//                 bestCost = cost;
//                 bestSolution = sol;
//             }
//         }
//         // pheromone update
//         updatePheromone(population);
//     }
//     return bestSolution;
// }

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

    auto best = antColonyOptimization(max_time, max_evaluations);

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
