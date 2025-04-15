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

struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

using namespace std;

struct Customer {
    int id;
    double x, y;
    int demand;
    int readyTime;
    int dueTime;
    int serviceTime;
};

struct Route {
    vector<int> customers;
    double total_distance = 0.0;
};

struct TabuMove {
    int customer1, customer2;
    int tenure;
};


unordered_map<pair<int, int>, int, pair_hash> tabu_map; 
vector<Customer> customers;
vector<vector<double>> dist;
vector<vector<int>> best_solution;
vector<vector<int>> bestFeasibleSolution;
double best_distance ;
int vehicleCount, vehicleCapacity, numCustomers;
int max_evaluations, max_time;
string file;
mt19937 rng(time(nullptr));
chrono::time_point<chrono::steady_clock> globalStart;
vector<TabuMove> tabu_list;

double euclidean(const Customer &a, const Customer &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void buildDistanceMatrix() {
    int n = customers.size();
    dist.assign(n, vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist[i][j] = euclidean(customers[i], customers[j]);
}

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


bool validRoute(const vector<int>& route, int &load) {
    double time = 0.0;
    load = 0;
    if (route.front() != 0 || route.back() != 0) return false;
    for (size_t i = 1; i < route.size(); ++i) {
        int prev = route[i - 1], curr = route[i];
        double arrival = time + dist[prev][curr];
        time = max(arrival, (double)customers[curr].readyTime);
        if (time > customers[curr].dueTime) return false;
        if (curr != 0) {
            time += customers[curr].serviceTime;
            load += customers[curr].demand;
            if (load > vehicleCapacity) return false;
        }
    }
    return true;
}

double routeCost(const vector<int>& route) {
    double cost = 0.0;
    for (size_t i = 0; i < route.size() - 1; ++i)
        cost += dist[route[i]][route[i + 1]];
    return cost;
}

// Calculate total cost of a solution (sum of all route costs)
double totalCost(const vector<vector<int>>& sol) {
    double cost = 0.0;
    for (const auto& r : sol)
        cost += routeCost(r);
    return cost;
}

struct Solution {
    vector<vector<int>> routes;
    int vehicleCount;
    double cost;

    Solution(const vector<vector<int>>& r) : routes(r) {
        vehicleCount = 0;
        for (const auto& route : r) {
            if (route.size() > 2) vehicleCount++;
        }
        cost = totalCost(r); // این تابع باید درست کار کنه
    }
};

// This ensures minimizing number of vehicles is prioritized
bool isBetter(const Solution& s1, const Solution& s2) {
    if (s1.vehicleCount < s2.vehicleCount) return true;
    if (s1.vehicleCount > s2.vehicleCount) return false;
    return s1.cost < s2.cost;
}
vector<vector<int>> generateInitialSolution(bool useGreedy = true) {
    vector<vector<int>> solution;
    vector<bool> visited(numCustomers, false);
    visited[0] = true; // depot always visited

    if (useGreedy) {
        // مرحله‌ی اول: مرتب‌سازی مشتری‌ها به روش زاویه‌ای
        vector<pair<double, int>> polarSorted;
        double depotX = customers[0].x;
        double depotY = customers[0].y;

        for (int i = 1; i < numCustomers; ++i) {
            double dx = customers[i].x - depotX;
            double dy = customers[i].y - depotY;
            double angle = atan2(dy, dx); // زاویه قطبی
            polarSorted.emplace_back(angle, i);
        }

        sort(polarSorted.begin(), polarSorted.end());

        // مرحله‌ی دوم: درج در بهترین جای ممکن (مثل کد قبلیت)
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
                    if (validRoute(temp, load)) {
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
                // اگر نشد در هیچ مسیری قرار بگیره، مسیر جدید می‌سازیم
                vector<int> newRoute = {0, cust, 0};
                int load = 0;
                if (validRoute(newRoute, load)) {
                    visited[cust] = true;
                    solution.push_back(newRoute);
                }
            }
        }
    } else {
        // حالت تصادفی همان نسخه‌ی قبلی
        vector<int> custList;
        for (int i = 1; i < numCustomers; ++i)
            custList.push_back(i);
        shuffle(custList.begin(), custList.end(), rng);
        for (int cust : custList) {
            bool inserted = false;
            vector<int> indices(solution.size());
            for (size_t i = 0; i < solution.size(); ++i)
                indices[i] = i;
            shuffle(indices.begin(), indices.end(), rng);
            for (int idx : indices) {
                vector<int> temp = solution[idx];
                temp.insert(temp.end() - 1, cust);
                int load = 0;
                if (validRoute(temp, load)) {
                    solution[idx].insert(solution[idx].end() - 1, cust);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                vector<int> newRoute = {0, cust, 0};
                int load = 0;
                if (validRoute(newRoute, load))
                    solution.push_back(newRoute);
            }
        }
    }

    return solution;
}

// vector<vector<int>> generateInitialSolution(bool useGreedy = true) {
//     vector<vector<int>> solution;
//     vector<bool> visited(numCustomers, false);
//     visited[0] = true; // depot is always visited

//     if (useGreedy) {
//         // مرتب‌سازی مشتری‌ها براساس فاصله از دپو
//         vector<pair<double, int>> sorted;
//         for (int i = 1; i < numCustomers; ++i)
//             sorted.emplace_back(dist[0][i], i);
//         sort(sorted.begin(), sorted.end());

//         for (auto& [_, cust] : sorted) {
//             if (visited[cust]) continue;

//             int bestRouteIdx = -1;
//             int bestInsertPos = -1;
//             double bestCostIncrease = 1e9;
//             int bestLoad = 0;

//             // بررسی همه مسیرها و همه مکان‌های ممکن برای درج
//             for (size_t i = 0; i < solution.size(); ++i) {
//                 auto& route = solution[i];
//                 for (size_t pos = 1; pos < route.size(); ++pos) {
//                     vector<int> temp = route;
//                     temp.insert(temp.begin() + pos, cust);
//                     int load = 0;
//                     if (validRoute(temp, load)) {
//                         double costBefore = 0.0, costAfter = 0.0;
//                         for (size_t j = 1; j < route.size(); ++j)
//                             costBefore += dist[route[j - 1]][route[j]];
//                         for (size_t j = 1; j < temp.size(); ++j)
//                             costAfter += dist[temp[j - 1]][temp[j]];
//                         double costDiff = costAfter - costBefore;
//                         if (costDiff < bestCostIncrease) {
//                             bestCostIncrease = costDiff;
//                             bestRouteIdx = i;
//                             bestInsertPos = pos;
//                             bestLoad = load;
//                         }
//                     }
//                 }
//             }

//             if (bestRouteIdx != -1) {
//                 solution[bestRouteIdx].insert(solution[bestRouteIdx].begin() + bestInsertPos, cust);
//                 visited[cust] = true;
//             } else {
//                 vector<int> newRoute = {0, cust, 0};
//                 int load = 0;
//                 if (validRoute(newRoute, load)) {
//                     visited[cust] = true;
//                     solution.push_back(newRoute);
//                 }
//             }
//         }
//     }
//     else {
//         // حالت تصادفی بدون تغییر
//         vector<int> custList;
//         for (int i = 1; i < numCustomers; ++i)
//             custList.push_back(i);
//         shuffle(custList.begin(), custList.end(), rng);
//         for (int cust : custList) {
//             bool inserted = false;
//             vector<int> indices(solution.size());
//             for (size_t i = 0; i < solution.size(); ++i)
//                 indices[i] = i;
//             shuffle(indices.begin(), indices.end(), rng);
//             for (int idx : indices) {
//                 vector<int> temp = solution[idx];
//                 temp.insert(temp.end() - 1, cust);
//                 int load = 0;
//                 if (validRoute(temp, load)) {
//                     solution[idx].insert(solution[idx].end() - 1, cust);
//                     inserted = true;
//                     break;
//                 }
//             }
//             if (!inserted) {
//                 vector<int> newRoute = {0, cust, 0};
//                 int load = 0;
//                 if (validRoute(newRoute, load))
//                     solution.push_back(newRoute);
//             }
//         }
//     }

//     return solution;
// }

vector<vector<int>> clarkeWrightInitialSolutionWithTimeWindows() {
    vector<vector<int>> routes;
    vector<int> routeIndex(numCustomers, -1);
    vector<int> routeLoad;
    vector<double> routeEndTime;

    // مرحله 1: مسیر اولیه برای هر مشتری
    for (int i = 1; i < numCustomers; ++i) {
        vector<int> route = {0, i, 0};
        routes.push_back(route);
        routeIndex[i] = i - 1;
        routeLoad.push_back(customers[i].demand);

        // محاسبه زمان اتمام سرویس این مسیر
        double arrival = dist[0][i];
        double startService = max((double)customers[i].readyTime, arrival);
        double finishService = startService + customers[i].serviceTime + dist[i][0];
        routeEndTime.push_back(finishService);
    }

    // مرحله 2: محاسبه savings
    struct Saving {
        int i, j;
        double value;
        bool operator<(const Saving& s) const {
            return value > s.value;
        }
    };

    vector<Saving> savings;
    for (int i = 1; i < numCustomers; ++i) {
        for (int j = i + 1; j < numCustomers; ++j) {
            double s = dist[0][i] + dist[0][j] - dist[i][j];
            savings.push_back({i, j, s});
        }
    }
    sort(savings.begin(), savings.end());

    // مرحله 3: ترکیب مسیرها با بررسی پنجره زمانی
    for (const auto& s : savings) {
        int i = s.i, j = s.j;
        int r1 = routeIndex[i];
        int r2 = routeIndex[j];

        if (r1 == -1 || r2 == -1 || r1 == r2) continue;

        auto& route1 = routes[r1];
        auto& route2 = routes[r2];

        if (route1[route1.size() - 2] == i && route2[1] == j) {
            int newLoad = routeLoad[r1] + routeLoad[r2];
            if (newLoad > vehicleCapacity) continue;

            // بررسی امکان پذیر بودن ترکیب از نظر پنجره زمانی
            vector<int> newRoute = route1;
            newRoute.pop_back();
            newRoute.insert(newRoute.end(), route2.begin() + 1, route2.end());

            double time = 0;
            bool feasible = true;
            for (size_t k = 1; k < newRoute.size(); ++k) {
                int from = newRoute[k - 1];
                int to = newRoute[k];
                time += dist[from][to];
                time = max(time, (double)customers[to].readyTime);
                if (time > customers[to].dueTime) {
                    feasible = false;
                    break;
                }
                time += customers[to].serviceTime;
            }

            if (feasible) {
                // ترکیب مسیرها
                route1 = newRoute;
                routeLoad[r1] = newLoad;
                routeEndTime[r1] = time;

                for (int k = 1; k < route2.size() - 1; ++k) {
                    routeIndex[route2[k]] = r1;
                }
                route2.clear();
                routeLoad[r2] = 0;
                routeEndTime[r2] = 0;
            }
        }
    }

    // مرحله آخر: حذف مسیرهای خالی
    vector<vector<int>> finalRoutes;
    vector<bool> visited(numCustomers, false);
    visited[0] = true;

    for (const auto& route : routes) {
        if (!route.empty()) {
            finalRoutes.push_back(route);
            for (size_t i = 1; i < route.size() - 1; ++i)
                visited[route[i]] = true;
        }
    }

    int visitedCount = 0;
    for (int i = 1; i < numCustomers; ++i)
        if (visited[i]) visitedCount++;

    if (visitedCount < numCustomers - 1) {
        cout << "⚠️ Warning: Not all customers were visited! Visited: " << visitedCount
             << "/" << (numCustomers - 1) << endl;
    }

    return finalRoutes;
}

vector<vector<vector<int>>> get_neighbors(const vector<vector<int>>& solution) {
    const size_t MAX_NEIGHBORS = 200;
    vector<vector<vector<int>>> neighbors;
    uniform_real_distribution<double> op_choice(0.0, 1.0);
    double op = op_choice(rng);

    stringstream logs;

    vector<int> routeIndices(solution.size());
    iota(routeIndices.begin(), routeIndices.end(), 0);
    shuffle(routeIndices.begin(), routeIndices.end(), rng);

    if (op < 0.33) {
        logs << "[INFO] Applying 2-opt\n";
        for (int r : routeIndices) {
            const vector<int>& route = solution[r];
            if (route.size() <= 3) continue;
            for (size_t i = 1; i < route.size() - 2; ++i) {
                for (size_t j = i + 1; j < route.size() - 1; ++j) {
                    vector<vector<int>> newSol = solution;
                    vector<int>& newRoute = newSol[r];
                    reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
                    int load = 0;
                    if (validRoute(newRoute, load)) {
                        neighbors.push_back(newSol);
                        logs << "Generated neighbor via 2-opt on route " << r
                             << ", reversed [" << i << ", " << j << "]\n";
                        if (neighbors.size() >= MAX_NEIGHBORS) goto finish;
                    }
                }
            }
        }
    }
    else if (op < 0.66) {
        logs << "[INFO] Applying Relocate\n";
        for (int r1 : routeIndices) {
            const vector<int>& route1 = solution[r1];
            if (route1.size() <= 3) continue;
            for (int r2 : routeIndices) {
                if (r1 == r2) continue;
                for (size_t i = 1; i < route1.size() - 1; ++i) {
                    int cust = route1[i];
                    vector<vector<int>> newSol = solution;
                    vector<int>& newRoute1 = newSol[r1];
                    newRoute1.erase(newRoute1.begin() + i);
                    for (size_t pos = 1; pos < newSol[r2].size(); ++pos) {
                        vector<vector<int>> tempSol = newSol;
                        vector<int>& tempRoute2 = tempSol[r2];
                        tempRoute2.insert(tempRoute2.begin() + pos, cust);
                        int load1 = 0, load2 = 0;
                        if (validRoute(tempSol[r1], load1) && validRoute(tempSol[r2], load2)) {
                            neighbors.push_back(tempSol);
                            logs << "Generated neighbor via Relocate: moved customer " << cust
                                 << " from route " << r1 << " to route " << r2
                                 << " at position " << pos << "\n";
                            if (neighbors.size() >= MAX_NEIGHBORS) goto finish;
                        }
                    }
                }
            }
        }
    }
    else {
        logs << "[INFO] Applying Swap\n";
        for (size_t r1 = 0; r1 < solution.size(); ++r1) {
            const vector<int>& route1 = solution[r1];
            if (route1.size() <= 2) continue;
            for (size_t r2 = r1 + 1; r2 < solution.size(); ++r2) {
                const vector<int>& route2 = solution[r2];
                if (route2.size() <= 2) continue;
                for (size_t i = 1; i < route1.size() - 1; ++i) {
                    for (size_t j = 1; j < route2.size() - 1; ++j) {
                        vector<vector<int>> newSol = solution;
                        vector<int>& newRoute1 = newSol[r1];
                        vector<int>& newRoute2 = newSol[r2];
                        swap(newRoute1[i], newRoute2[j]);
                        int load1 = 0, load2 = 0;
                        if (validRoute(newRoute1, load1) && validRoute(newRoute2, load2)) {
                            neighbors.push_back(newSol);
                            logs << "Generated neighbor via Swap: route " << r1
                                 << " customer " << i << " <--> route " << r2
                                 << " customer " << j << "\n";
                            if (neighbors.size() >= MAX_NEIGHBORS) goto finish;
                        }
                    }
                }
            }
        }
    }

finish:
    ofstream logFile("neighbors_log.txt", ios::trunc);
    logFile << logs.str();
    logFile.close();

    return neighbors;
}

bool is_tabu(int a, int b) {
    return tabu_map.find({a, b}) != tabu_map.end() || tabu_map.find({b, a}) != tabu_map.end();
}

void add_tabu(int a, int b, int tenure = 5) {
    tabu_map[{a, b}] = tenure;
}

void decrement_tabu() {
    for (auto it = tabu_map.begin(); it != tabu_map.end();) {
        if (--it->second <= 0) {
            it = tabu_map.erase(it);
        } else {
            ++it;
        }
    }
}

bool isFeasibleSolution(const vector<vector<int>>& sol) {
    if ((int)sol.size() > vehicleCount) return false;
    vector<bool> visited(numCustomers, false);
    visited[0] = true;
    for (const auto &route : sol) {
        int load = 0;
        if (!validRoute(route, load)) return false;
        for (size_t i = 1; i + 1 < route.size(); i++) {
            int cust = route[i];
            if (visited[cust]) return false;
            visited[cust] = true;
        }
    }
    return all_of(visited.begin(), visited.end(), [](bool v) { return v; });
}

void logIterationInfo(int evaluations, int vehiclesUsed, double cost, bool improved, const vector<TabuMove>& tabu_list, const string& filename) {
    static ofstream logFile(filename, ios::app); // append mode
    // اگر اولین بار است، Header چاپ شود
    if (evaluations == 1) {
        logFile << "Evaluations,VehiclesUsed,Cost,Improved,TabuList\n";
    }
    logFile << evaluations << "," << vehiclesUsed << "," << fixed << setprecision(2) << cost << "," 
            << (improved ? "Yes" : "No") << ",\"";

    bool first = true;
    for (const auto &entry : tabu_map) {
        if (!first)
            logFile << "; ";
        logFile << "(" << entry.first.first << "-" << entry.first.second 
                << ",t=" << entry.second << ")";
        first = false;
    }
    logFile << "\"\n";
}

void VRPTWTabuSearch(int max_time, int max_evaluations) {
    auto sol = generateInitialSolution();
    best_solution = sol;
    best_distance = totalCost(sol); 

    int evaluations = 0;
    int iteration = 0;
    globalStart = chrono::steady_clock::now();
        string logFilename = "tabu_report.csv"; 
        ofstream clearLog(logFilename); clearLog.close();

    while (true) {
        auto now = chrono::steady_clock::now();
        double elapsedGlobal = chrono::duration_cast<chrono::seconds>(now - globalStart).count();
        
        if ((max_time > 0 && elapsedGlobal >= max_time) || (max_evaluations > 0 && evaluations >= max_evaluations))
            break;

        auto neighbors = get_neighbors(sol);
        double best_neighbor_cost = 1e9;
        vector<vector<int>> best_route;
        int a_move = -1, b_move = -1;

       
        for (const auto& neighbor : neighbors) {
            double cost = totalCost(neighbor);
              // یک Evaluation انجام شد

            int move_a = -1, move_b = -1;
            for (size_t r = 0; r < sol.size(); ++r) {
                if (r >= neighbor.size()) break;
                const auto& old_r = sol[r];
                const auto& new_r = neighbor[r];
                for (size_t i = 1; i + 1 < min(old_r.size(), new_r.size()); ++i) {
                    if (old_r[i] != new_r[i]) {
                        move_a = old_r[i];
                        move_b = new_r[i];
                        break;
                    }
                }
                if (move_a != -1) break;
            }
            if (!is_tabu(move_a, move_b) && isFeasibleSolution(neighbor)) {
                if (best_route.empty() || isBetter(neighbor, best_route)) {
                    best_route = neighbor;
                    a_move = move_a;
                    b_move = move_b;
                    best_neighbor_cost = cost;
                }
            }
        }
        evaluations++;
        iteration++;

      
        int vehiclesUsed = 0;
        for (const auto& route : best_route) {
            if (!route.empty() && route.size() > 2) {
                vehiclesUsed++;
            }
        }
        bool improved = isBetter(best_route, best_solution) && isFeasibleSolution(best_route);
        vector<TabuMove> tabu_list_vec;
        for (const auto& entry : tabu_map) {
            tabu_list_vec.push_back({entry.first.first, entry.first.second, entry.second});
        }
        logIterationInfo(iteration, vehiclesUsed, best_neighbor_cost, improved, tabu_list_vec, logFilename);

        // cout << "Iteration: " << iteration << "  Total Evaluations: " << evaluations << endl;

        if (improved) {
            best_solution = best_route;
            best_distance = totalCost(best_route);
            bestFeasibleSolution = best_route;
        }

        sol = best_route;
        if (a_move != -1 && b_move != -1)
            add_tabu(a_move, b_move);
        decrement_tabu();
    }
    auto globalEnd = chrono::steady_clock::now();
}

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

bool validateSolution(const vector<vector<int>>& solution) {
    if (solution.size() > static_cast<size_t>(vehicleCount)) {
        cout << " Number of vehicles exceeds the available vehicle count.\n";
        return false;
    }
    vector<bool> visited(numCustomers, false);
    visited[0] = true; // Mark depot as visited
    for (const auto& route : solution) {
        int load = 0;
        for (size_t i = 1; i < route.size() - 1; ++i) {
            int cust = route[i];
            if (visited[cust]) {
                cout << " Customer " << cust << " visited multiple times.\n";
                return false;
            }
            visited[cust] = true;
        }
        if (!validRoute(route, load)) return false;
    }
    return all_of(visited.begin(), visited.end(), [](bool v) { return v; });
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " [instance-file] [max-time] [max-eval]\n";
        return 1;
    }
    string file = argv[1];
    max_time = atoi(argv[2]);
    max_evaluations = atoi(argv[3]);

    readInstance(file);
    auto globalStart = chrono::steady_clock::now();

    VRPTWTabuSearch(max_time,max_evaluations);
    auto globalEnd = chrono::steady_clock::now();
    outputSolution(best_solution, file);
    vector<vector<int>> bestSol = bestFeasibleSolution;

    cout << "Runtime: " << chrono::duration_cast<chrono::seconds>(globalEnd - globalStart).count() << " seconds\n";
    cout << "\n========== Execution Summary ==========\n";
    cout << " Total Runtime: " 
         << chrono::duration_cast<chrono::seconds>(globalEnd - globalStart).count() 
         << " seconds\n";
    cout << " Max Time Allowed: " << max_time << " seconds\n";
    cout << " Max Evaluations Allowed: " << max_evaluations << "\n";
    
    if (!bestSol.empty() && validateSolution(bestSol)) {
        cout << " The solution is valid and feasible!\n";
    } else {
        cout << " The solution is not valid!\n";
    }
    return 0;
}
