#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <numeric>      // << اضافه شد
#include <random>
#include <limits>   
#include <unordered_set>
#include <string>


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
double alpha = 0.2;   

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
    double time = 0;
    load = 0;
    if (route.front() != 0 || route.back() != 0) return false;
    for (size_t i = 1; i < route.size(); ++i) {
        int prev = route[i - 1], curr = route[i];
        time += dist[prev][curr];
        time = max(time, (double)customers[curr].readyTime);
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
        cost = totalCost(r); 
    }
};

size_t hashSolution(const vector<vector<int>>& sol) {
    size_t h = 0;
    hash<int> hasher;
    for (const auto& r : sol)
        for (int i : r)
            h ^= hasher(i) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
}
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
   
                vector<int> newRoute = {0, cust, 0};
                int load = 0;
                if (validRoute(newRoute, load)) {
                    visited[cust] = true;
                    solution.push_back(newRoute);
                }
            }
        }
    } else {
       
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

#include <unordered_set>

vector<vector<vector<int>>> get_neighbors(const vector<vector<int>>& solution) {
    const size_t MAX_NEIGHBORS = 10;
    vector<vector<vector<int>>> neighbors;
    unordered_set<size_t> seen;
    stringstream logs;

    uniform_real_distribution<double> op_choice(0.0, 1.0);
    double op = op_choice(rng);

    // فقط relocate برای نمونه نمایش
    for (size_t r1 = 0; r1 < solution.size(); ++r1) {
        const auto& route1 = solution[r1];
        if (route1.size() <= 3) continue;
        for (size_t r2 = 0; r2 < solution.size(); ++r2) {
            if (r1 == r2) continue;
            for (size_t i = 1; i < route1.size() - 1; ++i) {
                int cust = route1[i];
                auto newSol = solution;
                auto& newRoute1 = newSol[r1];
                newRoute1.erase(newRoute1.begin() + i);
                for (size_t pos = 1; pos < newSol[r2].size(); ++pos) {
                    auto tempSol = newSol;
                    auto& newRoute2 = tempSol[r2];
                    newRoute2.insert(newRoute2.begin() + pos, cust);
                    int l1 = 0, l2 = 0;
                    if (validRoute(tempSol[r1], l1) && validRoute(tempSol[r2], l2)) {
                        size_t h = hashSolution(tempSol);
                        if (seen.count(h)) continue;
                        seen.insert(h);
                        neighbors.push_back(tempSol);
                        logs << "Relocate " << cust << " from " << r1 << " to " << r2 << " pos " << pos << " "<<"\n";
                        if (neighbors.size() >= MAX_NEIGHBORS) goto finish;
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

// ======== فاز ساخت GRASP ========
extern mt19937 rng;  // می‌تونید global یا محلی تعریفش کنید
vector<vector<int>> buildGRASPSolution() {
    vector<vector<int>> solution;
    vector<bool> inSol(numCustomers, false);
    inSol[0] = true; // دپو

    vector<int> remaining;
    for (int i = 1; i < numCustomers; ++i)
        remaining.push_back(i);

    while (!remaining.empty()) {
        struct Candidate { int cust, r, pos; double delta; };
        vector<Candidate> cands;

        // آزمون درج در هر مسیر
        for (int cust : remaining) {
            for (size_t r = 0; r < solution.size(); ++r) {
                auto &route = solution[r];
                for (size_t p = 1; p < route.size(); ++p) {
                    auto tmp = route;
                    tmp.insert(tmp.begin() + p, cust);
                    int load = 0;
                    if (!validRoute(tmp, load)) continue;
                    double cost_before = routeCost(route);
                    tmp.insert(tmp.begin() + p, cust);
                    double cost_after = routeCost(tmp);
                    double d = cost_after - cost_before;
                    cands.push_back({cust, (int)r, (int)p, d});
                }
            }
            // آزمون شروع مسیر جدید
            vector<int> nr = {0, cust, 0};
            int load = 0;
            if (validRoute(nr, load)) {
                cands.push_back({cust, -1, -1, routeCost(nr)});
            }
        }
        if (cands.empty()) break;

        // RCL و انتخاب تصادفی
        sort(cands.begin(), cands.end(),
             [](auto &a, auto &b){ return a.delta < b.delta; });
        int rcl_sz = max(1, (int)ceil(alpha * cands.size()));
        uniform_int_distribution<int> pick(0, rcl_sz-1);
        auto sel = cands[pick(rng)];

        // درج انتخاب‌شده
        if (sel.r >= 0)
            solution[sel.r].insert(
                solution[sel.r].begin()+sel.pos, sel.cust
            );
        else
            solution.push_back({0, sel.cust, 0});

        remaining.erase(
            remove(remaining.begin(), remaining.end(), sel.cust),
            remaining.end()
        );
    }
    return solution;
}

// ======== اصلاح تابع localSearch با بررسی زمان ========
vector<vector<int>> localSearch(const vector<vector<int>>& initSol,
    int max_time,
    const chrono::time_point<chrono::steady_clock>& start)
{
    auto cur = initSol;
    double curCost = totalCost(cur);
    bool improved = true;
    while (improved) {
    // بررسی پایان زمان مجاز
    auto now = chrono::steady_clock::now();
    if (max_time > 0 &&
    chrono::duration_cast<chrono::seconds>(now - start).count() >= max_time)
    {
    return cur; // خروج زودهنگام از جستجوی محلی
    }
    improved = false;
    auto nbs = get_neighbors(cur);
    for (auto& nb : nbs) {
        auto now = chrono::steady_clock::now();
        double elapsed = chrono::duration_cast<chrono::seconds>(now - start).count();
        if (max_time > 0 && elapsed >= max_time) break;
    
        double c = totalCost(nb);
        if (c < curCost) {
            cur = nb;
            curCost = c;
            improved = true;
            break;
        }
    }
    
    }
    return cur;
}

// ======== اصلاح تابع اصلی GRASP با شرط حلقه واضح ========
void VRPTW_GRASP(int max_time, int max_evals) {
    double bestCost = numeric_limits<double>::infinity();
    vector<vector<int>> bestSol;
    int evaluations = 0;
    int iterations  = 0;
    auto globalStart = chrono::steady_clock::now();

    // حلقه تا زمان یا تعداد ارزیابی‌ها
    while ((max_time <= 0 ||
    chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - globalStart).count() < max_time)
    && (max_evals <= 0 || evaluations < max_evals))
    {
    iterations++;
    // ساخت اولیه و ارزیابی
    auto sol   = buildGRASPSolution();   evaluations++;
    auto local = localSearch(sol, max_time, globalStart);       evaluations++;

    double cost = totalCost(local);
    int vehicles = 0;
    for (auto &r : local)
    if (r.size() > 2) vehicles++;

    if (cost < bestCost && isFeasibleSolution(local)) {
    bestCost = cost;
    bestSol  = local;
    }

    double elapsed = chrono::duration_cast<chrono::seconds>(
    chrono::steady_clock::now() - globalStart).count();

    // چاپ و لاگ
    cout << "Iter " << iterations
    << " | Eval " << evaluations
    << " | Time " << elapsed << "s"
    << " | Veh "  << vehicles
    << " | Best " << bestCost << "\n";
    ofstream("grasp_report.csv", ios::app)
    << iterations << ","
    << evaluations << ","
    << elapsed     << ","
    << vehicles    << ","
    << bestCost    << "\n";

    
    }

    // به‌روزرسانی خروجی‌ها
    best_solution        = bestSol;
    best_distance        = bestCost;
    bestFeasibleSolution = bestSol;
}


int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] 
             << " [instance-file] [max-time-sec] [max-evaluations]\n";
        return 1;
    }
    string file          = argv[1];
    int max_time     = atoi(argv[2]);       // ۰ یعنی بی‌نهایت زمان
    int max_evaluations  = atoi(argv[3]);       // ۰ یعنی بی‌نهایت ارزیابی

    readInstance(file);

    auto t0 = chrono::steady_clock::now();

    int actualEvals;
    double actualTime;
    VRPTW_GRASP(max_time, max_evaluations);

    auto t1 = chrono::steady_clock::now();

    outputSolution(best_solution, file);

    // double runtime = chrono::duration_cast<chrono::seconds>(t1 - t0).count();

    cout << "\n========== Execution Summary ==========\n";
    cout << " Total Runtime: " 
         << chrono::duration_cast<chrono::seconds>(t1-t0).count() 
         << " seconds\n";
    cout << " Max Time Allowed: " << max_time << " seconds\n";
    cout << " Max Evaluations Allowed: " << max_evaluations << "\n";

    if (validateSolution(bestFeasibleSolution)) {
        cout << "Solution is valid and feasible.\n";
    } else {
        cout << "Solution is NOT valid!\n";
    }
    return 0;
}
