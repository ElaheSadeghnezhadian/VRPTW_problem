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
#include <functional>   // برای std::hash
#include <utility>      // برای std::pair

struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        // استفاده ساده از XOR و شیفت می‌تواند کافی باشد.
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

unordered_map<pair<int, int>, int, pair_hash> tabu_map; // استفاده از unordered_map برای حرکات تابو
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

// محاسبه فاصله اقلیدسی
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

// This ensures minimizing number of vehicles is prioritized
bool isBetter(const vector<vector<int>>& sol1, const vector<vector<int>>& sol2) {
    int v1 = 0, v2 = 0;
    for (const auto& r : sol1) if (r.size() > 2) v1++;
    for (const auto& r : sol2) if (r.size() > 2) v2++;

    if (v1 < v2) return true;
    if (v1 > v2) return false;

    double d1 = totalCost(sol1);
    double d2 = totalCost(sol2);
    return d1 < d2;
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

vector<vector<int>> generateInitialSolution(bool useGreedy = true) {
    vector<vector<int>> solution;
    vector<bool> visited(numCustomers, false);
    visited[0] = true; // depot is always visited
    
    if (useGreedy) {
        // Greedy mode: sort customers by distance from depot
        vector<pair<double, int>> sorted;
        for (int i = 1; i < numCustomers; ++i)
            sorted.emplace_back(dist[0][i], i);
        sort(sorted.begin(), sorted.end());
        
        // Try to insert each customer into existing routes; create new route if not possible
        for (auto &[_, cust] : sorted) {
            if (visited[cust]) continue;
            bool added = false;
            for (auto &route : solution) {
                vector<int> temp = route;
                temp.insert(temp.end() - 1, cust);
                int load = 0;
                if (validRoute(temp, load)) {
                    route.insert(route.end() - 1, cust);
                    visited[cust] = true;
                    added = true;
                    break;
                }
            }
            if (!added) {
                vector<int> newRoute = {0, cust, 0};
                int load = 0;
                if (validRoute(newRoute, load)) {
                    visited[cust] = true;
                    solution.push_back(newRoute);
                }
            }
        }
    }
    else {
        // Random mode: create a shuffled list of customers (excluding depot)
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


bool is_tabu(int a, int b) {
    // بررسی وجود حرکت در تابو
    return tabu_map.find({a, b}) != tabu_map.end() || tabu_map.find({b, a}) != tabu_map.end();
}

void add_tabu(int a, int b, int tenure = 5) {
    // اضافه کردن حرکت به تابو
    tabu_map[{a, b}] = tenure;
}

void decrement_tabu() {
    // کاهش مدت زمان تابو و حذف حرکات که مدت زمان آنها به صفر رسیده
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

vector<vector<vector<int>>> get_neighbors(const vector<vector<int>>& solution) {
    vector<vector<vector<int>>> neighbors;
    uniform_real_distribution<double> op_choice(0.0, 1.0);
    double op = op_choice(rng);

    // -----------------------------------
    // 1. اگر عدد انتخاب‌شده کمتر از 0.33 باشد، از عملگر 2-opt استفاده کن:
    // -----------------------------------
    if (op < 0.33) {
        for (size_t r = 0; r < solution.size(); ++r) {
            const vector<int>& route = solution[r];
            if (route.size() <= 3) continue;  // مسیر کوتاه قابل تغییر نیست
            for (size_t i = 1; i < route.size() - 2; ++i) {
                for (size_t j = i + 1; j < route.size() - 1; ++j) {
                    // فقط مسیر r را کپی می‌کنیم؛ سایر مسیرها بدون تغییر باقی می‌مانند
                    vector<vector<int>> newSol = solution;
                    vector<int>& newRoute = newSol[r]; // مرجع به مسیر تغییر یافته
                    reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
                    int load = 0;
                    if (validRoute(newRoute, load)) {
                        neighbors.push_back(newSol);
                    }
                }
            }
        }
    }
    // -----------------------------------
    // 2. اگر عدد انتخاب‌شده بین 0.33 و 0.66 باشد، از عملگر Relocate استفاده کن:
    // -----------------------------------
    else if (op < 0.66) {
        for (size_t r1 = 0; r1 < solution.size(); ++r1) {
            const vector<int>& route1 = solution[r1];
            if (route1.size() <= 3) continue;
            for (size_t r2 = 0; r2 < solution.size(); ++r2) {
                if (r1 == r2) continue;
                const vector<int>& route2 = solution[r2];
                // سعی در برداشتن یک مشتری از مسیر r1
                for (size_t i = 1; i < route1.size() - 1; ++i) {
                    int cust = route1[i];
                    vector<vector<int>> newSol = solution;
                    vector<int>& newRoute1 = newSol[r1];
                    newRoute1.erase(newRoute1.begin() + i); // حذف مشتری از r1
                    // درج مشتری در مسیر r2 در موقعیت‌های مختلف (به جز دپو)
                    for (size_t pos = 1; pos < newSol[r2].size(); ++pos) {
                        vector<vector<int>> tempSol = newSol; // کپی موقت جهت درج
                        vector<int>& tempRoute2 = tempSol[r2];
                        tempRoute2.insert(tempRoute2.begin() + pos, cust);
                        int load1 = 0, load2 = 0;
                        if (validRoute(tempSol[r1], load1) && validRoute(tempSol[r2], load2)) {
                            neighbors.push_back(tempSol);
                        }
                    }
                }
            }
        }
    }
    // -----------------------------------
    // 3. در غیر این صورت (op >= 0.66): از عملگر Swap استفاده کن:
    // -----------------------------------
    else {
        for (size_t r1 = 0; r1 < solution.size(); ++r1) {
            const vector<int>& route1 = solution[r1];
            if (route1.size() <= 2) continue;
            for (size_t r2 = r1 + 1; r2 < solution.size(); ++r2) {
                const vector<int>& route2 = solution[r2];
                if (route2.size() <= 2) continue;
                for (size_t i = 1; i < route1.size() - 1; ++i) {
                    for (size_t j = 1; j < route2.size() - 1; ++j) {
                        vector<vector<int>> newSol = solution; // کپی کل راه‌حل
                        // تنها مسیرهای r1 و r2 تغییر می‌کنند
                        vector<int>& newRoute1 = newSol[r1];
                        vector<int>& newRoute2 = newSol[r2];
                        swap(newRoute1[i], newRoute2[j]);
                        int load1 = 0, load2 = 0;
                        if (validRoute(newRoute1, load1) && validRoute(newRoute2, load2)) {
                            neighbors.push_back(newSol);
                        }
                    }
                }
            }
        }
    }
    return neighbors;
}



void logIterationInfo(int evaluations, int vehiclesUsed, double cost, bool improved, const unordered_map<pair<int, int>, int, pair_hash>& tabu_map, const string& filename) {
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
    best_distance = totalCost(sol);  // مقدار اولیه بهترین هزینه برابر با هزینه حل اولیه

    int evaluations = 0;
    int iteration = 0;
    string logFilename = file.substr(0, file.find_last_of('.')) + "_tabu_report.csv";
    ofstream clearLog(logFilename); 
    clearLog.close();  // پاک‌سازی فایل گزارش (log) قبلی

    while (true) {
        auto now = chrono::steady_clock::now();
        double elapsedGlobal = chrono::duration_cast<chrono::seconds>(now - globalStart).count();

        // شرط توقف بر اساس زمان یا تعداد ارزیابی‌ها
        if ((max_time > 0 && elapsedGlobal >= max_time) || (max_evaluations > 0 && evaluations >= max_evaluations))
            break;

        auto neighbors = get_neighbors(sol);
        double best_neighbor_cost = 1e9;
        vector<vector<int>> best_route;
        int a_move = -1, b_move = -1;

        // بررسی همه همسایه‌ها
        for (const auto& neighbor : neighbors) {
            double cost = totalCost(neighbor);
            evaluations++;  // یک Evaluation انجام شد

            int move_a = -1, move_b = -1;
            // استخراج تغییر بین مسیرها برای ثبت حرکتی که انجام شده است
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
            // در صورتی که این حرکت در لیست تابو نباشد و راه‌حل معتبر باشد
            if (!is_tabu(move_a, move_b) && isFeasibleSolution(neighbor)) {
                if (best_route.empty() || isBetter(neighbor, best_route)) {
                    best_route = neighbor;
                    a_move = move_a;
                    b_move = move_b;
                    best_neighbor_cost = cost;
                }
            }
        }

        iteration++;  // یک Iteration کامل شد

        // شمارش وسایل نقلیه مورد استفاده در بهترین راه‌حل فعلی
        int vehiclesUsed = 0;
        for (const auto& route : best_route) {
            if (!route.empty() && route.size() > 2) {
                vehiclesUsed++;
            }
        }
        bool improved = isBetter(best_route, best_solution) && isFeasibleSolution(best_route);
        logIterationInfo(iteration, vehiclesUsed, best_neighbor_cost, improved, tabu_map, logFilename);

        // چاپ لاگ در کنسول برای مشاهده تعداد Iteration و Evaluations
        cout << "Iteration: " << iteration << "  Total Evaluations: " << evaluations << endl;

        if (improved) {
            best_solution = best_route;
            best_distance = totalCost(best_route);
            bestFeasibleSolution = best_route;
        }

        sol = best_route;  // به روز رسانی حل جاری برای Iteration بعدی
        if (a_move != -1 && b_move != -1)
            add_tabu(a_move, b_move);
        decrement_tabu();  // کاهش مدّت حرکات تابو
    }
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
