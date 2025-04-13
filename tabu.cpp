// نسخه بهینه‌شده VRPTW با Tabu Search و ساختار بهتر
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <random>
#include <numeric>
#include <limits>

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

vector<Customer> customers;
vector<vector<double>> dist;
vector<vector<int>> best_solution;
double best_distance ;
int vehicleCount, vehicleCapacity, numCustomers;
int max_evaluations, max_time;
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
double combinedObjective(const vector<vector<int>>& sol) {
    const double PENALTY = 1e9;
    return sol.size() * PENALTY + totalCost(sol);
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
    for (const auto& move : tabu_list) {
        if ((move.customer1 == a && move.customer2 == b) ||
            (move.customer1 == b && move.customer2 == a)) return true;
    }
    return false;
}

void decrement_tabu() {
    for (auto& move : tabu_list) move.tenure--;
    tabu_list.erase(remove_if(tabu_list.begin(), tabu_list.end(),
        [](const TabuMove& m) { return m.tenure <= 0; }), tabu_list.end());
}

void add_tabu(int a, int b, int tenure = 7) {
    tabu_list.push_back({a, b, tenure});
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

    // 1. Intra-route swaps
    for (size_t r = 0; r < solution.size(); ++r) {
        const auto& route = solution[r];
        if (route.size() <= 3) continue;
        for (size_t i = 1; i + 2 < route.size(); ++i) {
            for (size_t j = i + 1; j + 1 < route.size(); ++j) {
                auto new_sol = solution;
                swap(new_sol[r][i], new_sol[r][j]);
                int load = 0;
                if (validRoute(new_sol[r], load))
                    neighbors.push_back(new_sol);
            }
        }
    }

    // 2. Inter-route relocate
    for (size_t r1 = 0; r1 < solution.size(); ++r1) {
        for (size_t r2 = 0; r2 < solution.size(); ++r2) {
            if (r1 == r2) continue;
            for (size_t i = 1; i + 1 < solution[r1].size(); ++i) {
                int cust = solution[r1][i];
                auto new_sol = solution;
                new_sol[r1].erase(new_sol[r1].begin() + i);

                // Try inserting into r2 in all positions
                for (size_t j = 1; j < new_sol[r2].size(); ++j) {
                    auto temp_sol = new_sol;
                    temp_sol[r2].insert(temp_sol[r2].begin() + j, cust);
                    int load1 = 0, load2 = 0;
                    if (validRoute(temp_sol[r1], load1) && validRoute(temp_sol[r2], load2))
                        neighbors.push_back(temp_sol);
                }
            }
        }
    }

    // 3. Inter-route swap
    for (size_t r1 = 0; r1 < solution.size(); ++r1) {
        for (size_t r2 = r1 + 1; r2 < solution.size(); ++r2) {
            for (size_t i = 1; i + 1 < solution[r1].size(); ++i) {
                for (size_t j = 1; j + 1 < solution[r2].size(); ++j) {
                    auto new_sol = solution;
                    swap(new_sol[r1][i], new_sol[r2][j]);
                    int load1 = 0, load2 = 0;
                    if (validRoute(new_sol[r1], load1) && validRoute(new_sol[r2], load2))
                        neighbors.push_back(new_sol);
                }
            }
        }
    }

    return neighbors;
}

void VRPTWTabuSearch() {
        auto sol = generateInitialSolution();
        best_solution = sol;
        best_distance = totalCost(sol);  // مقدار اولیه بهترین هزینه را برابر با هزینه حل اولیه قرار می‌دهیم
    
        int evaluations = 0;
        globalStart = chrono::steady_clock::now();
    
        while ((max_evaluations == 0 || evaluations < max_evaluations) &&
               chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - globalStart).count() < max_time) {
    
            auto neighbors = get_neighbors(sol);
            double best_neighbor_cost = 1e9;
            vector<vector<int>> best_route;
            int a_move = -1, b_move = -1;
    
            for (const auto& route : neighbors) {
                double cost = totalCost(route);
                if (cost < best_neighbor_cost) {
                    best_neighbor_cost = cost;
                    best_route = route;
                    if (!route.empty() && route[0].size() > 2) {
                        a_move = route[0][1];
                        b_move = route[0][2];
                    }
                }
            }
    
            static int iteration = 0;
            iteration++;
            cout << "Iteration " << iteration << ": ";
    
            // شمارش وسایل نقلیه استفاده‌شده در بهترین مسیر (best_route)
            int vehiclesUsed = 0;
            for (const auto& route : best_route) {
                if (!route.empty() && route.size() > 2) {  // مسیری که حداقل 2 مشتری دارد (به جز دپو)
                    vehiclesUsed++;
                }
            }
    
            cout << "Vehicles used: " << vehiclesUsed << ", Cost = " << fixed << setprecision(2) << best_neighbor_cost;
    
            if (best_neighbor_cost < best_distance && isFeasibleSolution(best_route)) {
                best_solution = best_route;
                best_distance = best_neighbor_cost;
                cout << " -> Improved!";
            } else {
                cout << " -> No improvement.";
            }
            cout << endl;
    
            sol = best_route;  // به روز رسانی بهترین حل برای مرحله بعدی
            evaluations++;
        }
    }

void outputSolution(const vector<vector<int>>& solution, const string& filename) {
    double total = 0.0;
    for (const auto &route : solution) total += routeCost(route);
    int vehiclesUsed = solution.size();

    int vehicles_used = 0;
    for (const auto& route : solution) {
        if (!route.empty()) vehicles_used++;
    }
    string outFile = filename.substr(0, filename.find_last_of('.')) + "_output.txt";
    ofstream fout(outFile);
    fout << "Vehicles used: " << vehiclesUsed << "\n";
    fout << "Total cost: " << fixed << setprecision(2) << total << "\n";
    for (size_t i = 0; i < solution.size(); ++i) {
        if (solution[i].size() > 2) { // فقط مسیرهایی که مشتری دارند
            cout << "Route " << i + 1 << ": ";
            for (size_t j = 1; j + 1 < solution[i].size(); ++j) // از دپو شروع و پایان نکنید
                cout << solution[i][j] << " ";
            cout << "| Cost: " << fixed << setprecision(2) << routeCost(solution[i]) << "\n";
        }
    }
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
    VRPTWTabuSearch();
    outputSolution(best_solution, file);

    auto globalEnd = chrono::steady_clock::now();
    cout << "Runtime: " << chrono::duration_cast<chrono::seconds>(globalEnd - globalStart).count() << " seconds\n";
    return 0;
}