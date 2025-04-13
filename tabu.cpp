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
double best_distance = 1e9;
int vehicleCount, vehicleCapacity, numCustomers;
int max_evaluations, max_time;
mt19937 rng(time(nullptr));
chrono::time_point<chrono::steady_clock> globalStart;
vector<TabuMove> tabu_list;

// محاسبه فاصله اقلیدسی
double euclidean(const Customer &a, const Customer &b) {
    return hypot(a.x - b.x, a.y - b.y);
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
    for (size_t i = 0; i + 1 < route.size(); ++i)
        cost += dist[route[i]][route[i + 1]];
    return cost;
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

vector<vector<int>> generateInitialSolution() {
    vector<vector<int>> routes;
    vector<bool> visited(numCustomers, false);
    visited[0] = true;
    for (int i = 1; i < numCustomers; ++i) {
        if (visited[i]) continue;
        vector<int> route = {0, i, 0};
        int load = 0;
        if (validRoute(route, load)) {
            routes.push_back(route);
            visited[i] = true;
        }
    }
    return routes;
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

vector<vector<int>> get_neighbors(const vector<vector<int>>& solution) {
    vector<vector<int>> neighbors;
    for (const auto& route : solution) {
        if (route.size() <= 3) continue;
        for (size_t i = 1; i + 2 < route.size(); ++i) {
            for (size_t j = i + 1; j + 1 < route.size(); ++j) {
                auto newRoute = route;
                swap(newRoute[i], newRoute[j]);
                int load = 0;
                if (validRoute(newRoute, load))
                    neighbors.push_back(newRoute);
            }
        }
    }
    return neighbors;
}

void VRPTWTabuSearch() {
    auto sol = generateInitialSolution();
    best_solution = sol;
    best_distance = 1e9;

    int evaluations = 0;
    globalStart = chrono::steady_clock::now();

    while ((max_evaluations == 0 || evaluations < max_evaluations) &&
           chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - globalStart).count() < max_time) {

        auto neighbors = get_neighbors(sol);
        double best_neighbor_cost = 1e9;
        vector<int> best_route;
        int a_move = -1, b_move = -1;

        for (const auto& route : neighbors) {
            double cost = routeCost(route);
            if (cost < best_neighbor_cost) {
                best_neighbor_cost = cost;
                best_route = route;
                if (route.size() > 2) a_move = route[1], b_move = route[2];
            }
        }

        if (!best_route.empty() && !is_tabu(a_move, b_move)) {
            sol = {best_route};
            if (best_neighbor_cost < best_distance && isFeasibleSolution(sol)) {
                best_solution = sol;
                best_distance = best_neighbor_cost;
            }
            add_tabu(a_move, b_move);
        }

        decrement_tabu();
        evaluations++;
    }
}

void outputSolution(const vector<vector<int>>& solution, const string& filename) {
    double total = 0.0;
    for (const auto &route : solution) total += routeCost(route);
    int vehiclesUsed = solution.size();

    cout << "Vehicles used: " << vehiclesUsed << "\n";
    cout << "Total cost: " << fixed << setprecision(2) << total << "\n";
    for (size_t i = 0; i < solution.size(); ++i) {
        cout << "Route " << i + 1 << ": ";
        for (size_t j = 1; j + 1 < solution[i].size(); ++j)
            cout << solution[i][j] << " ";
        cout << "| Cost: " << fixed << setprecision(2) << routeCost(solution[i]) << "\n";
    }

    string outFile = filename.substr(0, filename.find_last_of('.')) + "_output.txt";
    ofstream fout(outFile);
    fout << "Vehicles used: " << vehiclesUsed << "\n";
    fout << "Total cost: " << fixed << setprecision(2) << total << "\n";
    for (size_t i = 0; i < solution.size(); ++i) {
        fout << "Route " << i + 1 << ": ";
        for (size_t j = 1; j + 1 < solution[i].size(); ++j)
            fout << solution[i][j] << " ";
        fout << "| Cost: " << fixed << setprecision(2) << routeCost(solution[i]) << "\n";
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