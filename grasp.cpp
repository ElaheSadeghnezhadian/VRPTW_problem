#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <numeric> 
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
double alpha = 0.1;   
int evaluations = 0;


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
    evaluations++;
    double cost = 0.0;
    for (const auto& r : sol)
        cost += routeCost(r);
    return cost;
}

double combinedObjective(const vector<vector<int>>& sol) {
    double totalDist = 0.0;
    int vehicleUsed = 0;
    for (auto& route : sol) {
        if (route.size() > 2) {
            totalDist += routeCost(route);
            vehicleUsed++;
        }
    }
    return 10000.0 * vehicleUsed + totalDist;
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

vector<vector<vector<int>>> get_neighbors(const vector<vector<int>>& solution) {
    const size_t MAX_NEIGHBORS = 100;
    vector<vector<vector<int>>> neighbors;
    neighbors.reserve(MAX_NEIGHBORS);

    int R = (int)solution.size();

    vector<vector<int>> newSol = solution;

    // 1) Relocate within same route
    for (int r = 0; r < R && neighbors.size() < MAX_NEIGHBORS; ++r) {
        auto& route = newSol[r];
        int sz = (int)route.size();
        if (sz <= 3) continue;
        for (int i = 1; i < sz-2 && neighbors.size() < MAX_NEIGHBORS; ++i) {
            int cust = route[i];
            route.erase(route.begin()+i);
            for (int j = 1; j < sz-1 && neighbors.size() < MAX_NEIGHBORS; ++j) {
                if (j == i) continue;
                route.insert(route.begin()+j, cust);
                int load=0;
                if (validRoute(route, load)) {
                    neighbors.push_back(newSol);
                }
                route.erase(route.begin()+j);
            }
            route.insert(route.begin()+i, cust);
        }
    }

    // 2) Relocate between routes
    for (int r1 = 0; r1 < R && neighbors.size() < MAX_NEIGHBORS; ++r1) {
        auto& route1 = newSol[r1];
        int sz1 = (int)route1.size();
        if (sz1 <= 3) continue;
        for (int i = 1; i < sz1-1 && neighbors.size() < MAX_NEIGHBORS; ++i) {
            int cust = route1[i];
            route1.erase(route1.begin()+i);
            for (int r2 = 0; r2 < R && neighbors.size() < MAX_NEIGHBORS; ++r2) {
                if (r2==r1) continue;
                auto& route2 = newSol[r2];
                int sz2 = (int)route2.size();
                for (int j = 1; j < sz2 && neighbors.size() < MAX_NEIGHBORS; ++j) {
                    route2.insert(route2.begin()+j, cust);
                    int l1=0,l2=0;
                    if (validRoute(route1,l1) && validRoute(route2,l2)) {
                        neighbors.push_back(newSol);
                    }
                    route2.erase(route2.begin()+j);
                }
            }
            route1.insert(route1.begin()+i, cust);
        }
    }

    // 3) Swap within same route
    for (int r = 0; r < R && neighbors.size() < MAX_NEIGHBORS; ++r) {
        auto& route = newSol[r];
        int sz = (int)route.size();
        if (sz <= 3) continue;
        for (int i = 1; i < sz-2 && neighbors.size() < MAX_NEIGHBORS; ++i) {
            for (int j = i+1; j < sz-1 && neighbors.size() < MAX_NEIGHBORS; ++j) {
                swap(route[i], route[j]);
                int l = 0;
                if (validRoute(route, l)) {
                    neighbors.push_back(newSol);
                }
                swap(route[i], route[j]);
            }
        }
    }

    // 4) Swap between routes
    for (int r1 = 0; r1 < R && neighbors.size() < MAX_NEIGHBORS; ++r1) {
        for (int r2 = r1+1; r2 < R && neighbors.size() < MAX_NEIGHBORS; ++r2) {
            auto& route1 = newSol[r1];
            auto& route2 = newSol[r2];
            int sz1 = (int)route1.size(), sz2 = (int)route2.size();
            if (sz1<=3 || sz2<=3) continue;
            for (int i = 1; i < sz1-1 && neighbors.size() < MAX_NEIGHBORS; ++i) {
                for (int j = 1; j < sz2-1 && neighbors.size() < MAX_NEIGHBORS; ++j) {
                    swap(route1[i], route2[j]);
                    int l1=0,l2=0;
                    if (validRoute(route1,l1) && validRoute(route2,l2)) {
                        neighbors.push_back(newSol);
                    }
                    swap(route1[i], route2[j]);
                }
            }
        }
    }

    // 5) 2‑opt within same route
    for (int r = 0; r < R && neighbors.size() < MAX_NEIGHBORS; ++r) {
        auto& route = newSol[r];
        int sz = (int)route.size();
        if (sz <= 4) continue;
        for (int i = 1; i < sz-2 && neighbors.size() < MAX_NEIGHBORS; ++i) {
            for (int j = i+1; j < sz-1 && neighbors.size() < MAX_NEIGHBORS; ++j) {
                reverse(route.begin()+i, route.begin()+j+1);
                int l = 0;
                if (validRoute(route, l)) {
                    neighbors.push_back(newSol);
                }
                reverse(route.begin()+i, route.begin()+j+1);
            }
        }
    }

    return neighbors;
}

bool isFeasibleSolution(const vector<vector<int>>& sol) {
    evaluations++;
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
            if (sol[i].size() <= 2) continue;
        
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
            if (sol[i].size() <= 2) continue; // skip empty routes
        
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

vector<vector<int>> buildGRASPSolution() {
    ofstream log("grasp_build_log.txt", ios::out);
    vector<vector<int>> solution;
    vector<bool> inSol(numCustomers, false);
    inSol[0] = true; 

    vector<int> remaining;
    for (int i = 1; i < numCustomers; ++i)
        remaining.push_back(i);

    while (!remaining.empty()) {
        struct Candidate { int cust, r, pos; double delta; };
        vector<Candidate> cands;

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
            vector<int> nr = {0, cust, 0};
            int load = 0;
            if (validRoute(nr, load)) {
                cands.push_back({cust, -1, -1, routeCost(nr)});
            }
        }
        if (cands.empty()) break;

        sort(cands.begin(), cands.end(),
             [](auto &a, auto &b){ return a.delta < b.delta; });
        int rcl_sz = max(1, (int)ceil(alpha * cands.size()));
        uniform_int_distribution<int> pick(0, rcl_sz-1);
        auto sel = cands[pick(rng)];

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
        log << "Inserted customer " << sel.cust;
        if (sel.r >= 0)
            log << " into route " << sel.r << " at position " << sel.pos;
        else
            log << " in a new route";
        log << ", cost delta: " << sel.delta << "\n";
    }
    log.close();

    return solution;
}

void bestInsertion(vector<int>& route, int cust) {
    double bestCost = numeric_limits<double>::infinity();
    int bestPos = -1;
    for (int i = 1; i < (int)route.size(); ++i) {
        vector<int> temp = route;
        temp.insert(temp.begin() + i, cust);
        int load = 0;
        if (validRoute(temp, load)) {
            double cost = routeCost(temp);
            if (cost < bestCost) {
                bestCost = cost;
                bestPos = i;
            }
        }
    }
    if (bestPos >= 0)
        route.insert(route.begin() + bestPos, cust);
    else
        route.insert(route.end() - 1, cust);  // اگر جای خوب پیدا نشد، نزدیک دپو اضافه کن
}

void insertionRepair(vector<vector<int>>& routes) {
    vector<bool> visited(numCustomers, false);
    for (auto& r : routes)
        for (int j : r) visited[j] = true;
    visited[0] = true;

    vector<int> unassigned;
    for (int j = 1; j < numCustomers; ++j)
        if (!visited[j]) unassigned.push_back(j);

    while (!unassigned.empty()) {
        double bestCostInc = numeric_limits<double>::infinity();
        int bestCust = -1, bestRoute = -1, bestPos = -1;

        for (int cust : unassigned) {
            for (int k = 0; k < routes.size(); ++k) {
                auto& route = routes[k];
                double baseCost = routeCost(route);

                for (int pos = 1; pos < route.size(); ++pos) {
                    vector<int> temp = route;
                    temp.insert(temp.begin() + pos, cust);

                    int load = 0;
                    if (validRoute(temp, load)) {
                        double newCost = routeCost(temp);
                        double inc = newCost - baseCost;

                        if (inc < bestCostInc) {
                            bestCostInc = inc;
                            bestCust = cust;
                            bestRoute = k;
                            bestPos = pos;
                        }
                    }
                }
            }
        }

        if (bestCust == -1) {
            cerr << "!!! insertionRepair failed: No feasible insertion found for remaining customers.\n";
            break;
        }

        routes[bestRoute].insert(routes[bestRoute].begin() + bestPos, bestCust);
        visited[bestCust] = true;
        unassigned.erase(remove(unassigned.begin(), unassigned.end(), bestCust), unassigned.end());
    }
}

vector<vector<int>> constructInitialSolution_RP() {
    vector<vector<int>> solution;
    unordered_set<int> unvisited;
    for (int i = 1; i < numCustomers; ++i)
        unvisited.insert(i);

    for (int k = 0; k < vehicleCount && !unvisited.empty(); ++k) {
        vector<int> route = {0};    
        int curr = 0;
        double curr_time = 0.0;
        int load = 0;

        while (true) {
            vector<pair<int, double>> candidates;

            for (int j : unvisited) {
                const Customer& cust = customers[j];
                double travel_time = dist[curr][j];
                double arrival_time = max(curr_time + travel_time, (double)cust.readyTime);
                double leave_time = arrival_time + cust.serviceTime;

                double Edatij = arrival_time;
                double Ldatij = curr_time + travel_time;
                double LBTjs = dist[j][0];
                double UBTjs = cust.dueTime;

                bool feasible = 
                    Edatij <= cust.dueTime &&
                    Ldatij >= cust.readyTime &&
                    (leave_time + dist[j][0] <= customers[0].dueTime) &&
                    (load + cust.demand <= vehicleCapacity);

                if (feasible) {
                    candidates.emplace_back(j, LBTjs);
                }
            }

            if (candidates.empty()) break;

            sort(candidates.begin(), candidates.end(), [](auto& a, auto& b) {
                return a.second > b.second;
            });

            int Ncand = max(1, (int)ceil(alpha * candidates.size()));
            uniform_int_distribution<int> pick(0, Ncand - 1);
            int selIdx = pick(rng);
            int nextCust = candidates[selIdx].first;

            const Customer& c = customers[nextCust];
            double tt = dist[curr][nextCust];
            double arrival = max(curr_time + tt, (double)c.readyTime);
            curr_time = arrival + c.serviceTime;
            load += c.demand;
            curr = nextCust;

            route.push_back(curr);
            unvisited.erase(curr);
        }

        route.push_back(0);
        solution.push_back(route);
    }

    if (!unvisited.empty()) {
        insertionRepair(solution);
    }

    return solution;
}

vector<vector<int>> localSearch(const vector<vector<int>>& initSol, int max_time,
    const chrono::time_point<chrono::steady_clock>& start) {
auto cur = initSol;
double curCost = combinedObjective(cur);
bool improved = true;

ofstream log("local_search_log.txt", ios::out);
log << "=== Local Search Log ===\n";
log << "Initial cost: " << curCost << "\n";

while (improved) {
auto now = chrono::steady_clock::now();
if (max_time > 0 &&
chrono::duration_cast<chrono::seconds>(now - start).count() >= max_time) {
log << "Time limit reached. Exiting local search.\n";
break;
}

improved = false;

auto neighbors = get_neighbors(cur);
log << "Generated " << neighbors.size() << " neighbors\n";

for (auto& neighbor : neighbors) {
double neighborCost = combinedObjective(neighbor);
evaluations++;

if (neighborCost < curCost) {
log << "Improved: " << curCost << " → " << neighborCost << "\n";
cur = neighbor;
curCost = neighborCost;
improved = true;
break; 
}
}
}

log << "Final local cost: " << curCost << "\n";
log.close();
return cur;
}


void VRPTW_GRASP(int max_time, int max_evals) {
    double bestCost = numeric_limits<double>::infinity();
    vector<vector<int>> bestSol;
    int iterations  = 0;
    auto globalStart = chrono::steady_clock::now();

    {
        ofstream logFile("grasp_report.csv", ios::out);
        logFile << "Iteration,Evaluations,Time,Vehicles,BestCost\n";
    }

    while ((max_time <= 0 ||
    chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - globalStart).count() < max_time)
    && (max_evals <= 0 || evaluations < max_evals))
    {
    iterations++;
    auto sol   = constructInitialSolution_RP();
    insertionRepair(sol);
    auto local = localSearch(sol, max_time, globalStart);
    insertionRepair(local);

    double cost = combinedObjective(local);
    int vehicles = 0;
    for (auto &r : local)
    if (r.size() > 2) vehicles++;

    if (cost < bestCost && isFeasibleSolution(local)) {
    bestCost = cost;
    bestSol  = local;
    }

    double elapsed = chrono::duration_cast<chrono::seconds>(
    chrono::steady_clock::now() - globalStart).count();

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
    if (cost < bestCost && isFeasibleSolution(local)) {
        ofstream log("best_updates_log.txt", ios::app);
        log << "New best found at iteration " << iterations
            << " with cost " << cost 
            << " and " << vehicles << " vehicles.\n";
        log.close();
    }
    
    }

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
    int max_time     = atoi(argv[2]);   
    int max_evaluations  = atoi(argv[3]);  

    readInstance(file);

    auto t0 = chrono::steady_clock::now();

    int actualEvals;
    double actualTime;
    VRPTW_GRASP(max_time, max_evaluations);

    auto t1 = chrono::steady_clock::now();

    outputSolution(best_solution, file);


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
