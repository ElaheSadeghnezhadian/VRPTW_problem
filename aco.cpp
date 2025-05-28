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

// === logging streams ===
ofstream logConstruction;
ofstream logPheromone;
ofstream logSummary;

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

    if (logConstruction.is_open()) {
    logConstruction << "[Ant #" << evaluationCounter << "]\n";
    double total = 0;
    for (size_t i = 0; i < sol.size(); ++i) {
        logConstruction << "Route " << i + 1 << ": ";
        for (size_t j = 0; j < sol[i].size(); ++j)
            logConstruction << sol[i][j] << (j+1 < sol[i].size() ? " → " : "");
        double rc = routeCost(sol[i]);
        logConstruction << " | Cost: " << fixed << setprecision(2) << rc << '\n';
        total += rc;
    }
    logConstruction << "Total Routes: " << sol.size() << "\n";
    logConstruction << "Total Distance: " << fixed << setprecision(2) << total << "\n";
    logConstruction << "----------------------------\n";
}

    return sol;
}

// === updatePheromone ===
void updatePheromone(const vector<Solution>& ants,const Solution& best,int iteration,
                     double bestObjective)
{
    /*── تبخیر ────────────────────────────────*/
    for (auto& row : pheromone)
        for (double& p : row)
            p *= (1.0 - EVAPORATION);

    /*── تقویت توسط همهٔ مورچه‌ها ─────────────*/
    for (const auto& sol : ants) {
        double q = Q / (totalCost(sol) + penaltyTerm(sol));
        for (const auto& r : sol)
            for (size_t i = 1; i < r.size(); ++i) {
                int u = r[i - 1], v = r[i];
                pheromone[u][v] += q;
                pheromone[v][u] += q;
            }
    }

    /*── تقویت اضافه توسط بهترین ─────────────*/
    double bestQ = Q * 2.0 / (totalCost(best) + penaltyTerm(best));
    for (const auto& r : best)
        for (size_t i = 1; i < r.size(); ++i) {
            int u = r[i - 1], v = r[i];
            pheromone[u][v] += bestQ;
            pheromone[v][u] += bestQ;
        }

    /*── لاگ فرومون ───────────────────────────*/
    if (logPheromone.is_open()) {
        logPheromone << "[Iteration " << iteration << "]\n";
        logPheromone << "Best Objective: "
                     << fixed << setprecision(2) << bestObjective << '\n';
        logPheromone << "Reinforced edges:\n";
        for (int u = 0; u < numCustomers; ++u)
            for (int v = u + 1; v < numCustomers; ++v)
                if (pheromone[u][v] > 1.0 + 1e-6)
                    logPheromone << "   (" << u << "," << v << "): "
                                 << fixed << setprecision(3)
                                 << pheromone[u][v] << '\n';
        logPheromone << "----------------------------\n";
        logPheromone.flush();
    }
}

// === antColonyOptimization ===
Solution antColonyOptimization(int maxTime, int maxEvaluations)
{
    initializePheromone();

    Solution globalBest;
    double   bestObj = numeric_limits<double>::max();
    int      evals   = 0;
    int      it      = 0;
    auto     start   = chrono::steady_clock::now();

    while (true) {
        auto now = chrono::steady_clock::now();
        if ((maxTime > 0 &&
             chrono::duration_cast<chrono::seconds>(now - start).count() >= maxTime) ||
            (maxEvaluations > 0 && evals >= maxEvaluations))
            break;

        /*-- تولید مورچه‌ها --*/
        vector<Solution> ants;
        ants.reserve(POP_SIZE);

        for (int i = 0; i < POP_SIZE &&
             (maxEvaluations == 0 || evals < maxEvaluations); ++i)
        {
            Solution antSol = constructAntSolution();
            ants.push_back(antSol);

            double obj = objective(antSol);
            ++evals;
            if (obj < bestObj) {
                bestObj    = obj;
                globalBest = antSol;
            }
        }

        /*-- لاگ خلاصه --*/
        if (logSummary.is_open()) {
            double t = chrono::duration_cast<chrono::seconds>(
                           chrono::steady_clock::now() - start).count();
            logSummary << "iter=" << it
                       << " evals=" << evals
                       << " bestObj=" << fixed << setprecision(2) << bestObj
                       << " time=" << t << "s\n";
            logSummary.flush();
        }

        /*-- به‌روزرسانی فرومون (با پارامترهای جدید) --*/
        updatePheromone(ants, globalBest, it, bestObj);   // ★

        ++it;
    }
    return globalBest;
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

    logConstruction.open("construction.log");
    logPheromone.open("pheromone.log");
    logSummary.open("summary.log");


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
    logConstruction.close();
    logPheromone.close();
    logSummary.close();

    return 0;
}