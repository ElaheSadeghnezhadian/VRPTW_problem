#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <time.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <numeric> 
#include <random>
#include <limits>   
#include <unordered_set>

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

vector<vector<int>> best_solution;
vector<vector<int>> bestFeasibleSolution;
double best_distance;
int vehicleCount, vehicleCapacity, numCustomers;
int max_evaluations, max_time;
string file;
chrono::time_point<chrono::steady_clock> globalStart; 
int evaluations = 0;

int interation = 500;
int customerNr;
vector<vector<int>> genes;
vector<double> fitness;

int POP_SIZE = 50;
double CROSSOVER_RATE = 0.8;
double MUTATION_RATE = 0.2;
int MAX_GEN = 500;

vector<Customer> customers;
vector<vector<double>> dist;
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

using Solution = vector<vector<int>>;

const int   population_size  = 50;
const int   elite_size       = 5;
const double crossover_rate  = 0.8;
const double mutation_rate   = 0.2;

double rand01() {
    static uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng);
}

vector<Solution> selectElites(const vector<Solution>& population) {
    vector<Solution> elites;
    for (int i = 0; i < elite_size && i < (int)population.size(); ++i)
        elites.push_back(population[i]);
    return elites;
}

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
    visited[0] = true;
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

Solution generateInitialSolution(bool useGreedy = true) {
    Solution solution;
    vector<bool> visited(numCustomers, false);
    visited[0] = true;            // depot

    if (useGreedy) {
        vector<pair<double,int>> polar;
        for(int i=1;i<numCustomers;++i){
            double dx = customers[i].x - customers[0].x;
            double dy = customers[i].y - customers[0].y;
            polar.emplace_back(atan2(dy,dx), i);
        }
        sort(polar.begin(), polar.end());

        for(auto &pr : polar) {
            int cust = pr.second;
            bool inserted = false;
            for(auto &route : solution){
                // امتحان درج بین دو صفر انتهایی
                route.insert(route.end()-1, cust);
                int load=0;
                if(validRoute(route, load)){
                    inserted = true;
                    break;
                }
                route.erase(route.end()-1);
            }
            if(!inserted){
                solution.push_back({0, cust, 0});
            }
        }
    } else {
        // به‌ شکل تصادفی (یا round‑robin) مثل قبل …
    }

    return solution;
}

vector<Solution> initPopulation() {
    vector<Solution> pop;
    int attempts = 0;
    while (pop.size() < population_size) {
        Solution s = generateInitialSolution(true);
        if (isFeasibleSolution(s)) {
            pop.push_back(s);
        } else if (++attempts > population_size * 10) {
            Solution fallback;
            for(int i=1;i<numCustomers;++i)
                fallback.push_back({0,i,0});
            if(isFeasibleSolution(fallback))
                pop.push_back(fallback);
            attempts = 0;
        }
    }
    return pop;
}

Solution tournamentSelection(const vector<Solution>& population, int k = 3) {
    Solution best;
    double best_val = std::numeric_limits<double>::infinity();
    for (int i = 0; i < k; ++i) {
        int idx = rand() % population.size();
        double cost = totalCost(population[idx]);
        if (cost < best_val) {
            best_val = cost;
            best = population[idx];
        }
    }
    return best;
}

Solution crossover(const Solution& parent1, const Solution& parent2) {
    Solution child(customers.size());
    unordered_set<int> used;
    for (int i = 0; i < parent1.size(); ++i) {
        if (i % 2 == 0) {
            for (int c : parent1[i]) {
                if (used.count(c) == 0) {
                    child[i].push_back(c);
                    used.insert(c);
                }
            }
        } else {
            for (int c : parent2[i]) {
                if (used.count(c) == 0) {
                    child[i].push_back(c);
                    used.insert(c);
                }
            }
        }
    }
    for (int c = 1; c < customers.size(); ++c) {
        if (used.count(c) == 0) {
            int i = rand() % child.size();
            child[i].push_back(c);
        }
    }

    return child;
}

void mutate(Solution& sol) {
    int a = rand() % sol.size();
    int b = rand() % sol.size();
    if (sol[a].empty() || sol[b].empty()) return;

    int i = rand() % sol[a].size();
    int j = rand() % sol[b].size();

    swap(sol[a][i], sol[b][j]);
}

void VRPTW_GA(int max_time, int max_evaluations) {
    bool unlimitedEvals = (max_evaluations == 0);
    bool unlimitedTime  = (max_time       == 0);

    globalStart = chrono::steady_clock::now();
    vector<Solution> population = initPopulation();  
    best_solution         = population.front();
    best_distance         = totalCost(best_solution);
    bestFeasibleSolution  = best_solution;
    best_distance = std::numeric_limits<double>::infinity();
    bestFeasibleSolution.clear();    

    while ((unlimitedEvals  || evaluations < max_evaluations) &&
           (unlimitedTime   || chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - globalStart).count() < max_time))
    {
        sort(population.begin(), population.end(),
             [&](auto &A, auto &B){ return totalCost(A) < totalCost(B); });

        double c0 = totalCost(population[0]);
        if (c0 < best_distance) {
            best_distance = c0;
            best_solution = population[0];
        }

        vector<Solution> new_pop = selectElites(population);
        while (new_pop.size() < population_size) {
            auto p1 = tournamentSelection(population);
            auto p2 = tournamentSelection(population);
            Solution child = (rand01() < crossover_rate ? crossover(p1,p2) : p1);
            if (rand01() < mutation_rate) mutate(child);

            if (isFeasibleSolution(child)) {
                double cc = totalCost(child);
                if (cc < best_distance) {
                    best_distance = cc;
                    bestFeasibleSolution = child;
                }
                new_pop.push_back(child);
            }
        }
        population.swap(new_pop);
    }

    outputSolution(bestFeasibleSolution, file);
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " [instance-file] [max-time-sec] [max-evaluations]\n";
        return 1;
    }

    file = argv[1];
    max_time = atoi(argv[2]);   
    max_evaluations = atoi(argv[3]);

    readInstance(file);

    auto t0 = chrono::steady_clock::now();
    
    VRPTW_GA(max_time, max_evaluations);

    auto t1 = chrono::steady_clock::now();

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
