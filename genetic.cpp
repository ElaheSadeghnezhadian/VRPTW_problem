#define _USE_MATH_DEFINES
#include <bits/stdc++.h>
#include <cmath>
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
mt19937 rng(time(nullptr));

// Genetic parameters
int POP_SIZE = 60;
double CROSSOVER_RATE = 0.9;
double MUTATION_RATE = 0.4;
bool useRoulette = true;

// Logging streams
ofstream logPopulation, logGeneration, logEvents;

// Compute Euclidean distance
inline double euclidean(const Customer &a, const Customer &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// Build distance matrix
void buildDistanceMatrix() {
    int n = customers.size();
    dist.assign(n, vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist[i][j] = euclidean(customers[i], customers[j]);
}

// Read VRPTW instance
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

// Check route feasibility
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

// Cost of a route
double routeCost(const vector<int>& route) {
    double c=0;
    for (int i=1; i<route.size(); ++i) c += dist[route[i-1]][route[i]];
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
                    penalty += (load - vehicleCapacity) * 10; // Capacity violation
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
        if (!visited[i]) penalty += 500; // Customer not visited

    return penalty;
}

// Objective: minimize vehicles then distance
// double objective(const Solution &sol) {
//     evaluationCounter++;
//     int used=0; double d=0;
//     for (auto &r:sol) if (r.size()>2) { used++; d+=routeCost(r);}    
//     return used*10000 + d;
// }

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
    return used * 10000 + distance + penalty * 100; // وزن penalty قابل تنظیم است
}

// Ensure solution visits all customers exactly once
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

// Log a solution to population log
void logSolution(const Solution &sol, int solId) {
    logPopulation << "Solution " << solId << ": Objective=" << objective(sol) << " Vehicles=";
    int v=0; for(auto &r:sol) if(r.size()>2) v++;
    logPopulation << v << "\n";
    for (int i=0; i<sol.size(); ++i) {
        logPopulation << " Route " << i << ":";
        for (int j=0; j<sol[i].size(); ++j) logPopulation << " " << sol[i][j];
        logPopulation << "\n";
    }
}

// Create a random solution: simple greedy insert
Solution randomSolution() {
    static int counter = 0;
    const double P_RANDOM = 0.6;
    vector<bool> used(numCustomers, false);
    used[0] = true;

    Solution sol;
    ofstream logEvents("logEvents.txt", ios::app);
    logEvents << "=== Generating Solution #" << counter << " ===\n";

    // محاسبه ترتیب مشتری‌ها بر اساس زاویه قطبی از دپو
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

    vector<int> polarOrder;
    for (auto& [angle, idx] : polarSorted)
        polarOrder.push_back(idx);

    for (int v = 0; v < vehicleCount; ++v) {
        bool allUsed = true;
        for (int i = 1; i < numCustomers; ++i)
            if (!used[i]) { allUsed = false; break; }
        if (allUsed) break;

        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            vector<int> candidates;
            for (int i : polarOrder) {
                if (used[i]) continue;
                double arrival = time + dist[current][i];
                arrival = max(arrival, (double)customers[i].readyTime);
                double finish = arrival + customers[i].serviceTime;
                if (finish <= customers[i].dueTime && load + customers[i].demand <= vehicleCapacity)
                    candidates.push_back(i);
            }

            if (candidates.empty()) break;

            int next;
            if (uniform_real_distribution<>(0,1)(rng) < P_RANDOM) {
                int idx = uniform_int_distribution<int>(0, candidates.size() - 1)(rng);
                next = candidates[idx];
                logEvents << "  [RANDOM] picks " << next << "\n";
            } else {
                // به‌جای کمترین فاصله، اولین مورد در polarOrder (که هنوز قابل استفاده است)
                next = candidates[0];
                logEvents << "  [POLAR-GREEDY] picks " << next << "\n";
            }

            route.push_back(next);
            used[next] = true;
            time += dist[current][next];
            time = max(time, (double)customers[next].readyTime) + customers[next].serviceTime;
            load += customers[next].demand;
            current = next;
        }

        if (route.size() > 1) {
            route.push_back(0);
            sol.push_back(route);
        }
    }

    for (int i = 1; i < numCustomers; ++i) {
        if (!used[i]) {
            logEvents << "  Unassigned customer " << i << " -> single route\n";
            sol.push_back({0, i, 0});
        }
    }

    logSolution(sol, counter++);
    return sol;
}

Solution solomonInitialSolution() {
    vector<bool> inserted(numCustomers, false);
    inserted[0] = true;

    Solution sol;

    for (int v = 0; v < vehicleCount; ++v) {
        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            int best = -1;
            double minCost = 1e9;

            for (int i = 1; i < numCustomers; ++i) {
                if (inserted[i]) continue;

                double arrival = time + dist[current][i];
                arrival = max(arrival, (double)customers[i].readyTime);
                double finish = arrival + customers[i].serviceTime;

                if (finish > customers[i].dueTime || load + customers[i].demand > vehicleCapacity)
                    continue;

                double cost = arrival; // می‌تونی معیار بهتر تعریف کنی
                if (cost < minCost) {
                    minCost = cost;
                    best = i;
                }
            }

            if (best == -1) break;

            route.push_back(best);
            inserted[best] = true;
            time = max(time + dist[current][best], (double)customers[best].readyTime) + customers[best].serviceTime;
            load += customers[best].demand;
            current = best;
        }

        if (route.size() > 1) {
            route.push_back(0);
            sol.push_back(route);
        }
    }

    for (int i = 1; i < numCustomers; ++i) {
        if (!inserted[i]) {
            sol.push_back({0, i, 0});
        }
    }

    return sol;
}

// Population initialization
vector<Solution> initPopulation() {
    logEvents << "Initializing population...\n";
    const double P_RANDOM = 0.6;
    vector<Solution> pop;
    for (int i = 0; i < POP_SIZE; ++i) {
        if (uniform_real_distribution<>(0,1)(rng) < P_RANDOM) {
            pop.push_back(solomonInitialSolution());
        }
        else
            pop.push_back(randomSolution());
    }
    // s = applyLocalSearch(s);
    // pop.push_back(s);
    logEvents << "Initial population of " << POP_SIZE << " solutions generated.\n";
    return pop;
}

void repairSolution(Solution& sol) {
    vector<int> visitCount(numCustomers, 0);

    for (auto& route : sol) {
        int load = 0;
        double time = 0;
        vector<int> newRoute = {0};

        for (int i = 1; i < route.size() - 1; ++i) {
            int curr = newRoute.back();
            int next = route[i];
            double arrival = time + dist[curr][next];

            arrival = max(arrival, (double)customers[next].readyTime);
            double leave = arrival + customers[next].serviceTime;
            int newLoad = load + customers[next].demand;

            if (leave > customers[next].dueTime || newLoad > vehicleCapacity) {
                sol.push_back({0, next, 0}); // New route
            } else {
                newRoute.push_back(next);
                load = newLoad;
                time = leave;
            }

            visitCount[next]++;
        }

        newRoute.push_back(0);
        route = newRoute;
    }

    // حذف مشتری‌های تکراری از مسیرها (فقط یکی باقی بماند)
    for (int i = 1; i < numCustomers; ++i) {
        if (visitCount[i] > 1) {
            int count = 0;
            for (auto& route : sol) {
                for (int j = 1; j < route.size() - 1; ++j) {
                    if (route[j] == i) {
                        count++;
                        if (count > 1) {
                            route.erase(route.begin() + j);
                            break;
                        }
                    }
                }
            }
        }
    }
}

// Tournament selection
Solution tournament(const vector<Solution> &pop) {
    int k=3;
    Solution best;
    double bestFit = numeric_limits<double>::infinity();
    uniform_int_distribution<int> distPop(0, pop.size()-1);
    for (int i=0;i<k;i++) {
        int idx = distPop(rng);
        double f = objective(pop[idx]);
        if (f<bestFit) { bestFit=f; best=pop[idx]; }
    }
    logEvents << "Tournament selected solution with objective "<<bestFit<<"\n";
    return best;
}

Solution rouletteSelect(const vector<Solution>& pop, const vector<double>& fit) {
    double sum = accumulate(fit.begin(), fit.end(), 0.0);
    double r = uniform_real_distribution<>(0,sum)(rng);
    double acc=0;
    for(int i=0;i<pop.size();i++){
        acc += (sum - fit[i]); // چون کمترین fitness بهتر است
        if(acc >= r) return pop[i];
    }
    return pop.back();
}

Solution eliteSelect(const vector<Solution>& pop, const vector<double>& fit) {
    // برترین k را انتخاب کن
    int k = 3;
    vector<int> idx(pop.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&](int a,int b){return fit[a]<fit[b];});
    int pick = idx[ uniform_int_distribution<>(0, k-1)(rng) ];
    return pop[pick];
}

// Crossover: route exchange
pair<Solution, Solution> crossover(const Solution &a, const Solution &b) {
    int r1 = uniform_int_distribution<int>(0, a.size() - 1)(rng);
    int r2 = uniform_int_distribution<int>(0, b.size() - 1)(rng);

    Solution child1, child2;
    set<int> used1 = {0}, used2 = {0};

    // Copy selected routes
    child1.push_back(a[r1]);
    for (int i : a[r1]) used1.insert(i);

    child2.push_back(b[r2]);
    for (int i : b[r2]) used2.insert(i);

    // Add remaining routes from other parent
    auto fillChild = [&](const Solution &parent, Solution &child, set<int> &used) {
        for (auto &r : parent) {
            vector<int> cleanRoute;
            for (int c : r) {
                if (used.count(c) == 0) {
                    cleanRoute.push_back(c);
                    used.insert(c);
                }
            }
            if (!cleanRoute.empty()) {
                cleanRoute.insert(cleanRoute.begin(), 0);
                cleanRoute.push_back(0);
                child.push_back(cleanRoute);
            }
        }
    };

    fillChild(b, child1, used1);
    fillChild(a, child2, used2);

    return {child1, child2};
}

// Mutation: swap two customers
void mutate(Solution &sol) {
    // 1. درون-مسیر (swap)
    for (auto &route : sol) {
        if (route.size() > 3 && uniform_real_distribution<>(0, 1)(rng) < 0.3) {
            int i = uniform_int_distribution<int>(1, route.size() - 2)(rng);
            int j = uniform_int_distribution<int>(1, route.size() - 2)(rng);
            swap(route[i], route[j]);
            if (!validRoute(route)) swap(route[i], route[j]); // undo
        }
    }

    // 2. بین-مسیر (move)
    if (sol.size() > 1 && uniform_real_distribution<>(0, 1)(rng) < 0.3) {
        int from = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        int to = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
        if (from == to) return;

        auto &r1 = sol[from], &r2 = sol[to];
        if (r1.size() <= 3) return; // nothing to move

        int idx = uniform_int_distribution<int>(1, r1.size() - 2)(rng);
        int customer = r1[idx];
        r1.erase(r1.begin() + idx);

        // insert to random pos
        int insertPos = uniform_int_distribution<int>(1, r2.size() - 1)(rng);
        r2.insert(r2.begin() + insertPos, customer);

        if (!validRoute(r1) || !validRoute(r2)) {
            r2.erase(r2.begin() + insertPos);
            r1.insert(r1.begin() + idx, customer); // undo
        }
    }
}

double routeDistance(const vector<int>& route) {
    double total = 0.0;
    for (int i = 0; i < (int)route.size() - 1; ++i)
        total += dist[route[i]][route[i + 1]];
    return total;
}

vector<int> twoOptSwap(const vector<int>& route) {
    int n = route.size();
    if (n <= 4) return route;  // خیلی کوتاهه، تغییر نمی‌خواد

    vector<int> bestRoute = route;
    double bestDist = routeDistance(route);

    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 1; i < n - 2; ++i) {
            for (int j = i + 1; j < n - 1; ++j) {
                vector<int> newRoute = bestRoute;
                reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
                double newDist = routeDistance(newRoute);
                if (newDist < bestDist) {
                    bestRoute = newRoute;
                    bestDist = newDist;
                    improved = true;
                }
            }
        }
    }

    return bestRoute;
}

Solution applyLocalSearch(const Solution& sol) {
    Solution newSol;
    for (const auto& route : sol) {
        newSol.push_back(twoOptSwap(route));
    }
    return newSol;
}

// Genetic Algorithm main
Solution geneticAlgorithm(int max_time, int max_evaluations) {
    logGeneration << "Gen, BestObjective\n";
    auto population = initPopulation();
    Solution best = population[0];
    double bestFit = objective(best);
    // logGeneration << 0 << "," << bestFit << "\n";

    auto t_start = chrono::steady_clock::now();
    int gen = 1;
    // ب) ارزیابی اولیه
    vector<double> fitness(POP_SIZE);
    for(int i=0;i<POP_SIZE;i++){
        fitness[i] = objective(population[i]);
    }

    // ج) حلقه نسل‌ها
    while (true) {
        // 1) چک زمان
        // cout<<"[GA] Loop start gen="<<gen<<"\n"; cout.flush();
        auto t_now = chrono::steady_clock::now();
        double elapsed_sec = chrono::duration<double>(t_now - t_start).count();
        if (max_time > 0 && elapsed_sec >= max_time) {
            cout << "Stopping: time limit reached after " << elapsed_sec << "s\n";
            break;
        }
        // 2) چک تعداد ارزیابی
        if (max_evaluations > 0 && evaluationCounter >= max_evaluations) {
            cout << "Stopping: evaluation limit reached (" 
                 << evaluationCounter << ")\n";
            break;
        }

        // ۱) انتخاب والدین (Roulette یا Elite)
        vector<Solution> matingPool;
        for(int i=0;i<POP_SIZE;i++){
            if (useRoulette)
                matingPool.push_back(rouletteSelect(population, fitness));
            else
                matingPool.push_back(eliteSelect(population, fitness));
        }

        // ۲) تولید فرزندان با crossover+mutate
        vector<Solution> offspring;
        for(int i=0;i<POP_SIZE/2;i++){
            auto p1 = matingPool[2*i], p2 = matingPool[2*i+1];
            Solution c1, c2;
            if (uniform_real_distribution<>(0,1)(rng) < CROSSOVER_RATE)
                tie(c1,c2) = crossover(p1,p2);
            else
                c1=p1, c2=p2;
            mutate(c1); mutate(c2);

            c1 = applyLocalSearch(c1);
            c2 = applyLocalSearch(c2);

            offspring.push_back(c1);
            offspring.push_back(c2);
        }

        // ۳) ارزیابی فرزندان و ترکیب (μ+λ) برای نسل بعد
        vector<pair<double,Solution>> combined;
        combined.reserve(POP_SIZE + offspring.size());
        for(int i=0;i<POP_SIZE;i++)
            combined.emplace_back(fitness[i], population[i]);
        for(auto &c : offspring)
            combined.emplace_back(objective(c), c);

        sort(combined.begin(), combined.end(),
             [](auto &a, auto &b){ return a.first < b.first; });

        population.clear();
        fitness.clear();
        for(int i=0;i<POP_SIZE;i++){
            population.push_back(combined[i].second);
            fitness.push_back(combined[i].first);
        }
        // پس از مرتب‌سازی `combined`
        int elitismCount = 1;  // تعداد نخبه‌هایی که مستقیماً حفظ می‌شوند
        for (int i = 0; i < elitismCount; ++i) {
            population.push_back(combined[i].second);
            fitness.push_back(combined[i].first);
        }
        for (int i = elitismCount; i < POP_SIZE; ++i) {
            population.push_back(combined[i].second);
            fitness.push_back(combined[i].first);
        }


        if (fitness[0] < bestFit) {
        bestFit = fitness[0];
        best = population[0];
        }


        // لاگ
        cout << "Gen " << gen << ", best=" << fitness[0] 
     << ", evaluations=" << evaluationCounter << "\n";
    logGeneration << gen << "," << bestFit << endl;
        if (gen % 50 == 0) cout << "Gen "<<gen<<" best="<<bestFit<<"\n";
    }

    return population[0];
}

// Output solution
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

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] 
             << " [instance-file] [max-time-sec] [max-evaluations]\n";
        return 1;
    }
    // open logs
    logPopulation.open("population_log.txt");
    logGeneration.open("generation_log.csv");
    logEvents.open("events_log.txt");

    string file          = argv[1];
    int max_time = atoi(argv[2]);    
    int max_evaluations = atoi(argv[3]);

    readInstance(file);

    auto t0 = chrono::steady_clock::now();

    auto best = geneticAlgorithm(max_time,max_evaluations);

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
    logGeneration.close();
    logEvents.close();

    return 0;
}
