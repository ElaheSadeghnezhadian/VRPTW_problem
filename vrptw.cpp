#include <iostream>       
#include <fstream>        
#include <vector>        
#include <cmath>          
#include <chrono>         
#include <sstream>       
#include <algorithm>      
#include <limits>         
#include <random>        
#include <iomanip>        

using namespace std;

struct Customer {
    int id;               // Customer ID
    double x, y;          // Coordinates
    int demand;           // Customer demand
    int readyTime;        // Time window start
    int dueTime;          // Time window end
    int serviceTime;      // Time needed to service the customer
};

class VRPTWSolver {
private:
    // Problem parameters and data
    int vehicleCount, vehicleCapacity;  
    int numCustomers;                   
    double temperature = 1000.0;        
    double coolingRate = 0.995;         
    double finalTemp = 1e-4;            
    string instanceFilename;           

    vector<Customer> customers;         
    vector<vector<double>> dist;        // Distance matrix

    vector<vector<int>> bestFeasibleSolution;
    double bestFeasibleObj = numeric_limits<double>::max();

    // Modern random number generator
    mt19937 rng;

public:
    // Constructor: initialize RNG using random_device
    VRPTWSolver() : rng(random_device{}()) {}

    // Read data from an instance file
    void readInstance(const string &filename) {
        instanceFilename = filename;
        ifstream fin(filename);
        if (!fin) {
            cerr << "Cannot open file: " << filename << endl;
            exit(1);
        }

        string line;
        // Search for "VEHICLE" section
        while (getline(fin, line)) {
            if (line.find("VEHICLE") != string::npos)
                break;
        }
        // Skip two lines after "VEHICLE"
        getline(fin, line);
        getline(fin, line);
        {
            istringstream iss(line);
            iss >> vehicleCount >> vehicleCapacity;
        }

        // Search for "CUSTOMER" section
        while (getline(fin, line)) {
            if (line.find("CUSTOMER") != string::npos)
                break;
        }
        // Skip header line
        getline(fin, line);
        // Read customer data
        while (getline(fin, line)) {
            if (line.empty()) continue;
            istringstream iss(line);
            Customer c;
            iss >> c.id >> c.x >> c.y >> c.demand >> c.readyTime >> c.dueTime >> c.serviceTime;
            customers.push_back(c);
        }
        numCustomers = customers.size();
        buildDistanceMatrix();  // Build distance matrix after reading customers
    }

    // Build the distance matrix between all customers
    void buildDistanceMatrix() {
        dist.resize(numCustomers, vector<double>(numCustomers, 0.0));
        for (int i = 0; i < numCustomers; ++i)
            for (int j = 0; j < numCustomers; ++j)
                dist[i][j] = euclidean(customers[i], customers[j]);
    }

    // Compute Euclidean distance between two customers
    double euclidean(const Customer &a, const Customer &b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    // Validate a route 
    bool validRoute(const vector<int>& route, int &load) {
        double time = 0.0;
        load = 0;
        // Route must start and end at depot (customer 0)
        if (route.front() != 0 || route.back() != 0) return false;
        for (size_t i = 1; i < route.size(); ++i) {
            int prev = route[i - 1];
            int curr = route[i];
            double travelTime = dist[prev][curr];
            double arrival = time + travelTime;
            // Service starts at max(arrival, readyTime)
            double start = max(arrival, static_cast<double>(customers[curr].readyTime));
            // If start time exceeds dueTime, route is invalid
            if (start > customers[curr].dueTime) return false;
            if (curr != 0) {  // For customers (excluding depot)
                time = start + customers[curr].serviceTime;
                load += customers[curr].demand;
                if (load > vehicleCapacity) return false;  // Capacity constraint
            } else {  // For depot, just update time
                time = start;
            }
        }
        if (time > customers[0].dueTime) return false; // Must return to depot within time window
        return true;
    }

    // Calculate cost of a single route (sum of distances)
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

    // Generate an initial solution using either greedy or random method
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

    // 2-opt improvement operator for a single route
    vector<int> twoOptSwap(const vector<int>& route) {
        vector<int> bestRoute = route;
        double bestCost = routeCost(route);
        bool improvement = true;
        while (improvement) {
            improvement = false;
            for (size_t i = 1; i < bestRoute.size() - 2; ++i) {
                for (size_t j = i + 1; j < bestRoute.size() - 1; ++j) {
                    vector<int> newRoute = bestRoute;
                    // Reverse the segment between positions i and j (inclusive)
                    reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
                    int load = 0;
                    if (validRoute(newRoute, load)) {
                        double newCost = routeCost(newRoute);
                        if (newCost < bestCost) {
                            bestCost = newCost;
                            bestRoute = newRoute;
                            improvement = true;
                        }
                    }
                }
            }
        }
        return bestRoute;
    }

    // Apply 2-opt improvement to each route in the solution
    vector<vector<int>> localSearch(const vector<vector<int>>& sol) {
        vector<vector<int>> newSol = sol;
        for (size_t r = 0; r < newSol.size(); ++r) {
            newSol[r] = twoOptSwap(newSol[r]);
        }
        return newSol;
    }

    // Generate a neighbor solution using a combination of neighborhood operators:
    vector<vector<int>> getNeighbor(const vector<vector<int>>& initSol) {
        vector<vector<int>> newSol = initSol;
        uniform_int_distribution<int> distRoute(0, newSol.size() - 1);
        uniform_real_distribution<double> choice(0.0, 1.0);
        double op = choice(rng);

        // Merge operator (30% probability)
        if (op < 0.3 && newSol.size() >= 2) {
            int r1 = distRoute(rng), r2 = distRoute(rng);
            while (r1 == r2) r2 = distRoute(rng);
            auto route1 = newSol[r1];
            auto route2 = newSol[r2];
            // Remove depot markers to merge the routes
            route1.erase(route1.begin());
            route1.pop_back();
            route2.erase(route2.begin());
            route2.pop_back();
            vector<int> merged = {0};
            merged.insert(merged.end(), route1.begin(), route1.end());
            merged.insert(merged.end(), route2.begin(), route2.end());
            merged.push_back(0);
            int load = 0;
            if (validRoute(merged, load)) {
                int high = max(r1, r2), low = min(r1, r2);
                newSol.erase(newSol.begin() + high);
                newSol.erase(newSol.begin() + low);
                newSol.push_back(merged);
                return localSearch(newSol);
            }
        }
        // Relocate operator (35% probability)
        else if (op < 0.65 && newSol.size() >= 2) {
            int r1 = distRoute(rng), r2 = distRoute(rng);
            while (r1 == r2 || newSol[r1].size() <= 3) {
                r1 = distRoute(rng);
                r2 = distRoute(rng);
            }
            uniform_int_distribution<int> distPos(1, newSol[r1].size() - 2);
            int pos = distPos(rng);
            int cust = newSol[r1][pos];
            vector<int> tempRoute = newSol[r1];
            tempRoute.erase(tempRoute.begin() + pos);
            int load = 0;
            if (!validRoute(tempRoute, load)) return initSol;
            newSol[r1] = tempRoute;
            int target = distRoute(rng);
            vector<int> newRoute = newSol[target];
            newRoute.insert(newRoute.end() - 1, cust);
            load = 0;
            if (validRoute(newRoute, load)) {
                newSol[target] = newRoute;
                return localSearch(newSol);
            }
        }
        // Swap operator (20% probability)
        else if (op < 0.85 && newSol.size() >= 2) {
            int r1 = distRoute(rng), r2 = distRoute(rng);
            while (r1 == r2 || newSol[r1].size() <= 3 || newSol[r2].size() <= 3) {
                r1 = distRoute(rng);
                r2 = distRoute(rng);
            }
            uniform_int_distribution<int> distPos1(1, newSol[r1].size() - 2);
            uniform_int_distribution<int> distPos2(1, newSol[r2].size() - 2);
            int pos1 = distPos1(rng), pos2 = distPos2(rng);
            swap(newSol[r1][pos1], newSol[r2][pos2]);
            int load1 = 0, load2 = 0;
            if (validRoute(newSol[r1], load1) && validRoute(newSol[r2], load2))
                return localSearch(newSol);
        }
        // 2-opt operator (15% probability)
        else {
            return localSearch(newSol);
        }
        return initSol;
    }

    // Checks if the entire solution is feasible:
    bool isFeasible(const vector<vector<int>>& sol) {
        if (sol.size() > static_cast<size_t>(vehicleCount)) return false;
        vector<bool> visited(numCustomers, false);
        visited[0] = true;
        for (const auto& route : sol) {
            int load = 0;
            for (size_t i = 1; i < route.size() - 1; ++i) {
                int cust = route[i];
                if (visited[cust]) return false;
                visited[cust] = true;
            }
            if (!validRoute(route, load)) return false;
        }
        return all_of(visited.begin(), visited.end(), [](bool v) { return v; });
    }

    // Simulated Annealing algorithm with multi-start and re-heating mechanism
    void solve(int maxTime, int maxEval) {
        const int numStarts = 1;           // Number of multi-start runs
        const int noImproveLimit = 100;    // Limit for iterations without improvement before re-heating
        vector<vector<int>> bestOverallSolution;
        double bestOverallObj = numeric_limits<double>::max();

        auto globalStart = chrono::steady_clock::now();

        for (int run = 0; run < numStarts; ++run) {
            cout << "Multi-start run: " << run + 1 << "\n";
            // Generate initial solution using greedy method
            vector<vector<int>> initSol = generateInitialSolution(true);

            // Randomize the initial temperature for this run in a range
            uniform_real_distribution<double> tempDist(0.8 * temperature, 1.2 * temperature);
            double runTemp = tempDist(rng);
            double temp = runTemp;
            vector<vector<int>> currentSol = initSol;
            double currObj = combinedObjective(currentSol);
            vector<vector<int>> bestRunSolution = currentSol;
            double bestRunObj = currObj;
            int evals = 1;
            int iteration = 0;
            int noImproveCount = 0;  // Count of iterations without improvement
            ofstream logFile("sa_log.csv");
            logFile << "Iteration,Temperature,initSolObj,BestFeasibleObj\n";

            auto runStart = chrono::steady_clock::now();

            while (true) {
                auto now = chrono::steady_clock::now();
                double elapsedGlobal = chrono::duration_cast<chrono::seconds>(now - globalStart).count();
                if ((maxTime > 0 && elapsedGlobal >= maxTime) || (maxEval > 0 && evals >= maxEval))
                    break;

                auto neighborSol = getNeighbor(currentSol);
                double neighObj = combinedObjective(neighborSol);
                evals++;
                double delta = neighObj - currObj;
                double acceptanceProbability = exp(-delta / temp);
                uniform_real_distribution<double> distProb(0.0, 1.0);
                bool accepted = false;

                // Decide whether to accept the neighbor solution
                if (delta < 0 || acceptanceProbability > distProb(rng)) {
                    currentSol = neighborSol;
                    currObj = neighObj;
                    accepted = true;
                    
                    if (currObj < bestRunObj) {
                        bestRunSolution = currentSol;
                        bestRunObj = currObj;
                        noImproveCount = 0;  // Reset no-improvement counter upon improvement
                    } else {
                        noImproveCount++;
                    }

                    if (isFeasible(currentSol)) {
                        double currentCost = totalCost(currentSol);
                        if (currentCost < bestFeasibleObj) {
                            bestFeasibleSolution = currentSol;
                            bestFeasibleObj = currentCost;
                        }
                    }
                } else {
                    noImproveCount++;
                }

                // Adaptive cooling: adjust cooling rate based on acceptance
                static double adaptiveCooling = coolingRate;
                if (accepted) {
                    adaptiveCooling = max(adaptiveCooling * 0.995, 0.9);
                } else {
                    adaptiveCooling = min(adaptiveCooling * 1.001, 0.999);
                }
                temp *= adaptiveCooling;

                // Re-heating: if temperature falls below final threshold or no improvement for many iterations, reset temperature
                if (temp < finalTemp || noImproveCount >= noImproveLimit) {
                    temp = runTemp;    // Reset to the initial temperature for this run
                    noImproveCount = 0;
                }

                logFile << iteration << "," << temp << "," << currObj << "," << bestFeasibleObj << "\n";
                iteration++;
            }

            cout << "Run " << run + 1 << " finished with best objective: " << bestRunObj << "\n";

            // Select the best overall solution among runs
            if (bestRunObj < bestOverallObj) {
                bestOverallSolution = bestRunSolution;
                bestOverallObj = bestRunObj;
            }
        } // End of multi-start runs

        // Output final solution if available
        if (!bestFeasibleSolution.empty()) {
            outputSolution(bestFeasibleSolution, instanceFilename);
        } else if (!bestOverallSolution.empty()) {
            outputSolution(bestOverallSolution, instanceFilename);
        } else {
            cout << "No feasible solution found after full search.\n";
        }

        auto globalEnd = chrono::steady_clock::now();
    }

    // Outputs the solution to console and writes it to a file
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
        cout << (isFeasible(sol) ? "✅ Solution is feasible.\n" : "❌ Solution is NOT feasible!\n");
        cout << "📄 Solution written to " << outputFile << "\n";
    }

    // Validates the solution by checking each route's feasibility and ensuring each customer is visited exactly once
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

    // Returns the best feasible solution found
    vector<vector<int>> getBestFeasibleSolution() const {
        return bestFeasibleSolution;
    }
};

int main(int argc, char* argv[]) {
    // Parse input arguments: instance file path, maximum execution time (seconds), and maximum number of evaluations
    string filename;
    int maxTime;
    int maxEval;
 
    if (argc == 4) {
        filename = argv[1];
        maxTime = atoi(argv[2]);
        maxEval = atoi(argv[3]);
    } else {
        cerr << "Usage: " << argv[0] << " [instance-file-path] [Max-execution-time-seconds] [Max-evaluation-number]\n";
        return 1;
    }
    
    VRPTWSolver solver;
    solver.readInstance(filename);
    
    auto globalStart = chrono::steady_clock::now();
    solver.solve(maxTime, maxEval);
    auto globalEnd = chrono::steady_clock::now();
    
    vector<vector<int>> bestSol = solver.getBestFeasibleSolution();
    
    cout << "\n========== Execution Summary ==========\n";
    cout << " Total Runtime: " 
         << chrono::duration_cast<chrono::seconds>(globalEnd - globalStart).count() 
         << " seconds\n";
    cout << " Max Time Allowed: " << maxTime << " seconds\n";
    cout << " Max Evaluations Allowed: " << maxEval << "\n";
    
    if (!bestSol.empty() && solver.validateSolution(bestSol)) {
        cout << " The solution is valid and feasible!\n";
    } else {
        cout << " The solution is not valid!\n";
    }
    return 0;
}
