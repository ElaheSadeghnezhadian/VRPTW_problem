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

using namespace std;

// Data structure for a customer
struct Customer {
    int id;
    double x, y;
    int demand;
    int readyTime,dueTime,serviceTime;
};

class VRPTWSolver {
private:
    int vehicleCount, vehicleCapacity;
    int numCustomers;
    double temperature = 1000.0;
    double coolingRate = 0.995;
    double finalTemp = 1e-4;
    string instanceFilename;

    vector<Customer> customers;
    vector<vector<double>> dist;

    public:
    void readInstance(const string &filename) {
        instanceFilename = filename;
            ifstream fin(filename);
            if (!fin) {
                cerr << "Cannot open file: " << filename << endl;
                exit(1);
            }
    
            string line;
            while (getline(fin, line)) {
                if (line.find("VEHICLE") != string::npos)
                    break;
            }
            getline(fin, line); getline(fin, line);
            {
                istringstream iss(line);
                iss >> vehicleCount >> vehicleCapacity;
            }
    
            while (getline(fin, line)) {
                if (line.find("CUSTOMER") != string::npos)
                    break;
            }
            getline(fin, line);
            while (getline(fin, line)) {
                if (line.empty()) continue;
                istringstream iss(line);
                Customer c;
                iss >> c.id >> c.x >> c.y >> c.demand >> c.readyTime >> c.dueTime >> c.serviceTime;
                customers.push_back(c);
            }
            numCustomers = customers.size();
            buildDistanceMatrix();
        }
    
        void buildDistanceMatrix() {
            dist.resize(numCustomers, vector<double>(numCustomers, 0.0));
            for (int i = 0; i < numCustomers; ++i)
                for (int j = 0; j < numCustomers; ++j)
                    dist[i][j] = euclidean(customers[i], customers[j]);
        }

        double euclidean(const Customer &a, const Customer &b) {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
        }
    
        bool validRoute(const vector<int>& route, int &load) {
            double time = 0.0;
            load = 0;
        
            if (route.front() != 0 || route.back() != 0) return false; // باید از دپو شروع و به دپو ختم شود
        
            for (size_t i = 1; i < route.size(); ++i) {
                int prev = route[i - 1];
                int curr = route[i];
        
                double travelTime = dist[prev][curr];
                double arrival = time + travelTime;
                double start = max(arrival, (double)customers[curr].readyTime);
        
                // check time window
                if (start > customers[curr].dueTime) return false;
        
                // برای مشتری (نه دپو)، بررسی ظرفیت و آپدیت زمان
                if (curr != 0) {
                    time = start + customers[curr].serviceTime;
                    load += customers[curr].demand;
                    if (load > vehicleCapacity) return false;
                } else {
                    time = start;
                }
            }
        
            // بررسی برگشت به دپو در محدوده مجاز
            if (time > customers[0].dueTime) return false;
        
            return true;
        }    
    
        double routeCost(const vector<int>& route) {
            double cost = 0.0;
            for (size_t i = 0; i < route.size() - 1; ++i)
                cost += dist[route[i]][route[i + 1]];
            return cost;
        }
    
        double totalCost(const vector<vector<int>>& sol) {
            double cost = 0.0;
            for (const auto& r : sol)
                cost += routeCost(r);
            return cost;
        }
    
        double combinedObjective(const vector<vector<int>>& sol) {
            const double PENALTY = 1e9;
            return sol.size() * PENALTY + totalCost(sol);
        }
    
        vector<vector<int>> generateInitialSolution() {
            vector<pair<double, int>> sorted;
            for (int i = 1; i < numCustomers; ++i)
                sorted.emplace_back(dist[0][i], i);
            sort(sorted.begin(), sorted.end());
    
            vector<vector<int>> solution;
            vector<bool> visited(numCustomers, false);
            visited[0] = true;
    
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
            return solution;
        }
    
        vector<vector<int>> getNeighbor(const vector<vector<int>>& current) {
            vector<vector<int>> newSol = current;
    
            if (newSol.size() >= 2) {
                int r1 = rand() % newSol.size();
                int r2 = rand() % newSol.size();
                while (r1 == r2) r2 = rand() % newSol.size();
    
                auto route1 = newSol[r1], route2 = newSol[r2];
                route1.erase(route1.begin()); route1.pop_back();
                route2.erase(route2.begin()); route2.pop_back();
    
                vector<int> merged = {0};
                merged.insert(merged.end(), route1.begin(), route1.end());
                merged.insert(merged.end(), route2.begin(), route2.end());
                merged.push_back(0);
    
                int load = 0;
                if (validRoute(merged, load)) {
                    newSol.erase(newSol.begin() + max(r1, r2));
                    newSol.erase(newSol.begin() + min(r1, r2));
                    newSol.push_back(merged);
                    return newSol;
                }
            }
    
            int r1 = rand() % newSol.size(), r2 = rand() % newSol.size();
            while (r1 == r2 || newSol[r1].size() <= 3 || newSol[r2].size() <= 3) {
                r1 = rand() % newSol.size();
                r2 = rand() % newSol.size();
            }
    
            int i = rand() % (newSol[r1].size() - 2) + 1;
            int j = rand() % (newSol[r2].size() - 2) + 1;
    
            swap(newSol[r1][i], newSol[r2][j]);
    
            int load1 = 0, load2 = 0;
            if (validRoute(newSol[r1], load1) && validRoute(newSol[r2], load2)) {
                return newSol;
            }
    
            return current;
        }
    
        bool isFeasible(const vector<vector<int>>& sol) {
            if ((int)sol.size() > vehicleCount) return false; // قید تعداد وسیله
        
            vector<bool> visited(numCustomers, false);
            visited[0] = true; // دپو
        
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
        
        void solve(int maxTime, int maxEval) {
            auto start = chrono::steady_clock::now();
            srand(time(0));
        
            vector<vector<int>> current = generateInitialSolution();
            vector<vector<int>> best = current;
        
            double currObj = combinedObjective(current);
            double bestObj = currObj;
        
            // اینجا بهترین جواب فیزیبل رو نگه می‌داریم
            vector<vector<int>> bestFeasible;
            double bestFeasibleObj = numeric_limits<double>::max();
        
            double temp = temperature;
            int evals = 1;
        
            while (true) {
                auto now = chrono::steady_clock::now();
                double elapsed = chrono::duration_cast<chrono::seconds>(now - start).count();
        
                if ((maxTime > 0 && elapsed >= maxTime) || (maxEval > 0 && evals >= maxEval)) break;
        
                auto neighbor = getNeighbor(current);
                double neighObj = combinedObjective(neighbor);
                evals++;
        
                double delta = neighObj - currObj;
                if (delta < 0 || (exp(-delta / temp) > ((double) rand() / RAND_MAX))) {
                    current = neighbor;
                    currObj = neighObj;
        
                    if (currObj < bestObj) {
                        best = current;
                        bestObj = currObj;
                    }
        
                    // اگر فیزیبل بود، بررسی کن که بهترینه یا نه
                    if (isFeasible(current)) {
                        double currentCost = totalCost(current);
                        if (currentCost < bestFeasibleObj) {
                            bestFeasible = current;
                            bestFeasibleObj = currentCost;
                        }
                    }
                }
        
                temp *= coolingRate;
                if (temp < finalTemp) temp = temperature;
            }
        
            // بعد از پایان، فقط بهترین فیزیبل رو خروجی بده
            if (!bestFeasible.empty()) {
                outputSolution(bestFeasible, instanceFilename); // ← bestFeasible نه best
            }        
            else {
                cout << "❗ No feasible solution found after full search.\n";
            }
        }
    
        void outputSolution(const vector<vector<int>>& best, const string& inputFilename) {
            double cost = totalCost(best);
            cout << "Vehicles used: " << best.size() << "\n";
            cout << "Total cost: " << fixed << setprecision(2) << cost << "\n";
        
            for (size_t i = 0; i < best.size(); ++i) {
                cout << "Route " << i + 1 << ": ";
                for (size_t j = 1; j < best[i].size() - 1; ++j)
                    cout << best[i][j] << " ";
                cout << "| Cost: " << fixed << setprecision(2) << routeCost(best[i]) << "\n";
            }
        
            // استخراج نام فایل خروجی بر اساس نام ورودی
            string outputFile;
            size_t lastSlash = inputFilename.find_last_of("/\\");
            string base = (lastSlash == string::npos) ? inputFilename : inputFilename.substr(lastSlash + 1);
        
            size_t dot = base.find_last_of('.');
            if (dot != string::npos) {
                base = base.substr(0, dot);
            }
        
            outputFile = base + "_output.txt";
        
            // نوشتن خروجی در فایل
            ofstream fout(outputFile);
            for (size_t i = 0; i < best.size(); ++i) {
                fout << "Route " << i + 1 << ": ";
                for (size_t j = 1; j < best[i].size() - 1; ++j)
                    fout << best[i][j] << " ";
                fout << "\n";
            }
        
            fout << "Vehicles: " << best.size() << "\n";
            fout << "Distance: " << fixed << setprecision(2) << cost << "\n";
            fout.close();
        
            cout << (isFeasible(best) ? "✅ Solution is feasible.\n" : "❌ Solution is NOT feasible!\n");
            cout << "📄 Solution written to " << outputFile << "\n";
        }
};

        int main(int argc, char* argv[]) {
            if (argc != 4) {
                cerr << "Usage: " << argv[0] << " [instance-file-path] [Max-execution-time-seconds] [Max-evaluation-number]\n";
                return 1;
            }
        
            VRPTWSolver solver;
            solver.readInstance(argv[1]);
            solver.solve(atoi(argv[2]), atoi(argv[3]));
            return 0;
        }