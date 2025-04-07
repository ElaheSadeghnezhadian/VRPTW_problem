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

// ساختار داده‌ای برای مشتری
struct Customer {
    int id;
    double x, y;
    int demand;
    int readyTime, dueTime, serviceTime;
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

    // نگهداری بهترین جواب فیزیبل
    vector<vector<int>> bestFeasibleSolution;
    double bestFeasibleObj = numeric_limits<double>::max();

    // مولد اعداد تصادفی مدرن
    mt19937 rng;

public:
    VRPTWSolver() : rng(random_device{}()) {}

    // خواندن داده‌های ورودی از فایل
    void readInstance(const string &filename) {
        instanceFilename = filename;
        ifstream fin(filename);
        if (!fin) {
            cerr << "Cannot open file: " << filename << endl;
            exit(1);
        }

        string line;
        // جستجو برای بخش مربوط به وسایل نقلیه
        while (getline(fin, line)) {
            if (line.find("VEHICLE") != string::npos)
                break;
        }
        // پرش از دو خط بعد از VEHICLE
        getline(fin, line);
        getline(fin, line);
        {
            istringstream iss(line);
            iss >> vehicleCount >> vehicleCapacity;
        }

        // جستجو برای بخش مربوط به مشتریان
        while (getline(fin, line)) {
            if (line.find("CUSTOMER") != string::npos)
                break;
        }
        // پرش از خط عنوان
        getline(fin, line);
        // خواندن داده‌های مشتریان
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

    // ساخت ماتریس فاصله بین مشتریان
    void buildDistanceMatrix() {
        dist.resize(numCustomers, vector<double>(numCustomers, 0.0));
        for (int i = 0; i < numCustomers; ++i)
            for (int j = 0; j < numCustomers; ++j)
                dist[i][j] = euclidean(customers[i], customers[j]);
    }

    // محاسبه فاصله اقلیدسی
    double euclidean(const Customer &a, const Customer &b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    // بررسی اعتبار یک مسیر از نظر پنجره‌های زمانی و ظرفیت
    bool validRoute(const vector<int>& route, int &load) {
        double time = 0.0;
        load = 0;

        // مسیر باید از دپو شروع و به دپو ختم شود
        if (route.front() != 0 || route.back() != 0) return false;

        for (size_t i = 1; i < route.size(); ++i) {
            int prev = route[i - 1];
            int curr = route[i];

            double travelTime = dist[prev][curr];
            double arrival = time + travelTime;
            double start = max(arrival, static_cast<double>(customers[curr].readyTime));

            // بررسی پنجره زمانی
            if (start > customers[curr].dueTime) return false;

            // برای مشتری (غیر از دپو)
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

    // محاسبه هزینه یک مسیر (فاصله طی‌شده)
    double routeCost(const vector<int>& route) {
        double cost = 0.0;
        for (size_t i = 0; i < route.size() - 1; ++i)
            cost += dist[route[i]][route[i + 1]];
        return cost;
    }

    // محاسبه هزینه کل یک جواب (مجموع هزینه مسیرها)
    double totalCost(const vector<vector<int>>& sol) {
        double cost = 0.0;
        for (const auto& r : sol)
            cost += routeCost(r);
        return cost;
    }

    // تابع هدف ترکیبی: تعداد مسیرها (به عنوان جریمه بسیار بزرگ) + هزینه کل
    double combinedObjective(const vector<vector<int>>& sol) {
        const double PENALTY = 1e9;
        return sol.size() * PENALTY + totalCost(sol);
    }

    // تولید جواب اولیه ساده با استفاده از روش نزدیگی
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

    // عملگر بهبود مسیر به کمک الگوریتم 2-opt
    vector<int> twoOptSwap(const vector<int>& route) {
        vector<int> bestRoute = route;
        double bestCost = routeCost(route);
        bool improvement = true;

        while (improvement) {
            improvement = false;
            for (size_t i = 1; i < bestRoute.size() - 2; ++i) {
                for (size_t j = i + 1; j < bestRoute.size() - 1; ++j) {
                    vector<int> newRoute = bestRoute;
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

    // عملگر محلی: اعمال بهبود 2-opt بر روی تمام مسیرها در یک جواب
    vector<vector<int>> localSearch(const vector<vector<int>>& sol) {
        vector<vector<int>> newSol = sol;
        for (size_t r = 0; r < newSol.size(); ++r) {
            newSol[r] = twoOptSwap(newSol[r]);
        }
        return newSol;
    }

    // تولید همسایه با ترکیب چند عملگر: ادغام، جابجایی بین مسیرها، انتقال مشتری و بهبود محلی 2-opt
    vector<vector<int>> getNeighbor(const vector<vector<int>>& current) {
        vector<vector<int>> newSol = current;
        uniform_int_distribution<int> distRoute(0, newSol.size() - 1);
        uniform_real_distribution<double> choice(0.0, 1.0);
        double op = choice(rng);

        // عملگر ادغام دو مسیر (Merge) – احتمال 30%
        if (op < 0.3 && newSol.size() >= 2) {
            int r1 = distRoute(rng);
            int r2 = distRoute(rng);
            while (r1 == r2) r2 = distRoute(rng);
            auto route1 = newSol[r1];
            auto route2 = newSol[r2];
            // حذف دپوها برای ادغام
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
                return localSearch(newSol); // بهبود محلی روی جواب
            }
        }
        // عملگر انتقال (Relocate) – احتمال 35%
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
            if (!validRoute(tempRoute, load)) return current; // عدم موفقیت
            newSol[r1] = tempRoute;
            // اضافه کردن مشتری به یک مسیر تصادفی دیگر
            int target = distRoute(rng);
            vector<int> newRoute = newSol[target];
            newRoute.insert(newRoute.end() - 1, cust);
            load = 0;
            if (validRoute(newRoute, load)) {
                newSol[target] = newRoute;
                return localSearch(newSol);
            }
        }
        // عملگر جابجایی بین مسیرها (Swap) – احتمال 20%
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
        // عملگر بهبود محلی (2-opt) – احتمال 15%
        else {
            return localSearch(newSol);
        }
        return current;
    }

    // بررسی اعتبار کلی یک جواب (تعداد وسایل، بازدید یکتای مشتریان و سایر محدودیت‌ها)
    bool isFeasible(const vector<vector<int>>& sol) {
        if (sol.size() > static_cast<size_t>(vehicleCount)) return false;
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

    // الگوریتم Simulated Annealing با استفاده از چند عملگر محلی برای بهبود کیفیت جواب
    void solve(int maxTime, int maxEval) {
        auto start = chrono::steady_clock::now();
        vector<vector<int>> current = generateInitialSolution();
        vector<vector<int>> best = current;
        double currObj = combinedObjective(current);
        double bestObj = currObj;
        int evals = 1;
        double temp = temperature;
        int iteration = 0;
    
        ofstream logFile("sa_log.csv");
        logFile << "Iteration,Temperature,CurrentObj,BestFeasibleObj\n";
    
        double adaptiveCooling = coolingRate;
    
        while (true) {
            auto now = chrono::steady_clock::now();
            double elapsed = chrono::duration_cast<chrono::seconds>(now - start).count();
            if ((maxTime > 0 && elapsed >= maxTime) || (maxEval > 0 && evals >= maxEval))
                break;
    
            auto neighbor = getNeighbor(current);
            double neighObj = combinedObjective(neighbor);
            evals++;
            double delta = neighObj - currObj;
            double acceptanceProbability = exp(-delta / temp);
            uniform_real_distribution<double> distProb(0.0, 1.0);
            bool accepted = false;
    
            if (delta < 0 || acceptanceProbability > distProb(rng)) {
                current = neighbor;
                currObj = neighObj;
                accepted = true;
    
                if (currObj < bestObj) {
                    best = current;
                    bestObj = currObj;
                }
    
                if (isFeasible(current)) {
                    double currentCost = totalCost(current);
                    if (currentCost < bestFeasibleObj) {
                        bestFeasibleSolution = current;
                        bestFeasibleObj = currentCost;
                    }
                }
            }
    
            // Adaptive cooling: کند یا تند شدن نرخ خنک‌سازی بسته به اینکه پذیرش داشتیم یا نه
            if (accepted) {
                adaptiveCooling = max(adaptiveCooling * 0.999, 0.9); // سریع‌تر سرد بشه
            } else {
                adaptiveCooling = min(adaptiveCooling * 1.001, 0.999); // آهسته‌تر سرد بشه
            }
    
            temp *= adaptiveCooling;
    
            logFile << iteration << "," << temp << "," << currObj << "," << bestFeasibleObj << "\n";
            iteration++;
    
            if (temp < finalTemp)
                break;
        }
    
        logFile.close();
    
        if (!bestFeasibleSolution.empty()) {
            outputSolution(bestFeasibleSolution, instanceFilename);
        } else {
            cout << "❗ No feasible solution found after full search.\n";
        }
    }    

    // نوشتن خروجی به فرمت مشخص شده
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
        // استخراج نام فایل خروجی از نام فایل ورودی
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

    // اعتبارسنجی جامع جواب با بررسی تمامی محدودیت‌ها
    bool validateSolution(const vector<vector<int>>& solution) {
        if (solution.size() > static_cast<size_t>(vehicleCount)) {
            cout << "❌ Number of vehicles exceeds the available vehicle count.\n";
            return false;
        }
        vector<bool> visited(numCustomers, false);
        visited[0] = true; // دپو
        for (const auto& route : solution) {
            int load = 0;
            double time = 0.0;
            if (route.front() != 0 || route.back() != 0) {
                cout << "❌ Route does not start and end at depot.\n";
                return false;
            }
            for (size_t i = 1; i < route.size() - 1; ++i) {
                int curr = route[i];
                if (visited[curr]) {
                    cout << "❌ Customer " << curr << " visited multiple times.\n";
                    return false;
                }
                visited[curr] = true;
                double travelTime = dist[route[i - 1]][curr];
                double arrivalTime = time + travelTime;
                double startTime = max(arrivalTime, static_cast<double>(customers[curr].readyTime));
                if (startTime > customers[curr].dueTime) {
                    cout << "❌ Customer " << curr << " violated time window.\n";
                    return false;
                }
                time = startTime + customers[curr].serviceTime;
                load += customers[curr].demand;
                if (load > vehicleCapacity) {
                    cout << "❌ Vehicle exceeded capacity at customer " << curr << ".\n";
                    return false;
                }
            }
            int lastCustomer = route[route.size() - 2];
            double returnTime = time + dist[lastCustomer][0];
            if (returnTime > customers[0].dueTime) {
                cout << "❌ Vehicle violated return time to depot.\n";
                return false;
            }
        }
        for (int i = 1; i < numCustomers; ++i) {
            if (!visited[i]) {
                cout << "❌ Customer " << i << " was not visited.\n";
                return false;
            }
        }
        cout << "✅ Solution is valid!\n";
        return true;
    }

    // تابع جهت دریافت بهترین جواب فیزیبل برای استفاده در main
    vector<vector<int>> getBestFeasibleSolution() const {
        return bestFeasibleSolution;
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
    vector<vector<int>> bestSol = solver.getBestFeasibleSolution();
    if (!bestSol.empty() && solver.validateSolution(bestSol)) {
        cout << "The solution is valid and feasible!\n";
    } else {
        cout << "The solution is not valid.\n";
    }
    return 0;
}
