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
    double temperature = 1000.0;   // دمای اولیه
    double coolingRate = 0.995;      // نرخ کاهش دما
    double finalTemp = 1e-4;         // دمای نهایی (برای توقف)
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
            if (curr != 0) {
                time = start + customers[curr].serviceTime;
                load += customers[curr].demand;
                if (load > vehicleCapacity) return false;
            } else {
                time = start;
            }
        }
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
// تولید جواب اولیه با دو حالت: greedy یا random
    vector<vector<int>> generateInitialSolution(bool useGreedy = true) {
        vector<vector<int>> solution;
        vector<bool> visited(numCustomers, false);
        visited[0] = true; // دپو
        
        if (useGreedy) {
            // حالت greedy: مشتریان بر اساس فاصله از دپو مرتب می‌شوند
            vector<pair<double, int>> sorted;
            for (int i = 1; i < numCustomers; ++i)
                sorted.emplace_back(dist[0][i], i);
            sort(sorted.begin(), sorted.end());
        
            for (auto &[_, cust] : sorted) {
                if (visited[cust]) continue;
                bool added = false;
                // سعی در افزودن مشتری به مسیرهای موجود
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
                // اگر نتوانستیم به مسیر موجود اضافه کنیم، مسیر جدیدی ایجاد کن
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
        else { // حالت random
            // ایجاد یک لیست از مشتریان (به جز دپو)
            vector<int> custList;
            for (int i = 1; i < numCustomers; ++i)
                custList.push_back(i);
        
            // مخلوط کردن لیست مشتریان به‌صورت تصادفی
            shuffle(custList.begin(), custList.end(), rng);
        
            // سعی در ساخت مسیرهای فیزیبل به‌صورت تصادفی
            for (int cust : custList) {
                bool inserted = false;
                // تلاش برای اضافه کردن مشتری به یک مسیر تصادفی موجود
                vector<int> indices(solution.size());
                for (size_t i = 0; i < solution.size(); ++i) indices[i] = i;
                shuffle(indices.begin(), indices.end(), rng);
        
                for (int idx : indices) {
                    // سعی در افزودن مشتری به انتهای مسیر (پیش از دپو پایانی)
                    vector<int> temp = solution[idx];
                    temp.insert(temp.end() - 1, cust);
                    int load = 0;
                    if (validRoute(temp, load)) {
                        solution[idx].insert(solution[idx].end() - 1, cust);
                        inserted = true;
                        break;
                    }
                }
                // اگر مشتری در هیچ مسیری اضافه نشد، مسیر جدید بساز
                if (!inserted) {
                    vector<int> newRoute = {0, cust, 0};
                    int load = 0;
                    if (validRoute(newRoute, load)) {
                        solution.push_back(newRoute);
                    }
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

    // تولید همسایه با ترکیب چند عملگر: ادغام، انتقال (Relocate)، جابجایی (Swap) و بهبود محلی (2-opt)
    vector<vector<int>> getNeighbor(const vector<vector<int>>& initSol) {
        vector<vector<int>> newSol = initSol;
        uniform_int_distribution<int> distRoute(0, newSol.size() - 1);
        uniform_real_distribution<double> choice(0.0, 1.0);
        double op = choice(rng);

        // عملگر ادغام دو مسیر (Merge) – احتمال 30%
        if (op < 0.3 && newSol.size() >= 2) {
            int r1 = distRoute(rng), r2 = distRoute(rng);
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
                return localSearch(newSol);
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
        return initSol;
    }

    // بررسی اعتبار کلی یک جواب (تعداد وسایل، یکتایی بازدید مشتریان و سایر محدودیت‌ها)
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

    // الگوریتم Simulated Annealing با چند عملگر محلی برای بهبود کیفیت جواب
 // الگوریتم Simulated Annealing با چند عملگر محلی برای بهبود کیفیت جواب
    void solve(int maxTime, int maxEval) {
        const int numStarts = 5;           // تعداد اجرای Multi-Start
        const int noImproveLimit = 100;    // تعداد تکرار بدون بهبود قبل از ری-هیتر
        vector<vector<int>> bestOverallSolution;
        double bestOverallObj = numeric_limits<double>::max();

        auto globalStart = chrono::steady_clock::now();

        for (int run = 0; run < numStarts; ++run) {
            cout << "Multi-start run: " << run + 1 << "\n";
            // تولید جواب اولیه؛ در اینجا از حالت greedy استفاده می‌کنیم
            vector<vector<int>> initSol = generateInitialSolution(true);
            // تنظیم دمای اولیه متفاوت برای هر اجرا (به صورت تصادفی در بازه‌ای مشخص)
            uniform_real_distribution<double> tempDist(0.8 * temperature, 1.2 * temperature);
            double runTemp = tempDist(rng);
            double temp = runTemp;
            vector<vector<int>> currentSol = initSol;
            double currObj = combinedObjective(currentSol);
            vector<vector<int>> bestRunSolution = currentSol;
            double bestRunObj = currObj;
            int evals = 1;
            int iteration = 0;
            int noImproveCount = 0;  // شمارنده تکرار بدون بهبود
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

                if (delta < 0 || acceptanceProbability > distProb(rng)) {
                    currentSol = neighborSol;
                    currObj = neighObj;
                    accepted = true;

                    if (currObj < bestRunObj) {
                        bestRunSolution = currentSol;
                        bestRunObj = currObj;
                        noImproveCount = 0;  // بهبود حاصل شده؛ شمارنده صفر می‌شود
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

                // تنظیم نرخ خنک‌سازی تطبیقی بر اساس پذیرش
                static double adaptiveCooling = coolingRate;
                if (accepted) {
                    adaptiveCooling = max(adaptiveCooling * 0.999, 0.9);
                } else {
                    adaptiveCooling = min(adaptiveCooling * 1.001, 0.999);
                }
                temp *= adaptiveCooling;

                // اگر دما به مقدار نهایی رسید یا بهبود برای noImproveLimit تکرار حاصل نشد، ری-هیتر می‌کنیم
                if (temp < finalTemp || noImproveCount >= noImproveLimit) {
                    // می‌توانیم دما را کمی بالاتر از دمای اولیه فعلی برای این اجرا تنظیم کنیم
                    temp = runTemp;  
                    noImproveCount = 0;
                    logFile << iteration << "," << temp << "," << currObj << "," << bestFeasibleObj << "\n";
                    iteration++;
                    // در صورت نیاز می‌توان به عنوان یک multi-start جزئی هم اقدام به تولید جواب اولیه جدید کرد
                }
                iteration++;
            } // پایان حلقه SA برای این اجرا

            cout << "Run " << run + 1 << " finished with best objective: " << bestRunObj << "\n";

            // انتخاب بهترین جواب از میان اجرای فعلی و بهترین کلی
            if (bestRunObj < bestOverallObj) {
                bestOverallSolution = bestRunSolution;
                bestOverallObj = bestRunObj;
            }
        } // پایان multi-start

        // ثبت نهایی بهترین جواب فیزیبل در صورت وجود
        if (!bestFeasibleSolution.empty()) {
            outputSolution(bestFeasibleSolution, instanceFilename);
        } else if (!bestOverallSolution.empty()) {
            outputSolution(bestOverallSolution, instanceFilename);
        } else {
            cout << "❗ No feasible solution found after full search.\n";
        }

        auto globalEnd = chrono::steady_clock::now();
        cout << "Total Time spent in solve: "
            << chrono::duration_cast<chrono::seconds>(globalEnd - globalStart).count()
            << " seconds\n";
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
        visited[0] = true;
        for (const auto& route : solution) {
            int load = 0;
            for (size_t i = 1; i < route.size() - 1; ++i) {
                int cust = route[i];
                if (visited[cust]) {
                    cout << "❌ Customer " << cust << " visited multiple times.\n";
                    return false;
                }
                visited[cust] = true;
            }
            if (!validRoute(route, load)) return false;
        }
        return all_of(visited.begin(), visited.end(), [](bool v) { return v; });
    }
    
    // تابع جهت دریافت بهترین جواب فیزیبل برای استفاده در main
    vector<vector<int>> getBestFeasibleSolution() const {
        return bestFeasibleSolution;
    }
};
    
int main(int argc, char* argv[]) {
    // const char* default_value_1 = "100-ce-8.txt"; // مقدار پیش‌فرض برای فایل
    // const char* default_value_2 = "2000";         // مقدار پیش‌فرض برای زمان اجرا (ثانیه)
    // const char* default_value_3 = "2000";         // مقدار پیش‌فرض برای تعداد ارزیابی‌ها
    
    string filename;
    int maxTime;
    int maxEval;
    
    // // استفاده از مقادیر پیش‌فرض در صورت نبود ورودی
    // if (argc == 1) {
    //     filename = default_value_1;
    //     maxTime = atoi(default_value_2);
    //     maxEval = atoi(default_value_3);
    // }
    // else if (argc == 2) {
    //     filename = argv[1];
    //     maxTime = atoi(default_value_2);
    //     maxEval = atoi(default_value_3);
    // }
    // else if (argc == 3) {
    //     filename = argv[1];
    //     maxTime = atoi(argv[2]);
    //     maxEval = atoi(default_value_3);
    // }
    // else 
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
    cout << "⏱️  Total Runtime: " << chrono::duration_cast<chrono::seconds>(globalEnd - globalStart).count() << " seconds\n";
    cout << "📊 Max Time Allowed: " << maxTime << " seconds\n";
    cout << "📈 Max Evaluations Allowed: " << maxEval << "\n";
    
    if (!bestSol.empty() && solver.validateSolution(bestSol)) {
        cout << "✅ The solution is valid and feasible!\n";
    } else {
        cout << "❌ The solution is not valid.\n";
    }
    return 0;
}