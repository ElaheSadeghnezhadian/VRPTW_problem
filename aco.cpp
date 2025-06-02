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


// ------------------ ACO pa4rameters ----------------------------
const int POP_SIZE     = 30;      // number of ants
const double ALPHA     = 3;     // pheromone importance
const double BETA      = 1.0;     // heuristic importance
const double EVAPORATION  = 0.2;     // evaporation rate
const double Q         = 300.0;   // pheromone deposit factor

// ――― Min–Max & reset parameters ―――
const double TAU0      = 1.0;     // initial pheromone
// const double TAU_MIN   = 0.10;    // lower bound
// const double TAU_MAX   = 1.2;     // upper bound
const int    STAG_LIM  = 2;      // iterations w/out improvement → reset

    /*──── 0) پارامترهای کنترل ────*/
constexpr int    SIGMA    = 10;     // تعداد مورچهٔ نخبه
constexpr double TAU_MIN  = 1e-4;   // حد پایین فرومون
constexpr double TAU_MAX  = 10.0;   // حد بالا فرومون
constexpr double MIN_E    = 0.05;   // حداقل تبخیر
constexpr double MAX_E    = 0.25;   // حداکثر تبخیر

double q0_dynamic = 0.2;

struct Ant {
    Solution sol;                // مسیر فعلی
    Solution bestSol;            // بهترین مسیری که خودش تا حالا دیده
    int  noImprove = 0;          // چند بار پشت‌سرهم بهتر نشده
};

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

double objective1(const Solution &sol, bool vehiclePhase=false) {
    evaluationCounter++;
    if(vehiclePhase) {
        return (double)sol.size(); 
             }         // فقط تعداد خودرو
    /* ---- نسخهٔ اصلی شما: */
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

// === Pheromone initialisation & reset ===

void initializePheromone() {
    pheromone.assign(numCustomers, vector<double>(numCustomers, TAU0));
}

void resetPheromone(const Solution& best) {
    // مرحله ❶: تضعیف تمام لبه‌ها به صورت یکنواخت به سمت TAU_MIN
    for (int i = 0; i < numCustomers; ++i)
        for (int j = 0; j < numCustomers; ++j)
            pheromone[i][j] = max(TAU_MIN, pheromone[i][j] * 0.1);  // ← کاهش نرم

    // مرحله ❷: افزایش وزن لبه‌های در حل‌های خوب (Best solution)
    for (const auto& route : best) {
        for (size_t i = 1; i < route.size(); ++i) {
            int u = route[i - 1], v = route[i];

            // افزایش به صورت نرم تا حداکثر TAU_MAX
            pheromone[u][v] = pheromone[v][u] = min(
                TAU_MAX, pheromone[u][v] + 0.9 * (TAU_MAX - pheromone[u][v])
            );
        }
    }
}

void resetPheromone1(const Solution& best) {
    // مرحله ❶: کاهش فرومون همهٔ یال‌ها به TAU_MIN
    for (int i = 0; i < numCustomers; ++i)
        for (int j = 0; j < numCustomers; ++j)
            pheromone[i][j] = TAU_MIN;

    // مرحله ❷: افزایش فرومون فقط روی یال‌های راه‌حل بهترین مورچه
    for (const auto& route : best) {
        for (size_t i = 1; i < route.size(); ++i) {
            int u = route[i - 1], v = route[i];
            pheromone[u][v] = pheromone[v][u] = TAU_MAX;
        }
    }
}

void reinforceGlobal(const Solution& best) {
    // مرحله ❶: بازنشانی همهٔ مقادیر فرومون به TAU0 (مقدار پایه)
    for (int i = 0; i < numCustomers; ++i)
        for (int j = 0; j < numCustomers; ++j)
            pheromone[i][j] = TAU0;

    // مرحله ❷: تقویت لبه‌های استفاده‌شده در بهترین راه‌حل
    for (const auto& route : best) {
        for (size_t i = 1; i < route.size(); ++i) {
            int u = route[i - 1], v = route[i];
            pheromone[u][v] = pheromone[v][u] = TAU0 + (TAU_MAX - TAU0) * 0.8;  // تقویت نرم
        }
    }
}

/* ---------- 1) مسیر اولیه با نزدیک‌ترین همسایه ---------- */
Solution nearestNeighborHeuristic()
{
    vector<bool> vis(numCustomers,false);
    vis[0]=true;
    Solution init;
    int vehLoad = 0; 
    double vehTime = 0; 
    int cur = 0;
    init.push_back({0});               // مسیر فعلی (فقط دپو)
    for(int served=1; served<numCustomers; ++served)
    {
        /* پیدا کردن نزدیک‌ترین مشتری بازدیدنشده با قیود TW و ظرفیت */
        double bestD = 1e18; 
        int best = -1;
        for(int i=1;i<numCustomers;++i)
            if(!vis[i] && vehLoad+customers[i].demand<=vehicleCapacity){
                double reach = max(vehTime + dist[cur][i], (double)customers[i].readyTime);
                if(reach<=customers[i].dueTime && dist[cur][i] < bestD){
                    bestD = dist[cur][i]; best=i;
                }
            }
        if(best==-1){                  // وسیله فعلی پر شد ⇒ مسیر را ببند
            init.back().push_back(0);
            init.push_back({0});
            vehLoad=0; 
            vehTime=0; 
            cur=0;
            --served;                   // مشتری قبلی هنوز سرو نشده
            continue;
        }
        /* اضافه کردن مشتری best به مسیر جاری */
        init.back().push_back(best);
        vis[best]=true;
        vehTime = max(vehTime+dist[cur][best], (double)customers[best].readyTime)
                + customers[best].serviceTime;
        vehLoad += customers[best].demand;
        cur = best;
    }
    init.back().push_back(0);          // بستن آخرین مسیر
    return init;
}

// === Solution construction (unchanged + LS) ===
void updateQ0(double bestObj, double prevBestObj) {
    double improvement = prevBestObj - bestObj;

    if (improvement > 1e-6) {
        // اگر بهبود داشتیم q0 رو کمی افزایش بده (بیشتر exploitation)
        q0_dynamic += 0.05;
    } else {
        // اگر بهبود نداشتیم q0 رو کاهش بده (بیشتر exploration)
        q0_dynamic -= 0.05;
    }

    // محدود کردن q0 به بازه [0.05, 0.95]
    if (q0_dynamic < 0.05) q0_dynamic = 0.05;
    if (q0_dynamic > 0.95) q0_dynamic = 0.95;
}

int noImprovementCount = 0;  // خارج از تابع، در scope الگوریتم تعریف شود

void updateQ02(double bestObj, double prevBestObj, int& noImprovementCount) {
    double improvement = prevBestObj - bestObj;

    if (improvement > 1e-6) {
        q0_dynamic += 0.05; // افزایش q0 در صورت بهبود
        noImprovementCount = 0; // ریست شمارنده
    } else {
        noImprovementCount++;
        if (noImprovementCount >= 2) { // مثلاً بعد از 5 تکرار بدون بهبود
            q0_dynamic -= 0.05;       // کاهش q0 (exploration بیشتر)
            noImprovementCount = 0;   // ریست شمارنده بعد از اعمال تغییر
        }
    }

    // محدود کردن بازه q0
    if (q0_dynamic < 0.05) q0_dynamic = 0.05;
    if (q0_dynamic > 0.95) q0_dynamic = 0.95;
}

void updateQ03(bool improved) {
    if (improved) {
        q0_dynamic += 0.05;           // بهره‌برداری بیشتر
        noImprovementCount = 0;             // ریست شمارنده
    } else {
        ++noImprovementCount;
        if (noImprovementCount >= 5) {      // مثلاً پس از 5 رکود پیاپی
            q0_dynamic -= 0.05;       // اکتشاف بیشتر
            noImprovementCount = 0;         // ریست
        }
    }
    q0_dynamic = std::clamp(q0_dynamic, 0.05, 0.95);
}

Solution constructAntSolution(double q0)
{
    const int NN_SIZE = max(5, numCustomers / 4);   // فهرست همسایه‌ها

    vector<bool> visited(numCustomers, false);
    visited[0] = true;     // Depot
    Solution sol;

    auto hasFeasible = [&](int load, double time)->bool {
        for (int i = 1; i < numCustomers; ++i)
            if (!visited[i] &&
                load + customers[i].demand <= vehicleCapacity &&
                max(time + dist[0][i], (double)customers[i].readyTime) <= customers[i].dueTime)
                return true;
        return false;
    };

    while (hasFeasible(0, 0))                        // ⇦ بهبود ❶
    {
        vector<int> route = {0};
        int    load  = 0;
        double time  = 0.0;
        int    curr  = 0;

        while (true)
        {
            /*──── ❷ ساخت لیست نامزدهای نزدیک ────*/
            vector<int> candidates;
            candidates.reserve(NN_SIZE);
            for (int i = 1; i < numCustomers; ++i)
                if (!visited[i] &&
                    load + customers[i].demand <= vehicleCapacity &&
                    time + dist[curr][i]        <= customers[i].dueTime)
                    candidates.push_back(i);

            if (candidates.empty()) break;

            size_t limit = std::min((size_t)NN_SIZE, candidates.size());

            if (limit > 0 && candidates.size() > 1) {
                auto nth = candidates.begin() + (limit - 1);
                std::nth_element(candidates.begin(), nth, candidates.end(),
                                [&](int a, int b) { return dist[curr][a] < dist[curr][b]; });
                candidates.resize(limit);
            }


            /*──── ❸ محاسبهٔ احتمال‌ها (τ·η) ────*/
            vector<double> score(candidates.size());
            double sumProb = 0.0, bestVal = -1.0;
            int bestIdx = -1;

            for (size_t idx = 0; idx < candidates.size(); ++idx) {
                int i = candidates[idx];

                double slack = customers[i].dueTime
                               - max(time + dist[curr][i], (double)customers[i].readyTime);

                double eta = 1.0 / (dist[curr][i] + 1e-6)              // فاصله
                           + 0.001 * slack                              // اسلاک
                           + 0.0001 * (vehicleCapacity - load);         // ظرفیت باقیمانده

                double val = pow(pheromone[curr][i], ALPHA) *
                             pow(eta,               BETA  );

                score[idx] = val;
                sumProb   += val;

                if (val > bestVal) { bestVal = val; bestIdx = (int)idx; }
            }

            /*──── ❸ انتخاب: q0-Rule ────*/
            int next = -1;
        if (uniform_real_distribution<>(0, 1)(rng) < q0) {
            // Exploitation
            next = candidates[bestIdx];
        } else {
            // Exploration                                                  // Explore
                double r = uniform_real_distribution<>(0, sumProb)(rng);
                for (size_t idx = 0; idx < score.size(); ++idx) {
                    r -= score[idx];
                    if (r <= 0) { next = candidates[idx]; break; }
                }
            }
            if (next == -1) break;  // ایمنی

            /*──── حرکت به مشتری انتخاب‌شده ────*/
            route.push_back(next);
            visited[next] = true;

            time = max(time + dist[curr][next],
                       (double)customers[next].readyTime)
                 + customers[next].serviceTime;
            load += customers[next].demand;
            curr  = next;
        }

        route.push_back(0);
        if (route.size() > 2) {
            /*──── ❺ بهبود موضعی 2-Opt ────*/
            // localSearchTwoOpt(route);         // اگر نمی‌خواهی، این خط را کامنت کن
            sol.push_back(route);
        }
    }

    /*──── لاگ اختیاری ────*/
    if (logConstruction.is_open()) {
        logConstruction << "[Ant #" << evaluationCounter << "]\n";
        double total = 0;
        for (size_t r = 0; r < sol.size(); ++r) {
            logConstruction << "Route " << r + 1 << ": ";
            for (size_t j = 0; j < sol[r].size(); ++j)
                logConstruction << sol[r][j]
                                << (j + 1 < sol[r].size() ? " → " : "");
            double rc = routeCost(sol[r]);
            total += rc;
            logConstruction << " | Cost: " << fixed << setprecision(2) << rc << '\n';
        }
        logConstruction << "Total Routes: "   << sol.size() << '\n'
                        << "Total Distance: " << fixed << setprecision(2) << total
                        << "\n----------------------------\n";
    }
    return sol;
}

// === Enhanced pheromone update ===

void updatePheromone(const vector<Solution>& ants,const Solution& best,int iteration,
                     double bestObjective)
{
    // ── تبخیر ────────────────────────────────
    for (auto& row : pheromone)
        for (double& p : row)
            p *= (1.0 - EVAPORATION);

    // ── تقویت توسط همهٔ مورچه‌ها ─────────────
    for (const auto& sol : ants) {
        double q = Q / (totalCost(sol) + penaltyTerm(sol));
        for (const auto& r : sol)
            for (size_t i = 1; i < r.size(); ++i) {
                int u = r[i - 1], v = r[i];
                pheromone[u][v] += q;
                pheromone[v][u] += q;
            }
    }

    // ── تقویت اضافه توسط بهترین ─────────────
    double bestQ = Q * 2.0 / (totalCost(best) + penaltyTerm(best));
    for (const auto& r : best)
        for (size_t i = 1; i < r.size(); ++i) {
            int u = r[i - 1], v = r[i];
            pheromone[u][v] += bestQ;
            pheromone[v][u] += bestQ;
        }

    // ── لاگ فرومون ───────────────────────────
    if (logPheromone.is_open()) {
        logPheromone << "[Iteration " << iteration << "]\n";
        logPheromone << "Best Objective: "
                     << fixed << setprecision(2) << bestObjective << '\n';
        logPheromone << "Reinforced edges:\n";
        for (int u = 1; u < numCustomers; ++u)
            for (int v = u + 1; v < numCustomers; ++v)
                if (pheromone[u][v] > TAU0 + 1e-6)
                    logPheromone << "   (" << u << "," << v << "): "
                                 << fixed << setprecision(3)
                                 << pheromone[u][v] << ' ';
        logPheromone << "\n";
        logPheromone << "----------------------------\n";
        logPheromone.flush();
    }
}

/*--------------------------------------------------------------------
 | Enhanced pheromone update – Gupta & Saini 2017
 | σ (نخبه‌ها) = 10   •   تبخیر = EVAPORATION
 | لبه‌های best → تقویت اضافی با 2 × Q
 -------------------------------------------------------------------*/
void updatePheromoneEnhanced(const vector<Solution>& ants,
                             const Solution&        best,
                             int                    iteration,
                             double                 bestObjective)
{
    /*──── 1) تبخیر همگانی ────*/
    for (auto& row : pheromone)
        for (double& p : row)
            p *= (1.0 - EVAPORATION);

    /*──── 2) σ مورچهٔ برتر (Rank-based) ────*/
    constexpr int SIGMA = 10;                       // σ = 10
    struct Ranked { double obj; const Solution* s; };
    vector<Ranked> ranking;
    ranking.reserve(ants.size());
    for (const auto& a : ants)
        ranking.push_back({ objective(a) , &a });

    sort(ranking.begin(), ranking.end(),
         [](const Ranked& A, const Ranked& B){ return A.obj < B.obj; });

    int elite = min<int>(SIGMA, ranking.size());
    for (int r = 0; r < elite; ++r)                // r = 0 → بهترین
    {
        double delta = Q / ranking[r].obj;         // وزن فرومون
        const Solution& sol = *ranking[r].s;

        for (const auto& route : sol)
            for (size_t k = 1; k < route.size(); ++k)
            {
                int u = route[k-1], v = route[k];
                pheromone[u][v] += delta;
                pheromone[v][u] += delta;
            }
    }

    /*──── 3) تقویت اضافه برای بهترین تاریخی ────*/
    double boost = 2.0 * Q / (totalCost(best) + penaltyTerm(best));
    for (const auto& route : best)
        for (size_t k = 1; k < route.size(); ++k)
        {
            int u = route[k-1], v = route[k];
            pheromone[u][v] += boost;
            pheromone[v][u] += boost;
        }

    /*──── 4) لاگ مانند قبل ────*/
    if (logPheromone.is_open()) {
        logPheromone << "[Iteration " << iteration << "]\n"
                     << "Best Objective: "
                     << fixed << setprecision(2) << bestObjective << '\n'
                     << "Reinforced edges (τ > τ0):\n";

        for (int u = 1; u < numCustomers; ++u)
            for (int v = u + 1; v < numCustomers; ++v)
                if (pheromone[u][v] - TAU0 > 1e-6)
                    logPheromone << "   (" << u << "," << v << "): "
                                 << fixed << setprecision(3)
                                 << pheromone[u][v] << ' ';
        logPheromone << "\n----------------------------\n";
        logPheromone.flush();
    }
}

void updatePheromoneEnhanced2(const vector<Solution>& ants,
                             const Solution&        bestSoFar,
                             int                    iteration,
                             double                 bestObjective)
{
    /*── 0) پارامترها ──*/
    constexpr int   SIGMA   = 10;     // تعداد مورچهٔ نخبه
    constexpr double MIN_E  = 0.05;   // حداقل تبخیر
    constexpr double MAX_E  = 0.25;   // حداکثر تبخیر

    /*── 1) تبخیر تطبیقی ──*/
    // برآورد سادهٔ واریانس اهداف
    double mean = 0, mean2 = 0;
    for (const auto& a : ants) {
        double f = objective(a);
        mean  += f;  mean2 += f*f;
    }
    mean  /= ants.size();
    mean2 /= ants.size();
    double var = mean2 - mean*mean;            // واریانس
    double evap = MAX_E - (MAX_E-MIN_E) *       // هر چه var کوچک‌تر ⇒ تبخیر کم‌تر
                  std::clamp(var / (mean*mean), 0.0, 1.0);

    for (auto& row : pheromone)
        for (double& p : row)
            p = std::max(TAU_MIN, p * (1.0 - evap));

    /*── 2) رتبه‌بندی مورچه‌ها ──*/
    struct Ranked { double obj; const Solution* s; };
    vector<Ranked> ranking;
    ranking.reserve(ants.size());
    for (const auto& a : ants)
        ranking.push_back({ objective(a), &a });

    std::sort(ranking.begin(), ranking.end(),
              [](const Ranked& A, const Ranked& B){ return A.obj < B.obj; });

    int elite = std::min(SIGMA, (int)ranking.size());

    /*── 3) رسوب فرومونِ Rank-based ──*/
    for (int r = 0; r < elite; ++r)
    {
        double weight = (elite - r) / (double)elite;      // 1, 0.9, ...
        double delta  = weight * Q / ranking[r].obj;

        const Solution& sol = *ranking[r].s;
        for (const auto& route : sol)
            for (size_t k = 1; k < route.size(); ++k) {
                int u = route[k-1], v = route[k];
                pheromone[u][v] = std::min(TAU_MAX, pheromone[u][v] + delta);
                pheromone[v][u] = pheromone[u][v];
            }
    }

    /*── 4) تقویتِ بهترینِ تاریخی (elitist-best) ──*/
    double boost = 2.0 * Q / (totalCost(bestSoFar) + penaltyTerm(bestSoFar));
    for (const auto& route : bestSoFar)
        for (size_t k = 1; k < route.size(); ++k) {
            int u = route[k-1], v = route[k];
            pheromone[u][v] = std::min(TAU_MAX, pheromone[u][v] + boost);
            pheromone[v][u] = pheromone[u][v];
        }

    /*── 5) لاگ اختیاری ──*/
    if (logPheromone.is_open()) {
        logPheromone << "[It " << iteration << "] "
                     << "evap=" << std::fixed << std::setprecision(3) << evap
                     << "  bestObj=" << bestObjective << '\n';
        logPheromone.flush();
    }
}

void updatePheromoneEnhanced1(const vector<Solution>& ants,
                             const Solution&        bestSoFar,
                             int                    iteration,
                             double                 bestObjective)
{

    /*──── 1) تبخیر تطبیقی ────*/
    double mean = 0.0, mean2 = 0.0;
    for (const auto& a : ants) {
        double f = objective(a);
        mean  += f;
        mean2 += f * f;
    }
    mean  /= ants.size();
    mean2 /= ants.size();
    double var   = mean2 - mean * mean;
    double evap  = MAX_E - (MAX_E - MIN_E) * std::clamp(var / (mean * mean), 0.0, 1.0);

    for (auto& row : pheromone)
        for (double& p : row)
            p = std::max(TAU_MIN, p * (1.0 - evap));

    /*──── 2) σ مورچهٔ برتر (Rank-based) ────*/
    struct Ranked { double obj; const Solution* s; };
    vector<Ranked> ranking;
    ranking.reserve(ants.size());
    for (const auto& a : ants)
        ranking.push_back({ objective(a), &a });

    sort(ranking.begin(), ranking.end(),
         [](const Ranked& A, const Ranked& B) { return A.obj < B.obj; });

    int elite = std::min(SIGMA, (int)ranking.size());
    for (int r = 0; r < elite; ++r)
    {
        double weight = (elite - r) / (double)elite;
        double delta = weight * Q / ranking[r].obj;
        const Solution& sol = *ranking[r].s;

        for (const auto& route : sol)
            for (size_t k = 1; k < route.size(); ++k)
            {
                int u = route[k - 1], v = route[k];
                pheromone[u][v] = std::min(TAU_MAX, pheromone[u][v] + delta);
                pheromone[v][u] = pheromone[u][v];
            }
    }

    /*──── 3) تقویت اضافه برای بهترین تاریخی ────*/
    double boost = 2.0 * Q / (totalCost(bestSoFar) + penaltyTerm(bestSoFar));
    for (const auto& route : bestSoFar)
        for (size_t k = 1; k < route.size(); ++k)
        {
            int u = route[k - 1], v = route[k];
            pheromone[u][v] = std::min(TAU_MAX, pheromone[u][v] + boost);
            pheromone[v][u] = pheromone[u][v];
        }

    /*──── 4) لاگ مانند قبل ────*/
    if (logPheromone.is_open()) {
        logPheromone << "[Iteration " << iteration << "]\n"
                     << "Best Objective: "
                     << fixed << setprecision(2) << bestObjective << '\n'
                     << "Reinforced edges (τ > τ0):\n";

        for (int u = 1; u < numCustomers; ++u)
            for (int v = u + 1; v < numCustomers; ++v)
                if (pheromone[u][v] - TAU0 > 1e-6)
                    logPheromone << "   (" << u << "," << v << "): "
                                 << fixed << setprecision(3)
                                 << pheromone[u][v] << ' ';
        logPheromone << "\n----------------------------\n";
        logPheromone.flush();
    }
}

// === Main ACO loop ===

Solution antColonyOptimization(int maxTime, int maxEvaluations)
{
    int noImprovementCount = 0;

    initializePheromone();

    Solution globalBest; 
    double bestObj = numeric_limits<double>::max();
    int iter=0;
    int lastImproved=0;
    bool improvedThisIter = false;
    auto t0 = chrono::steady_clock::now();
    double prevBestObj = numeric_limits<double>::max();

    Solution nnhInit = nearestNeighborHeuristic();

    globalBest = nnhInit;                  // ← اضافه
    bestObj    = objective(globalBest);    // ← اضافه
    prevBestObj = bestObj;                 // ← اضافه برای updateQ0

    int bestVeh = static_cast<int>(nnhInit.size());
    Solution bestVehSol = nnhInit;


    if(logSummary.is_open()){
    logSummary << "iter="<<iter
                << " evaluation=" << evaluationCounter
                << " veh="   << bestVeh
                << " cost="  << fixed << setprecision(2) << totalCost(globalBest)
                << " bestObj=" << "00000000.00"
                << " q0=" << fixed << setprecision(3) << q0_dynamic
                << "s\n";
            }  
    for (const auto& r : nnhInit){           // τ روی مسیر NNH = TAU_MAX
        for (size_t k=1;k<r.size();++k){
            int u=r[k-1], v=r[k];
            pheromone[u][v] = pheromone[v][u] = TAU_MAX;
        }
    }


    while(true){
        // check stopping criteria
        auto now = chrono::steady_clock::now();
        if((maxTime>0 && chrono::duration_cast<chrono::seconds>(now-t0).count()>=maxTime) ||
           (maxEvaluations>0 && evaluationCounter>=maxEvaluations)) break;

        // ------ construct population ------
        vector<Solution> ants; 
        ants.reserve(POP_SIZE);

        // --- مورچهٔ 0 = مسیر NNH ---
        ants.push_back(nnhInit);                   // ارزیابی لازم است
        double obj0 = objective(nnhInit);
        if (obj0 < bestObj) {                      // (بعید ولی برای ایمنی)
            prevBestObj = bestObj;
            bestObj = obj0;
            globalBest = nnhInit;
        }

        // --- بقیۀ مورچه‌ها ---
        for(int i=1; i<POP_SIZE && (maxEvaluations==0 || evaluationCounter<maxEvaluations); ++i){
            Solution s = constructAntSolution(q0_dynamic);
            ants.push_back(s);
            double obj = objective(s);
            if(obj < bestObj){ 
                bestObj=obj; 
                globalBest=s; 
                lastImproved=iter;
                updateQ0(bestObj, bestObj); }

        }
                    // اگر هیچ بهبودی نداشتیم باز هم q0 را کاهش دهیم (اکتشاف بیشتر)
            if(iter - lastImproved > 0) {
                updateQ0(bestObj, bestObj + 1e-3);  // بدون بهبود، q0 کاهش می‌یابد
            }
                    // اگر هیچ بهبودی نداشتیم باز هم q0 را کاهش دهیم (اکتشاف بیشتر)
            // if(iter - lastImproved > 0) {
            //     updateQ0(bestObj, bestObj + 1e-3, noImprovementCount);  // افزایش شمارنده در تابع
            // }


        // ------ logs ------
        if (logSummary.is_open()) {
            double t = chrono::duration_cast<chrono::seconds>(now - t0).count();

            int    vehUsed  = static_cast<int>(globalBest.size());
            double bestDist = totalCost(globalBest);

            logSummary << "iter="   << iter
                    << " evaluation=" << evaluationCounter
                    << " veh="   << vehUsed
                    << " cost="  << fixed << setprecision(2) << bestDist
                    << " bestObj=" << fixed << setprecision(2) << bestObj
                    << " q0=" << fixed << setprecision(3) << q0_dynamic
                    << " time="  << t
                    << "s\n";
            logSummary.flush();
        }


        // ------ pheromone update ------
        // updatePheromone(ants, globalBest,iter, bestObj);
        // updatePheromoneEnhanced(ants, globalBest, iter, bestObj);
        updatePheromoneEnhanced1(ants, globalBest, iter, bestObj);


        // ------ stagnation check & reset ------
        if(iter-lastImproved >= STAG_LIM){
            resetPheromone(globalBest);
            // reinforceGlobal(globalBest);
            lastImproved = iter; // give fresh start
            if(logPheromone.is_open()) 
                logPheromone << "<RESET>\n";
        }

        ++iter;
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
