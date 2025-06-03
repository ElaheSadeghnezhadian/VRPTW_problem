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
#include <deque>
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


// ------------------ ACO parameters ----------------------------
const int POP_SIZE     = 30;      // number of ants
const double ALPHA     = 3;     // pheromone importance
const double BETA      = 1;     // heuristic importance
const double EVAPORATION  = 0.15;     // evaporation rate
const double Q         = 200.0;   // pheromone deposit factor

// ――― Min–Max & reset parameters ―――
const double TAU0      = 1.0;     // initial pheromone
const double TAU_MIN   = 0.10;    // lower bound
const double TAU_MAX   = 1.2;     // upper bound
const int    STAG_LIM  = 4;      // iterations w/out improvement → reset

int elitist_weight = 5; //یا 10، قابل تنظیمه
double q0_dynamic = 0.1;

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

// 
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

// =====
/* بار باقیماندهٔ یک مسیر (بدون چک کامل TW) */
int routeLoad(const vector<int>& r)
{
    int load = 0;
    for (size_t k = 1; k + 1 < r.size(); ++k)
        load += customers[r[k]].demand;
    return load;
}

/* زمان پایان سرویس مسیر (مورد استفادهٔ سریع) */
double routeFinishTime(const vector<int>& r)
{
    double t = 0;
    for (size_t k = 1; k < r.size(); ++k) {
        int u = r[k-1], v = r[k];
        t = max(t + dist[u][v], (double)customers[v].readyTime)
            + customers[v].serviceTime;
    }
    return t;
}

// ====
bool quickImprove(Solution& sol)
{
    /*── Relocate ──*/
    for (size_t a = 0; a < sol.size(); ++a)
        for (size_t i = 1; i + 1 < sol[a].size(); ++i)
            for (size_t b = 0; b < sol.size(); ++b)
            {
                if (a == b) continue;

                int cust = sol[a][i];
                // ظرفیت سریع:
                if (routeLoad(sol[b]) + customers[cust].demand > vehicleCapacity)
                    continue;

                double bestInc = 1e18;
                size_t bestPos = 0;
                /* درج در بهترین موقعیت مسیر b */
                for (size_t pos = 1; pos < sol[b].size(); ++pos) {
                    sol[b].insert(sol[b].begin() + pos, cust);
                    if (validRoute(sol[b])) {
                        double inc = routeCost(sol[b]);
                        if (inc < bestInc) { bestInc = inc; bestPos = pos; }
                    }
                    sol[b].erase(sol[b].begin() + pos);
                }
                if (bestInc >= 1e18) continue;  // هیچ جای معتبر

                // اعمال جابه‌جایی
                sol[a].erase(sol[a].begin() + i);
                sol[b].insert(sol[b].begin() + bestPos, cust);
                if (sol[a].size() == 2) sol.erase(sol.begin() + a); // مسیر خالی شد
                return true;   // اولین بهبود
            }

    /*── Swap ──*/
    for (size_t a = 0; a < sol.size(); ++a)
        for (size_t i = 1; i + 1 < sol[a].size(); ++i)
            for (size_t b = a + 1; b < sol.size(); ++b)
                for (size_t j = 1; j + 1 < sol[b].size(); ++j)
                {
                    int ca = sol[a][i], cb = sol[b][j];
                    if (routeLoad(sol[a]) - customers[ca].demand + customers[cb].demand > vehicleCapacity) continue;
                    if (routeLoad(sol[b]) - customers[cb].demand + customers[ca].demand > vehicleCapacity) continue;

                    swap(sol[a][i], sol[b][j]);
                    if (validRoute(sol[a]) && validRoute(sol[b]) &&
                        routeCost(sol[a]) + routeCost(sol[b]) <
                        routeCost(sol[a]) + routeCost(sol[b]) + 1e-6)  // فقط معتبر بودن کافی است
                        return true;
                    swap(sol[a][i], sol[b][j]);  // برگشت
                }
    return false;
}

// === Pheromone initialisation & reset ===

void initializePheromone() {
    pheromone.assign(numCustomers, vector<double>(numCustomers, TAU0));
}

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

void resetPheromone(const Solution&              best,
                            const vector<Solution>&      elitePool,
                            int                          noImproveIter)
{
    /*──── 1) تبخیر تطبیقی ────*/
    double rho   = std::min(0.35, 0.05 + 0.02 * noImproveIter);
    for (int i = 0; i < numCustomers; ++i)
        for (int j = 0; j < numCustomers; ++j) {
            pheromone[i][j] *= (1.0 - rho);
            pheromone[i][j]  = std::max(pheromone[i][j], TAU_MIN);
        }

    /*──── 2) تقویت بهترین تاریخی ────*/
    double deltaBest = 2.0 * Q / totalCost(best);   // وزن دو برابر
    for (const auto& route : best)
        for (size_t k = 1; k < route.size(); ++k) {
            int u = route[k-1], v = route[k];
            pheromone[u][v] = pheromone[v][u] =
                std::min(TAU_MAX, pheromone[u][v] + deltaBest);
        }

    /*──── 3) تقویت نخبه‌های اخیر (حداکثر 3) ────*/
    int pick = std::min<int>(3, elitePool.size());
    for (int e = 0; e < pick; ++e) {
        double delta = Q / totalCost(elitePool[e]);   // وزن معمولی
        for (const auto& route : elitePool[e])
            for (size_t k = 1; k < route.size(); ++k) {
                int u = route[k-1], v = route[k];
                pheromone[u][v] = pheromone[v][u] =
                    std::min(TAU_MAX, pheromone[u][v] + delta);
            }
    }

    /*──── 4) تنوّع تصادفی روی ۲٪ یال‌ها ────*/
    int injectCnt = static_cast<int>(0.02 * numCustomers * numCustomers);
    uniform_int_distribution<int> rndNode(0, numCustomers - 1);
    for (int t = 0; t < injectCnt; ++t) {
        int a = rndNode(rng), b = rndNode(rng);
        if (a == b) continue;
        pheromone[a][b] = pheromone[b][a] =
            std::min(TAU_MAX, pheromone[a][b] + 0.05 * (TAU_MAX - TAU_MIN));
    }
}

// === Solution construction (unchanged + LS) ===
void updateQ0(bool improved)       // فقط یک فلَگ
{
    if (improved)          // بهبود داشتیم → exploitation بیشتر
        q0_dynamic += 0.05;
    else                   // رکود → exploration بیشتر
        q0_dynamic -= 0.05;

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
                time + dist[0][i] <= customers[i].dueTime)
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

            nth_element(candidates.begin(),
                         candidates.begin() + min<int>(NN_SIZE, candidates.size()),
                         candidates.end(),
                         [&](int a, int b){ return dist[curr][a] < dist[curr][b]; });
            if (candidates.size() > NN_SIZE)
                candidates.resize(NN_SIZE);

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

            if (q0 == 0.0) {
                // ⇨ مسیر تصادفی واقعی (explore خالص)
                uniform_int_distribution<int> randIndex(0, (int)candidates.size() - 1);
                next = candidates[randIndex(rng)];
            } else {
                // ⇨ حالت عادی ACO (q0-rule)
                double q = uniform_real_distribution<>(0, 1)(rng);
                if (q < q0) {
                    // Exploitation
                    next = candidates[bestIdx];
                } else {
                    // Exploration با وزن احتمال‌ها
                    double r = uniform_real_distribution<>(0, sumProb)(rng);
                    for (size_t idx = 0; idx < score.size(); ++idx) {
                        r -= score[idx];
                        if (r <= 0) { next = candidates[idx]; break; }
                    }
                }
            }


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
    
    quickImprove(sol);   // بهبود سریع بین‌مسیر
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

// === Main ACO loop ===

Solution antColonyOptimization(int maxTime, int maxEvaluations)
{
    initializePheromone();

    Solution globalBest; 
    double bestObj = numeric_limits<double>::max();
    int iter=0;
    int lastImproved=0;
    auto t0 = chrono::steady_clock::now();
    int vehUsed  = static_cast<int>(globalBest.size());
    double bestDist = totalCost(globalBest);

    int  noImproveIter   = 0;                 // شمارندهٔ رکود
    const int ELITE_MAX  = 3;                 // حداکثر سایز استخر نخبه
    std::deque<Solution> eliteSolutions; 

            // if (logSummary.is_open()) {
            // logSummary << "iter="   << iter
            //         << " evaluation=" << evaluationCounter
            //         << " veh="   << vehUsed
            //         << " cost="  << fixed << setprecision(2) << bestDist
            //         << " bestObj=" << fixed << setprecision(2) << "0"
            //         << " q0=" << fixed << setprecision(3) << q0_dynamic
            //         << "\n";
            // }
    while(true){
        bool improvedThisIter = false; 
        // check stopping criteria
        auto now = chrono::steady_clock::now();
        if((maxTime>0 && chrono::duration_cast<chrono::seconds>(now-t0).count()>=maxTime) ||
           (maxEvaluations>0 && evaluationCounter>=maxEvaluations)) break;

        // ------ construct population ------
        vector<Solution> ants; 
        ants.reserve(POP_SIZE);
        for(int i=0; i<POP_SIZE && (maxEvaluations==0 || evaluationCounter<maxEvaluations); ++i){
            double q0 = (i == 0) ? 0.0 : q0_dynamic; // مورچه‌ی اول فقط اکتشاف می‌کنه
            Solution s = constructAntSolution(q0);
            ants.push_back(s);
            double obj = objective(s); 
            if(obj < bestObj){ 
                bestObj=obj; 
                globalBest=s; 
                lastImproved=iter;
                improvedThisIter = true;}

        }
                updateQ0(improvedThisIter);

                /*── شمارندهٔ رکود و استخر نخبه ──*/
        if (improvedThisIter) {
            noImproveIter = 0;
            // اضافه کردن بهترین جدید به استخر نخبه
            eliteSolutions.push_front(globalBest);
            if ((int)eliteSolutions.size() > ELITE_MAX)
                eliteSolutions.pop_back();
        } else {
            ++noImproveIter;
        }

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
        updatePheromone(ants, globalBest, iter, bestObj);


        // ------ stagnation check & reset ------
        if (noImproveIter >= STAG_LIM) {
            resetPheromone(globalBest,
                                   vector<Solution>(eliteSolutions.begin(),
                                                    eliteSolutions.end()),
                                   noImproveIter);
            noImproveIter = 0;          // شمارنده ریست می‌شود
            if (logPheromone.is_open())
                logPheromone << "<ADAPTIVE RESET>\n";
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