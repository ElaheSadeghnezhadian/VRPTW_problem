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
int POP_SIZE = 50;
double CROSSOVER_RATE = 0.8;
double MUTATION_RATE = 0.2;

// GIDEON parameters
int itermax = 3;
int BSize   = 5;
vector<double> polarAngle;
Solution lr, gr;
double lcost, gcost;

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
    for (size_t i=1; i<route.size(); ++i)  // ← use size_t
        c += dist[route[i-1]][route[i]];
    return c;
}

double totalCost(const vector<vector<int>>& sol) {
    double cost = 0.0;
    for (const auto& r : sol)
        cost += routeCost(r);
    return cost;
}

// Objective: minimize vehicles then distance
double objective(const Solution &sol) {
    evaluationCounter++;
    int used=0; double d=0;
    for (auto &r:sol) if (r.size()>2) { used++; d+=routeCost(r);}    
    return used*10000 + d;
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

    for (int v = 0; v < vehicleCount; ++v) {
        // اگر همه مشتری‌ها تخصیص یافته‌اند، از حلقه خارج شو
        bool allUsed = true;
        for (int i = 1; i < numCustomers; ++i) if (!used[i]) { allUsed = false; break; }
        if (allUsed) break;

        vector<int> route = {0};
        int load = 0;
        double time = 0;
        int current = 0;

        while (true) {
            vector<int> candidates;
            for (int i = 1; i < numCustomers; ++i) {
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
                int idx = uniform_int_distribution<int>(0, candidates.size()-1)(rng);
                next = candidates[idx];
                logEvents << "  [RANDOM] picks " << next << "\n";
            } else {
                next = candidates[0];
                double best = dist[current][next];
                for (int c : candidates)
                    if (dist[current][c] < best) { best = dist[current][c]; next = c; }
                logEvents << "  [GREEDY] picks " << next << "\n";
            }

            route.push_back(next);
            used[next] = true;
            time += dist[current][next];
            time = max(time, (double)customers[next].readyTime) + customers[next].serviceTime;
            load += customers[next].demand;
            current = next;
        }

        // فقط اگر حداقل یک مشتری در مسیر هست، صفر انتهایی اضافه و نگه‌دار
        if (route.size() > 1) {
            route.push_back(0);
            sol.push_back(route);
        }
    }

    // اگر باقی‌مانده تک مشتری داریم، هرکدام را به‌صورت یک مسیر نگه‌دار
    for (int i = 1; i < numCustomers; ++i) {
        if (!used[i]) {
            logEvents << "  Unassigned customer " << i << " -> single route\n";
            sol.push_back({0, i, 0});
        }
    }

    logSolution(sol, counter++);
    return sol;
}

// Population initialization
vector<Solution> initPopulation() {
    logEvents << "Initializing population...\n";
    vector<Solution> pop;
    for (int i=0;i<POP_SIZE;i++) pop.push_back(randomSolution());
    logEvents << "Initial population of "<<POP_SIZE<<" solutions generated.\n";
    return pop;
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

// Crossover: route exchange
pair<Solution,Solution> crossover(const Solution &a, const Solution &b) {
    int r1 = uniform_int_distribution<int>(1,a.size()-1)(rng);
    int r2 = uniform_int_distribution<int>(1,b.size()-1)(rng);
    logEvents << "Crossover points: a->"<<r1<<" routes, b->"<<r2<<" routes\n";
    Solution c1, c2;
    set<int> used;
    for (int i=0;i<r1;i++) { c1.push_back(a[i]); for(int j: a[i]) used.insert(j);}    
    for (auto &r: b) {
        vector<int> nr;
        for (int j: r) if (!used.count(j)) nr.push_back(j);
        if (nr.size()>1) { nr.insert(nr.begin(),0); nr.push_back(0); c1.push_back(nr); }
    }
    used.clear();
    for (int i=0;i<r2;i++) { c2.push_back(b[i]); for(int j: b[i]) used.insert(j);}    
    for (auto &r: a) {
        vector<int> nr;
        for (int j: r) if (!used.count(j)) nr.push_back(j);
        if (nr.size()>1) { nr.insert(nr.begin(),0); nr.push_back(0); c2.push_back(nr); }
    }
    auto clean = [&](Solution &x){
        Solution tmp;
        for (auto &r : x)
            if (r.size() > 2) tmp.push_back(r);
        x.swap(tmp);
    };
    clean(c1);
    clean(c2);

    return {c1,c2};
}

// Mutation: swap two customers
void mutate(Solution &sol) {
    for (auto &route: sol) {
        if (route.size()>3 && uniform_real_distribution<>(0,1)(rng)<MUTATION_RATE) {
            int i = uniform_int_distribution<int>(1,route.size()-2)(rng);
            int j = uniform_int_distribution<int>(1,route.size()-2)(rng);
            swap(route[i], route[j]);
            if (!validRoute(route)) swap(route[i], route[j]);
            else logEvents << "Mutated route by swapping pos "<<i<<" and "<<j<<"\n";
        }
    }
}

// === GIDEON helpers ===
double decodeSeed(const Solution &chrom, int b){
    // simply spread seeds evenly
    const double PI = acos(-1.0);
    return b * (2*PI / BSize);
}

vector<vector<int>> sectorCustomers(double seed){
    // rotate angles, then bucket into BSize sectors
    vector<vector<int>> sec(BSize);
    for(int i=1;i<numCustomers;i++){
        double a = polarAngle[i] - seed;
        if(a<0) a+=2*M_PI;
        int s = int(a / (2*M_PI/BSize)) % BSize;
        sec[s].push_back(i);
    }
    return sec;
}

Solution cheapestInsertion(const vector<vector<int>>& sectors){
    Solution sol;
    for(auto &sec:sectors){
        vector<int> route={0,0}; // start+end
        for(int c:sec){
            // find best place to insert c
            double bestInc=1e18; int bestPos=1;
            for(int p=1;p<route.size();p++){
                double inc = dist[route[p-1]][c]+dist[c][route[p]] 
                             - dist[route[p-1]][route[p]];
                if(inc<bestInc){bestInc=inc; bestPos=p;}
            }
            route.insert(route.begin()+bestPos, c);
        }
        if(route.size()>2) sol.push_back(route);
    }
    return sol;
}

vector<int> twoOptImprove(const vector<int>& route){
    auto r=route; int n=r.size();
    bool imp=true;
    while(imp){
        imp=false;
        for(int i=1;i<n-2 && !imp;i++)
         for(int j=i+1;j<n-1 && !imp;j++){
            double before=dist[r[i-1]][r[i]]+dist[r[j]][r[j+1]];
            double after =dist[r[i-1]][r[j]]+dist[r[i]][r[j+1]];
            if(after+1e-6<before){
                reverse(r.begin()+i, r.begin()+j+1);
                imp=true;
            }
         }
    }
    return r;
}

void localPostOptimize(Solution &sol,double &cost){
    bool imp=true;
    while(imp){
        imp=false;
        for(auto &r:sol){
            auto r2=twoOptImprove(r);
            if(routeCost(r2)+1e-6<routeCost(r)){
                r=r2; imp=true;
            }
        }
    }
    cost=totalCost(sol);
    if(cost<gcost){gcost=cost; gr=sol;}
}

void assignInitialPolar(){
    polarAngle.resize(numCustomers);
    for(int i=0;i<numCustomers;i++){
        double dx=customers[i].x-customers[0].x;
        double dy=customers[i].y-customers[0].y;
        polarAngle[i]=atan2(dy,dx);
    }
}

void resequencePolar(const Solution &lr){
    // ۱) بازنشانی rank
    vector<int> rank(numCustomers);
    int idx = 0;
    for (auto &r : lr) {
        for (size_t j = 1; j + 1 < r.size(); ++j) {
            rank[r[j]] = idx++;
        }
    }

    // ۲) تعریف PI
    const double PI = acos(-1.0);

    // ۳) حلقه با متغیر size_t
    for (size_t i = 0; i < polarAngle.size(); ++i) {
        polarAngle[i] = rank[i] * (2 * PI / numCustomers);
    }
}


// Genetic Algorithm main
Solution geneticAlgorithm(int max_time, int max_evaluations) {
    logGeneration << "Gen, BestObjective\n";
    auto population = initPopulation();
    Solution best = population[0];
    double bestFit = objective(best);
    logGeneration << 0 << "," << bestFit << "\n";

    auto t_start = chrono::steady_clock::now();
    int gen = 1;
    cout<<"[GA] start loop\n"; cout.flush();
    while (true) {
        // 1) چک زمان
        cout<<"[GA] Loop start gen="<<gen<<"\n"; cout.flush();
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

        // 3) یک نسل جدید بساز
            // --- GIDEON: sectoring hook (Step 4) ---
        for (auto &chrom : population) {
            for (int b = 0; b < BSize; ++b) {
                double seed = decodeSeed(chrom, b);
                auto sectors = sectorCustomers(seed);
                auto sol     = cheapestInsertion(sectors);
                double cost  = totalCost(sol);
                if (cost < lcost) { lcost = cost; lr = sol; }
                if (cost < gcost) { gcost = cost; gr = sol; }
            }
        }
        // --- end sectoring hook ---
        cout<<"[GA] Sectoring done at gen="<<gen<<"\n"; cout.flush();
        vector<Solution> newPop;
        while (newPop.size() < population.size()) {
            Solution p1 = tournament(population);
            Solution p2 = tournament(population);
            if (uniform_real_distribution<>(0,1)(rng) < CROSSOVER_RATE) {
                auto [c1,c2] = crossover(p1,p2);
                mutate(c1);
                mutate(c2);
                newPop.push_back(c1);
                if (newPop.size() < population.size()) newPop.push_back(c2);
            } else {
                mutate(p1);
                mutate(p2);
                newPop.push_back(p1);
                if (newPop.size() < population.size()) newPop.push_back(p2);
            }
        }
        population.swap(newPop);

        // 4) به‌روزرسانی بهترین سلوشن
        for (auto &sol : population) {
            if (!isFeasible(sol)) continue;
            double f = objective(sol);
            if (f < bestFit) {
                bestFit = f;
                best = sol;
            }
        }

        // 5) لاگ و افزایش شمارنده‌ی نسل
        logGeneration << gen << "," << bestFit << "\n";
        cout << "Gen " << gen << " best=" << bestFit << "\n";
        cout.flush();
        gen++;
        cout<<"[GA] New population ready at gen="<<gen<<"\n"; cout.flush();
    }
    cout<<"[GA] end loop\n"; cout.flush();
    return best;
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
    cout << "Starting GIDEON-GA...\n";
    // open logs
    logPopulation.open("population_log.txt");
    logGeneration.open("generation_log.csv");
    logEvents.open("events_log.txt");

    string file  = argv[1];
    int max_time = atoi(argv[2]);    
    int max_evaluations = atoi(argv[3]);

    readInstance(file);

    // GIDEON init:
    polarAngle.resize(numCustomers);
    lcost = gcost = numeric_limits<double>::infinity();
    assignInitialPolar();   // به مشتری‌ها زاویه اولیه بده
    lr.clear(); gr.clear();


    auto t0 = chrono::steady_clock::now();
    cout << "About to enter geneticAlgorithm()\n"; cout.flush();

    Solution best;
    for(int iter = 1; iter <= itermax; ++iter) {
        // ← cluster‑route iteration
        best = geneticAlgorithm(max_time, max_evaluations);
        // ← local post‑opt روی lr
        localPostOptimize(lr, lcost);
        // ← باز‌رتبه‌بندی قطبی برای دور بعد
        resequencePolar(lr);
    }
    // حالا بهترین global در gr است
    best = gr;

cout << "Returned from geneticAlgorithm()\n"; cout.flush();
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
