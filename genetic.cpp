#include <bits/stdc++.h>
using namespace std;

struct Customer {
    int id;
    double x, y;
    int demand;
    int readyTime;
    int dueTime;
    int serviceTime;
};

// A solution is a set of routes, each route is a sequence of customer indices starting and ending at 0
using Solution = vector<vector<int>>;

vector<Customer> customers;
vector<vector<double>> dist;
int vehicleCount, vehicleCapacity, numCustomers;
mt19937 rng(time(nullptr));

// Genetic parameters
int POP_SIZE = 50;
double CROSSOVER_RATE = 0.8;
double MUTATION_RATE = 0.2;
int MAX_GENERATIONS = 500;

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

// Objective: minimize vehicles then distance
double objective(const Solution &sol) {
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

// Create a random solution: simple greedy insert
Solution randomSolution() { 
    vector<bool> used(numCustomers,false);
    used[0]=true;
    Solution sol;
    for (int i=1;i<numCustomers;i++) {
        int cust = i;
        bool inserted=false;
        for (auto &route:sol) {
            for (int pos=1;pos<route.size();pos++) {
                auto tmp=route; tmp.insert(tmp.begin()+pos,cust);
                if (validRoute(tmp)) { route=tmp; inserted=true; break; }
            }
            if (inserted) break;
        }
        if (!inserted) sol.push_back({0,cust,0});
    }
    return sol;
}

// Population initialization
vector<Solution> initPopulation() {
    vector<Solution> pop;
    for (int i=0;i<POP_SIZE;i++) pop.push_back(randomSolution());
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
    return best;
}

// Crossover: route exchange
pair<Solution,Solution> crossover(const Solution &a, const Solution &b) {
    // pick random split point of routes
    int r1 = uniform_int_distribution<int>(1,a.size()-1)(rng);
    int r2 = uniform_int_distribution<int>(1,b.size()-1)(rng);
    Solution c1, c2;
    // child1: first r1 routes from a, fill rest from b
    set<int> used;
    for (int i=0;i<r1;i++) { c1.push_back(a[i]); for(int j: a[i]) used.insert(j);}    
    for (auto &r: b) {
        vector<int> nr;
        for (int j: r) if (!used.count(j)) nr.push_back(j);
        if (nr.size()>1) { nr.insert(nr.begin(),0); nr.push_back(0); c1.push_back(nr); }
    }
    // child2 similarly
    used.clear();
    for (int i=0;i<r2;i++) { c2.push_back(b[i]); for(int j: b[i]) used.insert(j);}    
    for (auto &r: a) {
        vector<int> nr;
        for (int j: r) if (!used.count(j)) nr.push_back(j);
        if (nr.size()>1) { nr.insert(nr.begin(),0); nr.push_back(0); c2.push_back(nr); }
    }
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
        }
    }
}

// Genetic Algorithm main
Solution geneticAlgorithm() {
    auto population = initPopulation();
    Solution best = population[0];
    double bestFit = objective(best);

    for (int gen=0; gen<MAX_GENERATIONS; gen++) {
        vector<Solution> newPop;
        while (newPop.size() < population.size()) {
            Solution p1 = tournament(population);
            Solution p2 = tournament(population);
            if (uniform_real_distribution<>(0,1)(rng) < CROSSOVER_RATE) {
                auto [c1,c2] = crossover(p1,p2);
                mutate(c1);
                mutate(c2);
                newPop.push_back(c1);
                if (newPop.size()<population.size()) newPop.push_back(c2);
            } else {
                mutate(p1);
                mutate(p2);
                newPop.push_back(p1);
                if (newPop.size()<population.size()) newPop.push_back(p2);
            }
        }
        population.swap(newPop);
        for (auto &sol: population) {
            if (!isFeasible(sol)) continue;
            double f = objective(sol);
            if (f < bestFit) { bestFit = f; best = sol; }
        }
        // optional: print progress
        if (gen % 50 == 0) cout << "Gen "<<gen<<" best="<<bestFit<<"\n";
    }
    return best;
}

// Output solution
void outputSolution(const Solution &sol, const string &fname) {
    string base = fname;
    auto p = base.find_last_of('.');
    if(p!=string::npos) base=base.substr(0,p);
    string out = base + "_output.txt";

    ofstream f(out);
    int veh=0; double distSum=0;
    for(auto &r:sol) if(r.size()>2){
        veh++; 
        distSum += routeCost(r);
        f<<"Route "<<veh<<":";
        for(int i=1;i+1<r.size();i++) f<<" "<<r[i];
        f<<"\n";
    }
    f<<"Vehicles: "<<veh<<"\n";
    f<<"Distance: "<<fixed<<setprecision(2)<<distSum<<"\n";
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] 
             << " [instance-file] [max-time-sec] [max-evaluations]\n";
        return 1;
    }
    string file          = argv[1];
    int max_time     = atoi(argv[2]);   
    int max_evaluations  = atoi(argv[3]); 

    readInstance(file);
    // buildDistanceMatrix();
    auto t0 = chrono::steady_clock::now();

    auto best = geneticAlgorithm();

    auto t1 = chrono::steady_clock::now();

    outputSolution(best, file);


    cout << "\n========== Execution Summary ==========\n";
    cout << " Total Runtime: " 
         << chrono::duration_cast<chrono::seconds>(t1-t0).count() 
         << " seconds\n";
    cout << " Max Time Allowed: " << max_time << " seconds\n";
    cout << " Max Evaluations Allowed: " << max_evaluations << "\n";

    // if (validateSolution(bestFeasibleSolution)) {
    //     cout << "Solution is valid and feasible.\n";
    // } else {
    //     cout << "Solution is NOT valid!\n";
    // }
    return 0;
}
