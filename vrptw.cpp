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
    
}; 
        int main(int argc, char* argv[]) {
            if (argc != 4) {
                cerr << "Usage: " << argv[0] << " [instance-file-path] [Max-execution-time-seconds] [Max-evaluation-number]\n";
                return 1;
            }
        
            VRPTWSolver solver;
            solver.readInstance(argv[1]);
            return 0;
        }