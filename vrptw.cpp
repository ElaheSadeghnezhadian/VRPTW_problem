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