#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <unordered_set>
#include <cstdlib>


using namespace std;

// Data structure for a customer
struct Customer {
    int id;
    double x, y;       // Coordinates
    int demand;        // Demand amount
    int ready_time, due_date; // Time window
    int service_time;  // Service time
};

// Global variables
int num_vehicles, vehicle_capacity;
vector<Customer> customers;
vector<vector<double>> distance_matrix; // Matrix to store the distances

// Function to read data from the file
void read_input(const string &filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }

    string line;

    // Skip the first line (e.g., "100-ce-8")
    getline(infile, line);

    // Read VEHICLE section
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line); // Skip "NUMBER     CAPACITY"
            infile >> num_vehicles >> vehicle_capacity;
            break;
        }
    }

    // Read CUSTOMER section
    while (getline(infile, line)) {
        if (line.find("CUSTOMER") != string::npos) {
            getline(infile, line); // Skip "CUST NO.  XCOORD. ..."
            break;
        }
    }

    // Read customer data
    int id;
    double x, y;
    int demand, ready_time, due_date, service_time;

    while (infile >> id >> x >> y >> demand >> ready_time >> due_date >> service_time) {
        customers.push_back({id, x, y, demand, ready_time, due_date, service_time});
    }

    infile.close();
}

// Function to calculate Euclidean distance
double calculate_distance(const Customer &c1, const Customer &c2) {
    return sqrt(pow(c2.x - c1.x, 2) + pow(c2.y - c1.y, 2));
}

// Function to build the distance matrix
void build_distance_matrix() {
    int n = customers.size();
    distance_matrix.resize(n, vector<double>(n, 0));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i <= j) {
                distance_matrix[i][j] = calculate_distance(customers[i], customers[j]);
            }
        }
    }
}


int main() {
    string filename = "c:\\Users\\ela\\Desktop\\25-ch-3.txt"; // Fixed file path

    // Step 1: Read input data
    read_input(filename);

    // Step 2: Build distance matrix
    build_distance_matrix();
    
}