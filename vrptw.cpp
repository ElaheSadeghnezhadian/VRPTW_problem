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


int main() {
    string filename = "c:\\Users\\ela\\Desktop\\25-ch-3.txt"; // Fixed file path

    // Step 1: Read input data
    read_input(filename);
}