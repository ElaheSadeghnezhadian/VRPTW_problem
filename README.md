# VRPTW_problem
This repository contains the implementation and experiments related to the Vehicle Routing Problem with Time Windows (VRPTW). The VRPTW is a classic optimization problem in logistics where the objective is to find optimal routes for a fleet of vehicles to service a set of customers within specific time windows

A **feasible route**:
- Starts and ends at depot **𝑣₀**
- Must not exceed vehicle capacity **Q**
- Must respect customers’ time windows
- Can include waiting times
- Must complete within total route duration **D**

The objective is to construct up to **m** vehicle routes that visit each customer **exactly once**, within their time window, **minimizing total distance**.

---

## 🚀 Implemented Algorithms

- **Tabu Search (tabu)**
- **Simulated Annealing (SA)**  
(More metaheuristics may be added in future versions)

---
✅ Requirements

C++17 compatible compiler (e.g., g++, MSVC)
No external dependencies
---

## 🛠️ How to Run
The executable expects the following command-line arguments:
```terminal
[instance-file-path] [max-execution-time-in-seconds] [max-objective-function-evaluation-number]
```
📌 Example:
```terminal
.\tabu.exe Instances\800-rch-63.txt 300 0
```
800-rch-63.txt is the input instance
300 means the algorithm runs for 300 seconds max
0 means there's no limit on the number of objective function evaluations

