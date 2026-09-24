# Dynamic Rolling-Horizon (MSRH) Optimization for 2E-VRP

This repository contains the architectural implementation of a Multi-Stage Rolling Horizon (MSRH) framework developed for complex urban last-mile delivery systems utilizing mothership vans and sidewalk autonomous delivery robots (SADRs)[cite: 2].

## Key Features
* **Iterative Dynamic Routing:** Implements an MILP model solved iteratively over consecutive temporal stages to handle continuous, dynamic customer demand[cite: 2].
* **Dynamic Backlog Mechanism:** Effectively manages capacity shortages by systematically carrying over unserved modular parcels to subsequent stages under an escalating penalty framework[cite: 2].
* **Exact MILP Modeling:** Formulates a stationary micro-hub policy ensuring strict spatial-temporal synchronization between the van and deployed robots[cite: 2]. Includes fundamental OR formulations such as flow conservation and MTZ subtour elimination.

*Note: To comply with academic publishing restrictions (e.g., EWGT 2026 proceedings), full real-world network datasets (Rotterdam topology) and proprietary van-robot synchronization constraints are abstracted[cite: 2]. This repository demonstrates mathematical modeling proficiency and `docplex` integration.*

## Tech Stack
- **Core Language:** Python 3.x
- **Mathematical Modeling:** IBM Decision Optimization (`docplex` / CPLEX)
- **Numerical Processing:** `numpy` (Parameter array management)
- **Data Manipulation & I/O:** `pandas`, `openpyxl`

Copyright (c) 2026 Mohammad Hamed Mahdavi. All Rights Reserved.
