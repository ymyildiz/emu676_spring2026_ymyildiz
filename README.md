# Electric Vehicle Routing Problem with Time Windows, Partial Recharging and Driver Rest (E-VRPTW-PR-DR)
This repository is specified for the Term Project in the scope of EMU 676 Optimization Models and Algorithms in Transportation and Distribution, Spring 2026.
# E-VRPTW-PR-DR Project

This repository contains the implementation files for the **Electric Vehicle Routing Problem with Time Windows, Partial Recharging and Driver Rest Constraints (E-VRPTW-PR-DR)**.

The project includes two solution approaches:

1. **MILP / MIP exact model** implemented with Gurobi
2. **Hybrid ALNS + DP metaheuristic** implemented in Python

The objective is to minimize the number of electric vehicles and the total travel distance while satisfying customer time windows, vehicle capacity, battery capacity, partial recharging, and driver rest constraints.

---

## Repository Structure

```text
.
├── math_model.py        # MILP model solved with Gurobi
├── heuristic.py         # Hybrid ALNS + DP heuristic
├── c101C5.txt           # Example small instance
├── c106C15.txt          # Example larger instance
└── README.md
```

Additional `.txt` instance files can also be placed in the same folder.

---

## Required Software and Packages

The codes are written in Python. The following packages are required:

```text
numpy
gurobipy
```

The heuristic code only requires:

```text
numpy
```

The exact mathematical model requires:

```text
gurobipy
```

Therefore, to run `math_model.py`, Gurobi must be installed and a valid Gurobi license must be available.

---

## Installation

First, clone or download this repository.

Then install the required Python packages:

```bash
pip install numpy
```

If you want to run the MILP model, also install Gurobi Python interface:

```bash
pip install gurobipy
```

Make sure that Gurobi is correctly installed and licensed before running the exact model.

---

## Input File Format

The input files must be in `.txt` format. Each file should include:

* depot node,
* recharging station nodes,
* customer nodes,
* vehicle battery capacity,
* vehicle load capacity,
* energy consumption rate,
* inverse recharging rate,
* average speed.

The node table should have the following structure:

```text
StringID   Type       x          y          demand     ReadyTime  DueDate    ServiceTime
D0         d          ...
S0         f          ...
C1         c          ...
```

The meaning of the node types is:

```text
d : depot
f : recharging station
c : customer
```

At the end of the file, vehicle and energy parameters should be given in the following format:

```text
Q Vehicle fuel tank capacity /77.75/
C Vehicle load capacity /200.0/
r fuel consumption rate /1.0/
g inverse refueling rate /3.47/
v average Velocity /1.0/
```

---

# 1. Running the MILP Model

The MILP model is implemented in:

```text
math_model.py
```

This file builds and solves the exact mathematical model using Gurobi.

## How to Choose the Input `.txt` File

In the current version, the instance file name is written at the bottom of `math_model.py`.

Open `math_model.py` and find the following part:

```python
if __name__ == '__main__':
    model = solve_evrptw('c101C5.txt')
```

To solve another instance, replace `c101C5.txt` with the name of your own input file.

For example, to solve `c106C15.txt`, change the line as follows:

```python
if __name__ == '__main__':
    model = solve_evrptw('c106C15.txt')
```

The `.txt` file must be in the same folder as `math_model.py`.

## Running the MILP Model

After selecting the input file, run:

```bash
python math_model.py
```

## Optional Parameters

The main function is:

```python
solve_evrptw(filepath, n_vehicles=None, time_limit=300, verbose=True)
```

The parameters are:

| Parameter    | Description                                                                         |
| ------------ | ----------------------------------------------------------------------------------- |
| `filepath`   | Name or path of the input `.txt` file                                               |
| `n_vehicles` | Number of available vehicles. If `None`, it is set equal to the number of customers |
| `time_limit` | Gurobi time limit in seconds                                                        |
| `verbose`    | If `True`, Gurobi output is shown                                                   |

For example, if you want to solve `c101C5.txt` with at most 3 vehicles and a 600-second time limit, you can write:

```python
if __name__ == '__main__':
    model = solve_evrptw(
        filepath='c101C5.txt',
        n_vehicles=3,
        time_limit=600,
        verbose=True
    )
```

## MILP Output

The MILP code prints:

* input file name,
* solver status,
* objective value,
* vehicle routes,
* arrival times,
* battery levels,
* recharging decisions,
* load values,
* driving time values,
* number of used vehicles.

Example output structure:

```text
Dosya     : c101C5.txt
Status    : OPTIMAL
Objective : 457.7475

Vehicle 1: D0 -> C12 -> S5 -> C100 -> S0 -> D0_dest
...
Kullanılan araç sayısı: 2
```

---

# 2. Running the Hybrid ALNS + DP Heuristic

The heuristic algorithm is implemented in:

```text
heuristic.py
```

This code uses a hybrid structure:

* **ALNS** determines customer sequences.
* **DP route evaluator** inserts charging stations, partial recharge amounts, and driver rest decisions when needed.

## Default Run

If no input file is given, the code automatically uses:

```text
c101C5.txt
```

Run:

```bash
python heuristic.py
```

This is equivalent to running:

```bash
python heuristic.py c101C5.txt
```

## Running with a Different Instance

To run the heuristic with another input file, write the file name after the Python file name.

For example:

```bash
python heuristic.py c106C15.txt
```

or:

```bash
python heuristic.py c101_21.txt
```

The `.txt` file must be in the same folder as `heuristic.py`, unless a full or relative path is provided.

For example, if the instance is inside a folder named `data`, use:

```bash
python heuristic.py data/c106C15.txt
```

## Where the Input File Name Is Read

In `heuristic.py`, the input file is read from the command line using this part:

```python
filepath = sys.argv[1] if len(sys.argv) > 1 else 'c101C5.txt'
```

This means:

* If you run `python heuristic.py`, the file `c101C5.txt` is used.
* If you run `python heuristic.py c106C15.txt`, the file `c106C15.txt` is used.

---

## Heuristic Parameters

The ALNS parameters are defined near the bottom of `heuristic.py`:

```python
alns = ALNS(
    n_iter   = 500,
    n_remove = max(1, len(ctx.customers) // 4),
    T_init   = None,
    cooling  = 0.995,
    w_update = 50,
    sigma    = (10, 6, 3),
    decay    = 0.8,
    seed     = 42,
)
```

The parameters are:

| Parameter  | Description                                   |
| ---------- | --------------------------------------------- |
| `n_iter`   | Number of ALNS iterations                     |
| `n_remove` | Number of customers removed in each iteration |
| `T_init`   | Initial temperature for simulated annealing   |
| `cooling`  | Cooling rate                                  |
| `w_update` | Operator weight update interval               |
| `sigma`    | Operator reward scores                        |
| `decay`    | Weight update decay factor                    |
| `seed`     | Random seed                                   |

---

## Changing the Number of Iterations

To increase or decrease the number of ALNS iterations, change:

```python
n_iter = 500
```

For example:

```python
n_iter = 1000
```

A higher number of iterations may improve solution quality, but it also increases computational time.

---

## Changing the Number of Removed Customers

The current setting is:

```python
n_remove = max(1, len(ctx.customers) // 4)
```

This means that approximately one fourth of the customers are removed in each ALNS iteration.

For example:

* if there are 5 customers, `n_remove = 1`;
* if there are 15 customers, `n_remove = 3`;
* if there are 100 customers, `n_remove = 25`.

You can also set this manually. For example:

```python
n_remove = 2
```

or:

```python
n_remove = 5
```

---

## Meaning of `T_init = None`

The parameter `T_init` is used in the simulated annealing acceptance rule.

In the current implementation:

```python
T = self.T_init if self.T_init else max(current.cost * 0.05, 1.0)
```

Therefore, when:

```python
T_init = None
```

the initial temperature is calculated automatically as:

```text
max(5% of initial solution cost, 1.0)
```

This makes the initial temperature depend on the size and cost of the instance.

If desired, `T_init` can be fixed manually. For example:

```python
T_init = 50
```

or:

```python
T_init = 100
```

---

## Heuristic Output

The heuristic code prints:

* instance information,
* number of customers,
* number of recharging stations,
* vehicle parameters,
* initial solution cost,
* iteration progress,
* final objective value,
* number of vehicles,
* total travel distance,
* detailed vehicle routes,
* arrival time,
* battery level,
* recharge amount,
* remaining load,
* continuous driving time,
* rest decision,
* final constraint check.

Example output structure:

```text
Instance : c106C15.txt
Musteri  : 15  Istasyon : 3  Q=77.75  C=200.0  r=1.0  g=3.47

Baslangic: cost=...
iter 100 | best=...
iter 200 | best=...

=================================================================
ALNS SONUCU
  Objective    : 617.6815
  Arac sayisi  : 3  (x100 = 300)
  Toplam mesafe: 317.6815
=================================================================

Vehicle 0: D0 -> C20 -> C27 -> C9 -> D0
...
Kisit Kontrolu:
  Tum kisitlar saglaniyor [OK]
```

---

# 3. Example Runs

## Example 1: Solve Small Instance with MILP

First, open `math_model.py` and make sure that the last line is:

```python
model = solve_evrptw('c101C5.txt')
```

Then run:

```bash
python math_model.py
```

## Example 2: Solve Larger Instance with MILP

Open `math_model.py` and change the last line to:

```python
model = solve_evrptw('c106C15.txt')
```

Then run:

```bash
python math_model.py
```

For larger instances, the MILP model may take longer or may not prove optimality within the time limit.

## Example 3: Solve Small Instance with ALNS

Run:

```bash
python heuristic.py c101C5.txt
```

## Example 4: Solve Larger Instance with ALNS

Run:

```bash
python heuristic.py c106C15.txt
```

---

# 4. Recommended Workflow

A recommended workflow is:

1. Run the MILP model on a small instance.
2. Use the MILP result as an optimal benchmark.
3. Run the ALNS heuristic on the same small instance.
4. Compare the ALNS result with the MILP optimum.
5. Run the ALNS heuristic on larger instances.
6. Report objective value, number of vehicles, total distance, CPU time, and solution feasibility.

For example:

```bash
python heuristic.py c101C5.txt
python heuristic.py c106C15.txt
```

---

# 5. Notes on Objective Function

Both codes use the following objective structure:

```text
Objective = 100 × number of vehicles + total travel distance
```

Therefore, the model first penalizes the use of additional vehicles and then considers the travelled distance.

For example, if a solution uses 3 vehicles and has total distance 317.6815, then:

```text
Objective = 100 × 3 + 317.6815 = 617.6815
```

---

# 6. Notes on Recharging Stations

The ALNS solution representation stores only customer sequences. Recharging stations are not fixed directly by ALNS.

Instead, the DP route evaluator decides:

* whether a station is needed,
* which station should be visited,
* how much energy should be charged,
* whether a driver rest break is required.

This allows the ALNS part to focus on customer ordering, while the DP part handles electric vehicle feasibility.

---

# 7. Troubleshooting

## File Not Found Error

If you get an error such as:

```text
FileNotFoundError: [Errno 2] No such file or directory
```

check that the `.txt` instance file is in the same folder as the Python file.

For example, this command:

```bash
python heuristic.py c106C15.txt
```

requires the following structure:

```text
.
├── heuristic.py
└── c106C15.txt
```

If your file is in a folder named `data`, use:

```bash
python heuristic.py data/c106C15.txt
```

For `math_model.py`, make sure the correct file name is written inside the code:

```python
model = solve_evrptw('data/c106C15.txt')
```

---

## Gurobi License Error

If `math_model.py` gives a Gurobi license error, check that:

* Gurobi is installed,
* `gurobipy` is installed,
* your Gurobi license is active.

You can test Gurobi with:

```bash
python -c "import gurobipy as gp; print(gp.gurobi.version())"
```

---

## No Feasible Solution Found

If the heuristic prints:

```text
Feasible cozum bulunamadi.
```

possible reasons are:

* the instance is too restrictive,
* the time windows are very tight,
* battery capacity is too low,
* the number of iterations is too small,
* the destroy/repair settings need tuning.

You may try increasing:

```python
n_iter = 1000
```

or changing:

```python
n_remove
```

---

# 8. Reproducibility

The random behavior of the ALNS algorithm is controlled by the `seed` parameter:

```python
seed = 42
```

Using the same seed should reproduce the same ALNS run, assuming the same Python version and input file.

To test different random solutions, change the seed value:

```python
seed = 1
seed = 2
seed = 3
```

---

# 9. Output Interpretation

Important values in the output are:

| Output field    | Meaning                             |
| --------------- | ----------------------------------- |
| `Objective`     | Total objective value               |
| `Arac sayisi`   | Number of used vehicles             |
| `Toplam mesafe` | Total travelled distance            |
| `tau`           | Arrival time                        |
| `y_arr`         | Battery level upon arrival          |
| `y_dep`         | Battery level upon departure        |
| `charge`        | Amount of energy recharged          |
| `load`          | Remaining vehicle load              |
| `D`             | Accumulated continuous driving time |
| `rest`          | Whether a rest break is taken       |

---

# 10. Contact / Project Information

This project was developed for:

```text
EMU676 Optimization Models and Algorithms in Transportation and Distribution
```

Project topic:

```text
Electric Vehicle Routing Problem with Time Windows,
Partial Recharging and Driver Rest Constraints
```
