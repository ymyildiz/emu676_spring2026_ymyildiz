# Electric Vehicle Routing Problem with Time Windows, Partial Recharging and Driver Rest (E-VRPTW-PR-DR) Project
This repository is specified for the Term Project in the scope of EMU 676 Optimization Models and Algorithms in Transportation and Distribution, Spring 2026, and contains the implementation files for the **(E-VRPTW-PR-DR)**.

The project includes two solution approaches:

1. **MILP / MIP exact model** implemented in Python by Gurobi
2. **Hybrid ALNS + DP metaheuristic** implemented in Python

The objective is to minimize the number of electric vehicles and the total travel distance while satisfying customer time windows, vehicle capacity, battery capacity, partial recharging, and driver rest constraints.

---

## Repository Structure

```text
.
├── math_model.py        # MILP model to be solved by Gurobi
├── heuristic.py         # Hybrid ALNS + DP heuristic
├── c101C5.txt           # Example small instance
├── c106C15.txt          # Example larger instance
└── README.md
```

Additional `.txt` instance files can also be placed in the same folder.

---

## Required Software and Packages

The codes are written in Python. The following Python packages are required:

```text
numpy
gurobipy
```

To run `math_model.py`, Gurobi must be installed and a valid Gurobi license must be available.

---

## Installation

First, clone or download this repository.
Then install the required Python packages.
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

 Example 1: Solve Small Instance with MILP

 Example 2: Solve Larger Instance with MILP
 (For larger instances, the MILP model may take longer or may not prove optimality within the time limit.)

 Example 3: Solve Small Instance with ALNS

 Example 4: Solve Larger Instance with ALNS


---


# 4. Contact / Project Information

This project was developed for:

```text
EMU676 Optimization Models and Algorithms in Transportation and Distribution
```

by:

```text
Yiğit Muzaffer YILDIZ
```

Project topic:

```text
Electric Vehicle Routing Problem with Time Windows,
Partial Recharging and Driver Rest Constraints
```
