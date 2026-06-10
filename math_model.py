import gurobipy as gp
from gurobipy import GRB
import numpy as np
import re

# ============================================================
# 1. VERİ OKUMA
# ============================================================

def parse_instance(filepath):
    instance = {'depot': None, 'customers': [], 'stations': [],
                 'Q': None, 'C': None, 'r': None, 'g': None, 'v': None}
    with open(filepath, 'r') as f:
        lines = f.readlines()
    in_table = False
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith('StringID'):
            in_table = True
            continue
        m = re.match(r'^([QCrgv])\s+\w.*?/([\d.]+)/', line)
        if m:
            instance[m.group(1)] = float(m.group(2))
            continue
        if in_table:
            parts = line.split()
            if len(parts) < 8:
                continue
            node = {
                'id':           parts[0],
                'type':         parts[1],
                'x':            float(parts[2]),
                'y':            float(parts[3]),
                'demand':       float(parts[4]),
                'ready_time':   float(parts[5]),
                'due_date':     float(parts[6]),
                'service_time': float(parts[7]),
            }
            if node['type'] == 'd':
                instance['depot'] = node
            elif node['type'] == 'f':
                instance['stations'].append(node)
            elif node['type'] == 'c':
                instance['customers'].append(node)
    return instance


def build_nodes(instance):
    """
    Node index sırası:
      0                    : depot origin
      1 .. n_c             : customers
      n_c+1 .. n_c+n_f    : stations
      n_c+n_f+1           : depot destination
    """
    depot    = instance['depot']
    custs    = instance['customers']
    stations = instance['stations']
    n_c = len(custs)
    n_f = len(stations)
    depot_dest = {**depot, 'id': depot['id'] + '_dest'}
    nodes  = [depot] + custs + stations + [depot_dest]
    orig   = 0
    dest   = n_c + n_f + 1
    C_idx  = list(range(1, n_c + 1))
    F_idx  = list(range(n_c + 1, n_c + n_f + 1))
    demand = {idx: nodes[idx]['demand'] for idx in range(len(nodes))}
    return nodes, orig, dest, C_idx, F_idx, demand


def compute_dist(nodes):
    n = len(nodes)
    d = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            dx = nodes[i]['x'] - nodes[j]['x']
            dy = nodes[i]['y'] - nodes[j]['y']
            d[i][j] = round(np.sqrt(dx**2 + dy**2), 6)
    return d


# ============================================================
# 2. GUROBI MODELİ
# ============================================================

def solve_evrptw(filepath, n_vehicles=None, time_limit=300, verbose=True):

    # --- Veri ---
    inst = parse_instance(filepath)
    nodes, orig, dest, C_idx, F_idx, demand = build_nodes(inst)
    dist = compute_dist(nodes)

    Q = inst['Q']   # batarya kapasitesi
    C = inst['C']   # yük kapasitesi
    r = inst['r']   # tüketim oranı (enerji/mesafe)
    g = inst['g']   # şarj hızı (süre/enerji)
    v = inst['v']   # hız

    t = dist / v    # seyahat süresi matrisi

    n_c = len(C_idx)
    if n_vehicles is None:
        n_vehicles = n_c
    K = list(range(n_vehicles))

    all_nodes = list(range(len(nodes)))
    C_plus_F  = C_idx + F_idx

    # Arc seti:
    #   - j != orig  : depot origin'e giriş yok
    #   - i != dest  : depot destination'dan çıkış yok
    #   - i != j     : self-loop yok
    A = [(i, j) for i in all_nodes for j in all_nodes
         if i != j and j != orig and i != dest]

    # Kısıtlarda güvenli erişim için önceden gruplandırılmış arc listeleri
    # out_arcs[(i,k)] = i'den k aracıyla gidilebilecek j listesi
    # in_arcs[(j,k)]  = j'ye k aracıyla gelebilecek i listesi
    out_arcs = {(i, k): [] for i in all_nodes for k in K}
    in_arcs  = {(j, k): [] for j in all_nodes for k in K}
    for (i, j) in A:
        for k in K:
            out_arcs[(i, k)].append(j)
            in_arcs[(j, k)].append(i)

    M_big = 1e5
    Tmax  = 270.0   # 4.5 saat = 270 dk (EU mevzuatı)
    Trest = 45.0    # 45 dk zorunlu mola

    # --------------------------------------------------------
    model = gp.Model("E-VRPTW-PR-DR")
    if not verbose:
        model.setParam('OutputFlag', 0)
    model.setParam('TimeLimit', time_limit)

    # --- Karar Değişkenleri ---
    x     = model.addVars(A, K,         vtype=GRB.BINARY,     name="x")
    b     = model.addVars(F_idx, K,     vtype=GRB.BINARY,     name="b")
    tau   = model.addVars(all_nodes, K, lb=0, vtype=GRB.CONTINUOUS, name="tau")
    u     = model.addVars(all_nodes, K, lb=0, ub=C, vtype=GRB.CONTINUOUS, name="u")
    y     = model.addVars(all_nodes, K, lb=0, ub=Q, vtype=GRB.CONTINUOUS, name="y")
    y_hat = model.addVars(F_idx, K,     lb=0, ub=Q, vtype=GRB.CONTINUOUS, name="y_hat")
    D     = model.addVars(all_nodes, K, lb=0, ub=Tmax, vtype=GRB.CONTINUOUS, name="D")

    # --- Amaç Fonksiyonu ---
    # minimize: 100 * (araç sayısı) + toplam mesafe
    model.setObjective(
        100 * gp.quicksum(x[orig, j, k]
                          for k in K for j in out_arcs[(orig, k)])
        + gp.quicksum(dist[i][j] * x[i, j, k]
                      for (i, j) in A for k in K),
        GRB.MINIMIZE
    )

    # --------------------------------------------------------
    # KISITLAR
    # --------------------------------------------------------

    # (2) Her müşteri tam 1 kez ziyaret edilmeli
    for ci in C_idx:
        model.addConstr(
            gp.quicksum(x[ci, j, k]
                        for k in K for j in out_arcs[(ci, k)]) == 1,
            name=f"visit_{ci}"
        )

   
    # (4) Akış dengesi: her node için araç bazında giren = çıkan
    for k in K:
        for i in C_plus_F:
            in_flow  = gp.quicksum(x[j, i, k] for j in in_arcs[(i, k)])
            out_flow = gp.quicksum(x[i, j, k] for j in out_arcs[(i, k)])
            model.addConstr(in_flow == out_flow, name=f"flow_{i}_{k}")

    # (4b) Her araç depot'tan en fazla 1 kez çıkar
    for k in K:
        model.addConstr(
            gp.quicksum(x[orig, j, k] for j in out_arcs[(orig, k)]) <= 1,
            name=f"depart_{k}"
        )

    # (5) Yük takibi: müşteriye varışta yük azalır
    for (i, j) in A:
        if j in C_idx:
            for k in K:
                model.addConstr(
                    u[j, k] <= u[i, k] - demand[j] * x[i, j, k]
                               + C * (1 - x[i, j, k]),
                    name=f"load_{i}_{j}_{k}"
                )

    # (6) Başlangıç yükü = tam kapasite
    for k in K:
        model.addConstr(u[orig, k] == C, name=f"u0_{k}")

    # (7) Batarya: müşteri veya depot'tan gidince azalır
    for (i, j) in A:
        if i in C_idx + [orig]:
            for k in K:
                model.addConstr(
                    y[j, k] <= y[i, k] - r * dist[i][j] * x[i, j, k]
                               + Q * (1 - x[i, j, k]),
                    name=f"batt_c_{i}_{j}_{k}"
                )

    # (8) Batarya: istasyondan çıkınca y_hat kullanılır
    for (i, j) in A:
        if i in F_idx:
            for k in K:
                model.addConstr(
                    y[j, k] <= y_hat[i, k] - r * dist[i][j] * x[i, j, k]
                               + Q * (1 - x[i, j, k]),
                    name=f"batt_f_{i}_{j}_{k}"
                )

    # (9) İstasyonda şarj sınırları: gelişten ayrılışa artabilir, max Q
    for fi in F_idx:
        for k in K:
            model.addConstr(y[fi, k] <= y_hat[fi, k], name=f"charge_lb_{fi}_{k}")
            model.addConstr(y_hat[fi, k] <= Q,         name=f"charge_ub_{fi}_{k}")

    # (10) Başlangıç bataryası = tam
    for k in K:
        model.addConstr(y[orig, k] == Q, name=f"y0_{k}")

    # (11) Zaman güncelleme: müşteri veya depot'tan
    for (i, j) in A:
        if i in C_idx + [orig]:
            for k in K:
                model.addConstr(
                    tau[j, k] >= tau[i, k] + nodes[i]['service_time'] + t[i][j]
                                 - M_big * (1 - x[i, j, k]),
                    name=f"time_c_{i}_{j}_{k}"
                )

    # (12) Zaman güncelleme: istasyondan (şarj süresi + opsiyonel rest)
    for (i, j) in A:
        if i in F_idx:
            for k in K:
                model.addConstr(
                    tau[j, k] >= tau[i, k] + g * (y_hat[i, k] - y[i, k])
                                 + Trest * b[i, k] + t[i][j]
                                 - M_big * (1 - x[i, j, k]),
                    name=f"time_f_{i}_{j}_{k}"
                )

    # (13) Zaman pencereleri
    for i in all_nodes:
        for k in K:
            model.addConstr(tau[i, k] >= nodes[i]['ready_time'],
                            name=f"tw_lb_{i}_{k}")
            model.addConstr(tau[i, k] <= nodes[i]['due_date'],
                            name=f"tw_ub_{i}_{k}")

    # (14) Sürüş süresi: istasyondan, rest alındıysa birikmiş süre sıfırlanır
    for (i, j) in A:
        if i in F_idx:
            for k in K:
                model.addConstr(
                    D[j, k] <= D[i, k] + t[i][j]
                               + M_big * (1 - x[i, j, k]) + Tmax * b[i, k],
                    name=f"drive14_{i}_{j}_{k}"
                )

    # (15) Sürüş süresi: rest alındıysa sıfırdan başlar
    for (i, j) in A:
        if i in F_idx:
            for k in K:
                model.addConstr(
                    D[j, k] <= t[i][j]
                               + M_big * (1 - x[i, j, k]) + Tmax * (1 - b[i, k]),
                    name=f"drive15_{i}_{j}_{k}"
                )

    # (16) Sürüş süresi: müşteri veya depot'tan birikir
    for (i, j) in A:
        if i in C_idx + [orig]:
            for k in K:
                model.addConstr(
                    D[j, k] <= D[i, k] + t[i][j] + M_big * (1 - x[i, j, k]),
                    name=f"drive16_{i}_{j}_{k}"
                )

    # Başlangıç sürüş süresi = 0
    for k in K:
        model.addConstr(D[orig, k] == 0, name=f"D0_{k}")

    # (17) Rest sadece uğranılan istasyonda alınabilir
    for fi in F_idx:
        for k in K:
            model.addConstr(
                b[fi, k] <= gp.quicksum(x[fi, j, k] for j in out_arcs[(fi, k)]),
                name=f"rest_only_{fi}_{k}"
            )

    # --------------------------------------------------------
    model.optimize()

    # ============================================================
    # 3. SONUÇLAR
    # ============================================================
    if model.status in [GRB.OPTIMAL, GRB.TIME_LIMIT]:
        print(f"\n{'='*58}")
        print(f"Dosya     : {filepath}")
        print(f"Status    : {'OPTIMAL' if model.status == GRB.OPTIMAL else 'TIME LIMIT'}")
        print(f"Objective : {model.ObjVal:.4f}")

        used_vehicles = 0
        for k in K:
            route   = []
            current = orig
            visited = set([orig])
            while True:
                moved = False
                for j in out_arcs[(current, k)]:
                    if j not in visited and x[current, j, k].X > 0.5:
                        route.append(j)
                        visited.add(j)
                        current = j
                        moved = True
                        break
                if not moved:
                    break

            if route:
                used_vehicles += 1
                route_ids = [nodes[n]['id'] for n in route]
                print(f"\nVehicle {k}: {nodes[orig]['id']} -> {' -> '.join(route_ids)}")
                print(f"  {'Node':<12} {'tau':>7} {'y':>7} {'y_hat':>7} "
                      f"{'u':>7} {'D':>7} {'b':>4}")
                print(f"  {'-'*55}")
                for n in [orig] + route:
                    yh = f"{y_hat[n,k].X:7.2f}" if n in F_idx else "      -"
                    bv = f"{int(b[n,k].X)}"     if n in F_idx else "-"
                    print(f"  {nodes[n]['id']:<12} {tau[n,k].X:7.1f} "
                          f"{y[n,k].X:7.2f} {yh} {u[n,k].X:7.1f} "
                          f"{D[n,k].X:7.1f} {bv:>4}")

        print(f"\nKullanılan araç sayısı: {used_vehicles}")
    else:
        print(f"Çözüm bulunamadı. Status: {model.status}")

    return model


# ============================================================
# ÇALIŞTIR
# ============================================================
if __name__ == '__main__':
    model = solve_evrptw('c101C5.txt')