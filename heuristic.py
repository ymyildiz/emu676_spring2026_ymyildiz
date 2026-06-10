"""
E-VRPTW-PR-DR  —  Hybrid ALNS + DP
====================================
Katman 1  ALNS  : musteri rotalarini belirler
Katman 2  DP    : her rota icin kisitlari karsilayan en iyi
                  istasyon / sarj miktari / mola kararini bulur

Kisitlar (MILP ile birebir):
  (2)  Her musteri tam 1 kez ziyaret
  (5)(6)  Yuk kapasitesi
  (7)(8)(9)(10)  Batarya / partial recharge
  (11)(12)(13)  Zaman pencereleri
  (14)(15)(16)(17)  Suruş suresi / EU mola

Kullanim:
  python alns.py                  -> c101C5.txt
  python alns.py c101_21.txt
"""

import random, math, copy, sys, re
import numpy as np

INFEASIBLE_COST = 1e9

# ============================================================
# 1. VERI OKUMA
# ============================================================

def parse_instance(filepath):
    instance = {'depot': None, 'customers': [], 'stations': [],
                 'Q': None, 'C': None, 'r': None, 'g': None, 'v': None}
    with open(filepath, 'r') as f:
        lines = f.readlines()
    in_table = False
    for line in lines:
        line = line.strip()
        if not line: continue
        if line.startswith('StringID'): in_table = True; continue
        m = re.match(r'^([QCrgv])\s+\w.*?/([\d.]+)/', line)
        if m: instance[m.group(1)] = float(m.group(2)); continue
        if in_table:
            parts = line.split()
            if len(parts) < 8: continue
            node = {'id': parts[0], 'type': parts[1],
                    'x': float(parts[2]), 'y': float(parts[3]),
                    'demand': float(parts[4]), 'ready_time': float(parts[5]),
                    'due_date': float(parts[6]), 'service_time': float(parts[7])}
            if node['type'] == 'd': instance['depot'] = node
            elif node['type'] == 'f': instance['stations'].append(node)
            elif node['type'] == 'c': instance['customers'].append(node)
    return instance


def compute_dist(nodes):
    n = len(nodes)
    d = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            dx = nodes[i]['x'] - nodes[j]['x']
            dy = nodes[i]['y'] - nodes[j]['y']
            d[i][j] = round(math.sqrt(dx**2 + dy**2), 6)
    return d


# ============================================================
# 2. PROBLEM CONTEXT
# ============================================================

class ProblemContext:
    """
    Node index sirasi (all_nodes):
      0          : depot
      1..n_c     : customers
      n_c+1..    : stations
    """
    def __init__(self, filepath):
        inst = parse_instance(filepath)
        self.depot     = inst['depot']
        self.customers = inst['customers']
        self.stations  = inst['stations']
        self.Q    = inst['Q']
        self.C    = inst['C']
        self.r    = inst['r']
        self.g    = inst['g']
        self.v    = inst['v']
        self.Tmax  = 270.0
        self.Trest = 45.0
        self.all_nodes = [self.depot] + self.customers + self.stations
        self.dist      = compute_dist(self.all_nodes)
        self.depot_idx = 0
        self.n_c       = len(self.customers)
        self.n_f       = len(self.stations)
        self.cust_idx  = list(range(1, self.n_c + 1))
        self.stat_idx  = list(range(self.n_c + 1, self.n_c + self.n_f + 1))

    def node(self, idx):
        return self.all_nodes[idx]

    def tt(self, i, j):
        return self.dist[i][j] / self.v


# ============================================================
# 3. DP SUB-PROBLEM
# ============================================================
#
# Proactive lookahead:
#   Her musteriye gitmeden once sunu kontrol et:
#   "Mevcut batarya ile bu musteriye gidip, kalan rotayi ve
#    depot donusunu tamamlayabilir miyim?"
#   Hayir ise, simdi en uygun istasyona ugra (partial recharge).
#
# Bu sekilde reaktif (batarya=0) yerine proaktif istasyon
# karari verilir; Gurobi'nin buldugu rotalara erisim saglanir.
# ============================================================

def _min_energy_to_finish(from_idx, remaining_custs, ctx):
    """
    from_idx'ten baslayarak remaining_custs'i ziyaret edip
    depoya donmek icin gereken MINIMUM enerji (istasyon yok varsayimi).
    Bu lower bound: bu kadar bataryamiz yoksa istasyon lazim.
    """
    r     = ctx.r
    route = [from_idx] + list(remaining_custs) + [ctx.depot_idx]
    total = 0.0
    for i in range(len(route) - 1):
        total += r * ctx.dist[route[i]][route[i + 1]]
    return total


def _find_best_station(from_idx, to_idx, remaining_custs,
                       tau, y, load, D, ctx):
    """
    from_idx -> [istasyon] -> to_idx gecisi icin en iyi istasyonu sec.

    Kisitlar:
      (7)  from_idx -> istasyon: batarya yetmeli
      (8)  istasyon -> to_idx: sarj sonrasi batarya yetmeli
      (9)  y_hat <= Q
      (12) zaman: sarj_suresi + mola (varsa)
      (13) istasyon ve to_idx zaman pencereleri
      (16) suruş suresi <= Tmax
      (17) mola sadece istasyonda

    Partial recharge: kalan rota icin gereken min enerji + buffer.
    """
    Q = ctx.Q; r = ctx.r; g = ctx.g
    best_cost = INFEASIBLE_COST
    best      = None

    for si in ctx.stat_idx:
        d_from = ctx.dist[from_idx][si]
        d_to   = ctx.dist[si][to_idx]

        # Kisit (7): from -> si batarya
        if r * d_from > y + 1e-6:
            continue

        tau_si = tau + ctx.node(from_idx)['service_time'] + ctx.tt(from_idx, si)
        y_si   = y - r * d_from
        D_si   = D + ctx.tt(from_idx, si)

        # Kisit (16): suruş suresi
        if D_si > ctx.Tmax + 1e-6:
            continue

        # Kisit (13): istasyon zaman penceresi
        st_node = ctx.node(si)
        if tau_si > st_node['due_date'] + 1e-6:
            continue
        if tau_si < st_node['ready_time']:
            tau_si = st_node['ready_time']

        # Kisit (17): mola gerekiyor mu?
        need_rest = (D_si >= ctx.Tmax - 1e-6)

        # Partial recharge hesabi (kisit 8, 9):
        # to_idx'e git, sonra remaining + depot icin gereken enerji
        future_dist = d_to
        prev = to_idx
        for nc in remaining_custs:
            future_dist += ctx.dist[prev][nc]
            prev = nc
        future_dist += ctx.dist[prev][ctx.depot_idx]

        min_charge    = max(0.0, r * future_dist - y_si)
        charge_amount = min(min_charge * 1.15, Q - y_si)  # %15 buffer
        charge_amount = max(0.0, charge_amount)

        # Kisit (12): sarj suresi + mola
        tau_depart = tau_si + g * charge_amount
        y_depart   = y_si + charge_amount
        D_depart   = D_si
        if need_rest:
            tau_depart += ctx.Trest
            D_depart    = 0.0   # kisit (14)(15)

        # Kisit (8): to_idx'e yeterli mi?
        if r * d_to > y_depart + 1e-6:
            # Tam sarj dene
            charge_amount = Q - y_si
            tau_depart    = tau_si + g * charge_amount
            y_depart      = Q
            D_depart      = D_si
            if need_rest:
                tau_depart += ctx.Trest
                D_depart    = 0.0
            if r * d_to > y_depart + 1e-6:
                continue

        # Kisit (13): to_idx zaman penceresi on kontrol
        tau_to  = tau_depart + ctx.tt(si, to_idx)
        to_node = ctx.node(to_idx)
        if to_node['type'] == 'c' and tau_to > to_node['due_date'] + 1e-6:
            continue

        # Secim kriteri: detour mesafesi (kisit 1 - amaç fonksiyonu)
        detour = d_from + d_to - ctx.dist[from_idx][to_idx]
        if detour < best_cost:
            best_cost = detour
            best = (si, charge_amount, need_rest,
                    tau_depart, y_depart, D_depart)

    return best


def dp_route(cust_sequence, ctx):
    """
    Verilen musteri sirasi icin kisit-uyumlu tam rotayi olustur.

    Her adimda PROACTIVE kontrol:
      "Mevcut bataryam, bu noktadan kalan rotayi tamamlamaya yeter mi?
       Yetmiyorsa, simdi en uygun istasyona ugra."

    Donurur: (feasible, dist_total, details)
    """
    Q     = ctx.Q
    C_cap = ctx.C
    r     = ctx.r
    depot = ctx.depot_idx

    tau  = 0.0
    y    = Q
    load = C_cap
    D    = 0.0
    dist_total = 0.0
    route_plan = [(depot, 0.0, False)]
    current    = depot

    for step_i, cust in enumerate(cust_sequence):
        remaining = cust_sequence[step_i + 1:]

        # --- PROACTIVE LOOKAHEAD ---
        # Mevcut batarya ile bu noktadan (current) kalan tum rotayi
        # (cust + remaining + depot) tamamlayabilir miyiz?
        min_energy = _min_energy_to_finish(current, [cust] + list(remaining), ctx)

        if min_energy > y + 1e-6:
            # Yeterli degil — en iyi istasyona ugra
            best = _find_best_station(
                current, cust, remaining, tau, y, load, D, ctx
            )
            if best is None:
                return False, INFEASIBLE_COST, []
            st_idx, st_charge, st_rest, st_tau, st_y, st_D = best
            dist_total += ctx.dist[current][st_idx]
            route_plan.append((st_idx, st_charge, st_rest))
            tau     = st_tau
            y       = st_y
            D       = st_D
            current = st_idx

        # Anlık batarya kontrolü (reaktif güvenlik)
        if r * ctx.dist[current][cust] > y + 1e-6:
            return False, INFEASIBLE_COST, []

        # --- Musteriye git ---
        d_ij  = ctx.dist[current][cust]
        tt_ij = ctx.tt(current, cust)
        dist_total += d_ij
        y   -= r * d_ij
        D   += tt_ij
        tau += tt_ij + ctx.node(current)['service_time']

        # Kisit (16): suruş suresi
        if D > ctx.Tmax + 1e-6:
            return False, INFEASIBLE_COST, []

        # Kisit (13): zaman penceresi
        cust_node = ctx.node(cust)
        if tau < cust_node['ready_time']:
            tau = cust_node['ready_time']
        if tau > cust_node['due_date'] + 1e-6:
            return False, INFEASIBLE_COST, []

        # Kisit (5): yuk kapasitesi
        load -= cust_node['demand']
        if load < -1e-6:
            return False, INFEASIBLE_COST, []

        route_plan.append((cust, 0.0, False))
        current = cust

    # --- Son musteriden depoya don ---
    min_energy_return = r * ctx.dist[current][depot]
    if min_energy_return > y + 1e-6:
        best = _find_best_station(
            current, depot, [], tau, y, load, D, ctx
        )
        if best is None:
            return False, INFEASIBLE_COST, []
        st_idx, st_charge, st_rest, st_tau, st_y, st_D = best
        dist_total += ctx.dist[current][st_idx]
        route_plan.append((st_idx, st_charge, st_rest))
        tau     = st_tau
        y       = st_y
        D       = st_D
        current = st_idx

    d_ij  = ctx.dist[current][depot]
    tt_ij = ctx.tt(current, depot)
    dist_total += d_ij
    y   -= r * d_ij
    D   += tt_ij
    tau += tt_ij + ctx.node(current)['service_time']

    if D > ctx.Tmax + 1e-6:
        return False, INFEASIBLE_COST, []

    depot_node = ctx.node(depot)
    if tau < depot_node['ready_time']:
        tau = depot_node['ready_time']
    if tau > depot_node['due_date'] + 1e-6:
        return False, INFEASIBLE_COST, []

    route_plan.append((depot, 0.0, False))
    details = _replay_route(route_plan, ctx)
    return True, dist_total, details


def _replay_route(route_plan, ctx):
    """
    route_plan: [(node_idx, charge_amount, take_rest), ...]
    Bastan sona oynat, her adim icin detay uret.
    """
    Q     = ctx.Q
    C_cap = ctx.C
    r     = ctx.r
    g     = ctx.g

    tau  = 0.0
    y    = Q
    load = C_cap
    D    = 0.0
    details = []

    for step, (ni, charge, rest) in enumerate(route_plan):
        nd = ctx.node(ni)
        if tau < nd['ready_time']:
            tau = nd['ready_time']

        y_arrive = round(y, 6)
        y_depart = y_arrive

        # Istasyonda sarj + mola (kisit 12)
        if nd['type'] == 'f' and step > 0 and charge > 0:
            y_depart  = round(min(y_arrive + charge, Q), 6)
            tau      += g * (y_depart - y_arrive)
            if rest:
                tau += ctx.Trest
                D    = 0.0

        if nd['type'] == 'c':
            load -= nd['demand']

        details.append({
            'node_id':   nd['id'],
            'node_type': nd['type'],
            'node_idx':  ni,
            'tau':       round(tau, 2),
            'y_arrive':  round(y_arrive, 4),
            'y_depart':  round(y_depart, 4),
            'charge':    round(y_depart - y_arrive, 4),
            'load':      round(load, 2),
            'D':         round(D, 2),
            'rest':      rest,
        })

        y = y_depart
        if step < len(route_plan) - 1:
            nxt_idx = route_plan[step + 1][0]
            d_ij    = ctx.dist[ni][nxt_idx]
            tt_ij   = ctx.tt(ni, nxt_idx)
            y  -= r * d_ij
            D  += tt_ij
            tau += tt_ij + nd['service_time']

    return details


def evaluate_solution(sol, ctx):
    total_dist = 0.0
    n_vehicles = 0
    all_info   = []
    for route in sol.routes:
        if not route: continue
        ok, dist, details = dp_route(route, ctx)
        if not ok:
            sol.feasible = False
            sol.cost     = INFEASIBLE_COST
            sol.route_info = []
            return sol
        total_dist += dist
        n_vehicles += 1
        all_info.append(details)
    sol.feasible   = True
    sol.cost       = 100 * n_vehicles + total_dist
    sol.route_info = all_info
    return sol


# ============================================================
# 4. COZUM TEMSILI
# ============================================================

class Solution:
    def __init__(self, routes):
        self.routes     = routes
        self.route_info = []
        self.cost       = INFEASIBLE_COST
        self.feasible   = False

    def copy(self):
        s = Solution([r[:] for r in self.routes])
        s.route_info = copy.deepcopy(self.route_info)
        s.cost       = self.cost
        s.feasible   = self.feasible
        return s

    def all_customers(self):
        return [c for r in self.routes for c in r]

    def remove_empty_routes(self):
        self.routes = [r for r in self.routes if r]


# ============================================================
# 5. BASLANGIC COZUMU
# ============================================================

def greedy_initial_solution(ctx):
    """
    Musterileri ready_time'a gore sirala.
    Her musteri: mevcut rotalara en iyi pozisyona ekle (DP kontrollu).
    Uymazsa yeni rota ac.
    """
    customers_sorted = sorted(ctx.cust_idx,
                               key=lambda i: ctx.node(i)['ready_time'])
    routes = []

    for c in customers_sorted:
        best_cost = INFEASIBLE_COST
        best_r    = None
        best_pos  = None

        for ri, route in enumerate(routes):
            ok0, d0, _ = dp_route(route, ctx)
            d0 = d0 if ok0 else INFEASIBLE_COST
            for pos in range(len(route) + 1):
                test = route[:pos] + [c] + route[pos:]
                ok, d1, _ = dp_route(test, ctx)
                if ok and (d1 - d0) < best_cost:
                    best_cost = d1 - d0
                    best_r    = ri
                    best_pos  = pos

        if best_r is not None:
            routes[best_r].insert(best_pos, c)
        else:
            routes.append([c])

    sol = Solution(routes)
    evaluate_solution(sol, ctx)
    return sol


# ============================================================
# 6. DESTROY OPERATORLERI
# ============================================================

def destroy_random(sol, ctx, n_remove):
    """Rastgele n musteri cikar."""
    customers = sol.all_customers()
    if not customers: return sol.copy(), []
    n       = min(n_remove, len(customers))
    removed = random.sample(customers, n)
    new_sol = sol.copy()
    for r in new_sol.routes:
        for c in removed:
            if c in r: r.remove(c)
    new_sol.remove_empty_routes()
    return new_sol, removed


def destroy_worst(sol, ctx, n_remove):
    """
    Marjinal mesafe katkisi en yuksek musterileri cikar.
    Katki = d(prev,c) + d(c,next) - d(prev,next)
    """
    contributions = []
    for route in sol.routes:
        for pos, c in enumerate(route):
            prev  = ctx.depot_idx if pos == 0            else route[pos - 1]
            nxt   = ctx.depot_idx if pos == len(route)-1 else route[pos + 1]
            saving = (ctx.dist[prev][c] + ctx.dist[c][nxt]
                      - ctx.dist[prev][nxt])
            contributions.append((saving, c))

    contributions.sort(reverse=True)
    n    = min(n_remove, len(contributions))
    pool = contributions[:max(n, int(n * 2))]
    random.shuffle(pool)
    removed = [c for _, c in pool[:n]]

    new_sol = sol.copy()
    for r in new_sol.routes:
        for c in removed:
            if c in r: r.remove(c)
    new_sol.remove_empty_routes()
    return new_sol, removed


def destroy_related(sol, ctx, n_remove):
    """
    Shaw removal: konum + zaman benzerligine gore musteri cikar.
    rel(a,b) = dist(a,b) + 0.3 * |ready_a - ready_b|
    """
    customers = sol.all_customers()
    if not customers: return sol.copy(), []

    seed    = random.choice(customers)
    removed = [seed]

    def rel(a, b):
        return (ctx.dist[a][b]
                + 0.3 * abs(ctx.node(a)['ready_time']
                            - ctx.node(b)['ready_time']))

    candidates = [c for c in customers if c != seed]
    n = min(n_remove, len(customers))

    while len(removed) < n and candidates:
        best_c = min(candidates,
                     key=lambda c: min(rel(c, rc) for rc in removed))
        removed.append(best_c)
        candidates.remove(best_c)

    new_sol = sol.copy()
    for r in new_sol.routes:
        for c in removed:
            if c in r: r.remove(c)
    new_sol.remove_empty_routes()
    return new_sol, removed


# ============================================================
# 7. REPAIR OPERATORLERI
# ============================================================

def repair_greedy(sol, removed, ctx):
    """
    Her musteri icin en dusuk maliyet artisi saglayan
    (rota, pozisyon) cifti — DP kisit kontrolu ile.
    Yeni rota acilirsa +100 arac cezasi eklenir.
    """
    unassigned = list(removed)
    random.shuffle(unassigned)
    new_sol = sol.copy()

    for c in unassigned:
        best_delta = INFEASIBLE_COST
        best_r     = None
        best_pos   = None

        for ri, route in enumerate(new_sol.routes):
            ok0, d0, _ = dp_route(route, ctx)
            d0 = d0 if ok0 else INFEASIBLE_COST
            for pos in range(len(route) + 1):
                test = route[:pos] + [c] + route[pos:]
                ok, d1, _ = dp_route(test, ctx)
                if ok:
                    delta = d1 - d0
                    if delta < best_delta:
                        best_delta = delta
                        best_r     = ri
                        best_pos   = pos

        # Yeni rota acma maliyeti (+100 arac cezasi)
        ok_new, d_new, _ = dp_route([c], ctx)
        if ok_new and (d_new + 100) < best_delta:
            best_r     = None
            best_pos   = None
            best_delta = d_new + 100

        if best_r is not None:
            new_sol.routes[best_r].insert(best_pos, c)
        else:
            new_sol.routes.append([c])

    evaluate_solution(new_sol, ctx)
    return new_sol


def repair_regret(sol, removed, ctx):
    """
    Regret-2 insertion:
    Her musteri icin best ve 2nd-best ekleme maliyetini hesapla.
    Regret = 2nd_best - best en buyuk olan musteri once yerlestirilir.
    """
    unassigned = list(removed)
    new_sol    = sol.copy()

    while unassigned:
        regret_list = []

        for c in unassigned:
            insertions = []

            for ri, route in enumerate(new_sol.routes):
                ok0, d0, _ = dp_route(route, ctx)
                d0 = d0 if ok0 else INFEASIBLE_COST
                for pos in range(len(route) + 1):
                    test = route[:pos] + [c] + route[pos:]
                    ok, d1, _ = dp_route(test, ctx)
                    if ok:
                        insertions.append((d1 - d0, ri, pos))

            # Yeni rota
            ok_new, d_new, _ = dp_route([c], ctx)
            if ok_new:
                insertions.append((d_new + 100, -1, 0))

            insertions.sort(key=lambda x: x[0])

            if not insertions:
                regret_list.append((INFEASIBLE_COST, c, -1, 0))
            elif len(insertions) == 1:
                regret_list.append((INFEASIBLE_COST, c,
                                    insertions[0][1], insertions[0][2]))
            else:
                regret = insertions[1][0] - insertions[0][0]
                regret_list.append((regret, c,
                                    insertions[0][1], insertions[0][2]))

        regret_list.sort(key=lambda x: -x[0])
        _, best_c, best_r, best_pos = regret_list[0]

        if best_r == -1:
            new_sol.routes.append([best_c])
        else:
            new_sol.routes[best_r].insert(best_pos, best_c)

        unassigned.remove(best_c)

    evaluate_solution(new_sol, ctx)
    return new_sol


# ============================================================
# 8. ALNS ANA DONGUSU
# ============================================================

class ALNS:
    """
    Parametreler:
      n_iter    : toplam iterasyon
      n_remove  : her iterasyonda cikartilacak musteri sayisi
      T_init    : SA baslangic sicakligi (None -> otomatik)
      cooling   : SA soguma orani
      w_update  : agirlik guncelleme periyodu
      sigma     : skor odulleri (yeni_best, daha_iyi, kabul_edildi)
      decay     : agirlik decay faktoru
      seed      : random seed
    """
    DESTROY_OPS = ['random', 'worst', 'related']
    REPAIR_OPS  = ['greedy', 'regret']

    def __init__(self, n_iter=500, n_remove=2, T_init=None,
                 cooling=0.995, w_update=50,
                 sigma=(10, 6, 3), decay=0.8, seed=42):
        self.n_iter   = n_iter
        self.n_remove = n_remove
        self.T_init   = T_init
        self.cooling  = cooling
        self.w_update = w_update
        self.sigma    = sigma
        self.decay    = decay
        random.seed(seed)
        np.random.seed(seed)

        self.d_weights = {op: 1.0 for op in self.DESTROY_OPS}
        self.r_weights = {op: 1.0 for op in self.REPAIR_OPS}
        self.d_scores  = {op: 0.0 for op in self.DESTROY_OPS}
        self.r_scores  = {op: 0.0 for op in self.REPAIR_OPS}
        self.d_counts  = {op: 0   for op in self.DESTROY_OPS}
        self.r_counts  = {op: 0   for op in self.REPAIR_OPS}

    def _select_op(self, weights_dict):
        ops   = list(weights_dict.keys())
        w     = [weights_dict[o] for o in ops]
        total = sum(w)
        probs = [wi / total for wi in w]
        return random.choices(ops, weights=probs, k=1)[0]

    def _update_weights(self):
        for op in self.DESTROY_OPS:
            if self.d_counts[op] > 0:
                perf = self.d_scores[op] / self.d_counts[op]
                self.d_weights[op] = (self.decay * self.d_weights[op]
                                      + (1 - self.decay) * perf)
            self.d_scores[op] = 0.0
            self.d_counts[op] = 0
        for op in self.REPAIR_OPS:
            if self.r_counts[op] > 0:
                perf = self.r_scores[op] / self.r_counts[op]
                self.r_weights[op] = (self.decay * self.r_weights[op]
                                      + (1 - self.decay) * perf)
            self.r_scores[op] = 0.0
            self.r_counts[op] = 0

    def _destroy(self, sol, op, ctx):
        if op == 'random':  return destroy_random(sol, ctx, self.n_remove)
        if op == 'worst':   return destroy_worst(sol, ctx, self.n_remove)
        if op == 'related': return destroy_related(sol, ctx, self.n_remove)

    def _repair(self, sol, removed, op, ctx):
        if op == 'greedy': return repair_greedy(sol, removed, ctx)
        if op == 'regret': return repair_regret(sol, removed, ctx)

    def run(self, ctx, verbose=True):
        current = greedy_initial_solution(ctx)
        best    = current.copy()

        if not current.feasible:
            print("UYARI: Baslangic cozumu infeasible!")

        n_veh = len([r for r in current.routes if r])
        if verbose:
            print(f"Baslangic: cost={current.cost:.2f}  "
                  f"arac={n_veh}  feasible={current.feasible}")

        T = self.T_init if self.T_init else max(current.cost * 0.05, 1.0)
        history = []

        for it in range(1, self.n_iter + 1):
            d_op = self._select_op(self.d_weights)
            r_op = self._select_op(self.r_weights)

            destroyed, removed = self._destroy(current, d_op, ctx)
            candidate = self._repair(destroyed, removed, r_op, ctx)

            score = 0
            if candidate.feasible:
                delta = candidate.cost - current.cost
                if candidate.cost < best.cost - 1e-6:
                    best    = candidate.copy()
                    current = candidate.copy()
                    score   = self.sigma[0]
                elif delta < -1e-6:
                    current = candidate.copy()
                    score   = self.sigma[1]
                elif random.random() < math.exp(-delta / max(T, 1e-9)):
                    current = candidate.copy()
                    score   = self.sigma[2]

            self.d_scores[d_op] += score
            self.r_scores[r_op] += score
            self.d_counts[d_op] += 1
            self.r_counts[r_op] += 1

            if it % self.w_update == 0:
                self._update_weights()

            T *= self.cooling
            history.append(best.cost)

            if verbose and it % 100 == 0:
                dw = {o: f"{self.d_weights[o]:.2f}" for o in self.DESTROY_OPS}
                rw = {o: f"{self.r_weights[o]:.2f}" for o in self.REPAIR_OPS}
                print(f"  iter {it:4d} | best={best.cost:.2f} "
                      f"| cur={current.cost:.2f} | T={T:.3f} "
                      f"| d={dw} | r={rw}")

        return best, history


# ============================================================
# 9. SONUC YAZICI + KISIT KONTROLU
# ============================================================

def print_solution(sol, ctx):
    if not sol.feasible:
        print("Feasible cozum bulunamadi.")
        return

    n_veh      = len([r for r in sol.routes if r])
    dist_total = sol.cost - 100 * n_veh

    print(f"\n{'='*65}")
    print(f"ALNS SONUCU")
    print(f"  Objective    : {sol.cost:.4f}")
    print(f"  Arac sayisi  : {n_veh}  (x100 = {100*n_veh:.0f})")
    print(f"  Toplam mesafe: {dist_total:.4f}")
    print(f"{'='*65}")

    for ri, (route, info) in enumerate(zip(sol.routes, sol.route_info)):
        if not route: continue
        route_ids = [ctx.node(c)['id'] for c in route]
        print(f"\nVehicle {ri}: {ctx.depot['id']} -> "
              f"{' -> '.join(route_ids)} -> {ctx.depot['id']}")
        print(f"  {'Node':<10} {'tau':>7} {'y_arr':>7} {'y_dep':>7} "
              f"{'charge':>8} {'load':>7} {'D':>7} {'rest':>5}")
        print(f"  {'-'*62}")
        for step in info:
            rest_str = 'YES' if step['rest'] else '-'
            print(f"  {step['node_id']:<10} {step['tau']:>7.1f} "
                  f"{step['y_arrive']:>7.2f} {step['y_depart']:>7.2f} "
                  f"{step['charge']:>8.4f} {step['load']:>7.1f} "
                  f"{step['D']:>7.1f} {rest_str:>5}")

    print(f"\n{'-'*65}")
    print("Kisit Kontrolu:")
    violations = _check_constraints(sol, ctx)
    if not violations:
        print("  Tum kisitlar saglaniyor [OK]")
        print("  (2) Her musteri 1 kez  (5)(6) Yuk  (7-10) Batarya"
              "  (11-13) TW  (14-17) Suruş")
    else:
        for v in violations:
            print(f"  IHLAL: {v}")


def _check_constraints(sol, ctx):
    violations = []

    # (2) Her musteri tam 1 kez
    visited = sol.all_customers()
    if len(visited) != len(set(visited)):
        violations.append("Bazi musteriler birden fazla ziyaret edildi")
    if set(visited) != set(ctx.cust_idx):
        missing = set(ctx.cust_idx) - set(visited)
        violations.append(f"Ziyaret edilmeyen musteriler: "
                          f"{[ctx.node(i)['id'] for i in missing]}")

    for ri, info in enumerate(sol.route_info):
        for step in info:
            ni = step['node_idx']
            nd = ctx.node(ni)

            # (13) Zaman pencereleri
            if step['tau'] < nd['ready_time'] - 1e-3:
                violations.append(
                    f"R{ri} {nd['id']}: tau={step['tau']:.1f} "
                    f"< ready={nd['ready_time']}")
            if step['tau'] > nd['due_date'] + 1e-3:
                violations.append(
                    f"R{ri} {nd['id']}: tau={step['tau']:.1f} "
                    f"> due={nd['due_date']}")

            # (5) Yuk kapasitesi
            if step['load'] < -1e-3:
                violations.append(
                    f"R{ri} {nd['id']}: load={step['load']:.2f} < 0")

            # (7)(10) Batarya negatif olmamali
            if step['y_arrive'] < -1e-3:
                violations.append(
                    f"R{ri} {nd['id']}: y_arrive={step['y_arrive']:.4f} < 0")

            # (9) y_hat <= Q
            if step['y_depart'] > ctx.Q + 1e-3:
                violations.append(
                    f"R{ri} {nd['id']}: y_depart={step['y_depart']:.4f} > Q")

            # (14-16) Suruş suresi
            if step['D'] > ctx.Tmax + 1e-3:
                violations.append(
                    f"R{ri} {nd['id']}: D={step['D']:.1f} > Tmax={ctx.Tmax}")

    return violations


# ============================================================
# CALISTIR
# ============================================================

if __name__ == '__main__':
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'c101C5.txt'

    print(f"Instance : {filepath}")
    ctx = ProblemContext(filepath)
    print(f"Musteri  : {len(ctx.customers)}  "
          f"Istasyon : {len(ctx.stations)}  "
          f"Q={ctx.Q}  C={ctx.C}  r={ctx.r}  g={ctx.g}")

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

    best_sol, history = alns.run(ctx, verbose=True)
    print_solution(best_sol, ctx)