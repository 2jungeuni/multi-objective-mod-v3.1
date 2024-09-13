# built-in
import sys
import copy
import datetime
import numpy as np

# solver
import gurobipy as gp
from gurobipy import *

# optimization status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}


class System:
    def __init__(self, logger):
        super(System, self).__init__()
        self.time = None
        self.vehicles_ids = set()
        self.users_ids = set()

        self.vehicles = set()
        self.users = set()
        self.stops = [(0, 0)]

        self.distance = {0: {}}     # 0 is artificial depot
        self.planner = None

        self.logger = logger

    # SET
    def set_time(self, time):
        self.time = time

    def set_planner(self, planner):
        self.planner = planner

    # GET
    def get_cost(self, frm, to):
        if frm in self.distance.keys():
            if to not in self.distance[frm].keys():
                if frm == 0 or to == 0:
                    self.distance[frm][to] = 0
                else:
                    self.planner.init()
                    self.distance[frm][to] = self.planner.astar(frm, to)
        else:
            self.distance[frm] = {to: 0}
            if frm == 0 or to == 0:
                self.distance[frm][to] = 0
            else:
                self.planner.init()
                self.distance[frm][to] = self.planner.astar(frm, to)


    # ADD
    def add_vehicles(self, veh):
        self.vehicles_ids.add(veh.id)   # add ids set
        self.vehicles.add(veh)          # add vehicles set

        # calculating distance from/to artificial depot
        self.get_cost(0, veh.loc)
        self.get_cost(veh.loc, 0)
        self.stops.append((veh.loc, veh))

        for v in self.vehicles:
            self.get_cost(v.loc, veh.loc)
            self.get_cost(veh.loc, v.loc)

        for u in self.users:
            self.get_cost(u.pu, veh.loc)
            self.get_cost(veh.loc, u.pu)
            self.get_cost(u.do, veh.loc)
            self.get_cost(veh.loc, u.do)

    def add_users(self, user):
        self.users_ids.add(user.id)     # add ids set
        self.users.add(user)            # add users set
        self.get_cost(user.pu, user.do)
        self.stops.append((user.pu, user))
        self.stops.append((user.do, user))
        user.shortest_time = self.distance[user.pu][user.do]

        # calculating distance from/to artificial depot
        self.get_cost(0, user.pu)
        self.get_cost(user.pu, 0)
        self.get_cost(0, user.do)
        self.get_cost(user.do, 0)

        for v in self.vehicles:
            self.get_cost(v.loc, user.pu)
            self.get_cost(user.pu, v.loc)
            self.get_cost(v.loc, user.do)
            self.get_cost(user.do, v.loc)

        for u in self.users:
            self.get_cost(u.pu, user.pu)
            self.get_cost(user.pu, u.pu)
            self.get_cost(u.do, user.pu)
            self.get_cost(user.pu, u.do)
            self.get_cost(u.pu, user.do)
            self.get_cost(user.do, u.pu)
            self.get_cost(u.do, user.do)
            self.get_cost(user.do, u.do)

    def opt(self, pty, detour):
        # optimization model
        m = gp.Model()
        m.Params.outputFlag = False

        # idx -> stop
        idx = 0
        nv = 0
        idx_stops = {}
        stops_idx = {}
        for loc, user in self.stops:
            if (user in self.vehicles) and (user.time + datetime.timedelta(seconds=user.working_time)) <= self.time:
                self.vehicles.remove(user)
                continue
            # if (user in self.users) and (user.vehicle != None):
            #     continue
            idx_stops[idx] = (loc, user)
            stops_idx[(loc, user)] = idx
            idx += 1

        # number of stops
        n = len(idx_stops)
        # number of vehicles
        nv = len(self.vehicles)

        # origins, pickups, dropoffs
        origins = []
        pickups = []
        dropoffs = []
        for (loc, user), idx in stops_idx.items():
            if user in self.vehicles:
                origins.append(idx)
            elif user in self.users and user.pu == loc:
                pickups.append(idx)
                user.reset()
            elif user in self.users and user.do == loc:
                dropoffs.append(idx)

        # capacity
        p = {}
        penalty = {}
        for i in range(n):
            for v in range(nv):
                if i in pickups:
                    p[(i, v)] = -1 * idx_stops[i][1].cap
                else:
                    p[(i, v)] = 0
            penalty[i] = pty
        penalty[0] = 0

        # vehicles
        vehicles = list(self.vehicles)

        # edges
        e_vars = m.addVars([(i, j, k)
                            for i in range(n) for j in range(n) for k in range(nv) if i != j],
                           vtype=GRB.BINARY, name="e")
        # passengers
        p_vars = m.addVars(p.keys(), obj=p, vtype=GRB.BINARY, name="p")
        # penalty
        pty_vars = m.addVars(penalty.keys(), obj=penalty, vtype=GRB.BINARY, name="penalty")
        # sequences
        s_vars = m.addVars(np.arange(1, n + 1), lb=1, ub=n, vtype=GRB.INTEGER, name="sequence")

        #
        for v in range(nv):
            for user in vehicles[v].on_board:
                m.addConstr(p_vars[stops_idx[(user.pu, user)], v] == 1)
            if vehicles[v].next_loc is not None:
                m.addConstr(e_vars[stops_idx[vehicles[v].here], stops_idx[vehicles[v].next_loc], v] == 1)

        cons1 = m.addConstrs(p_vars.sum(i, "*") <= 1 for i in range(n) if i != 0)
        # Constraint 2: visited node i must have an outgoing edge.
        cons2 = m.addConstrs(e_vars.sum(i, "*", v) == p_vars[(i, v)] for i in range(n) for v in range(nv)
                             if i not in origins and i != 0)
        # Constraint 3: visited node j must have an ingoing edge.
        cons3 = m.addConstrs(e_vars.sum("*", j, v) == p_vars[(j, v)] for j in range(n) for v in range(nv)
                             if j not in origins and j != 0)
        # # Constraint 4: considering the origin.
        cons4_1 = m.addConstr(p_vars.sum(0, "*") == nv)
        cons4_2 = m.addConstrs(e_vars.sum("*", 0, v) == 1 for v in range(nv))
        cons4_3 = m.addConstrs(e_vars.sum(0, "*", v) == 0 for v in range(nv))
        # # Constraint 5: fixed initial positions.
        cons5_1 = m.addConstrs(p_vars[stops_idx[(vehicles[v].loc, vehicles[v])], v] == 1 for v in range(nv))
        cons5_2 = m.addConstrs(e_vars.sum(stops_idx[(vehicles[v].loc, vehicles[v])], "*", v) ==
                               p_vars[stops_idx[(vehicles[v].loc, vehicles[v])], v] for v in range(nv))
        cons5_3 = m.addConstrs(e_vars.sum("*", stops_idx[(vehicles[v].loc, vehicles[v])], "*") == 0 for v in range(nv))
        # Constraint 6: working time limit.
        cons6 = m.addConstrs(gp.quicksum(e_vars[i, j, v] * self.distance[idx_stops[i][0]][idx_stops[j][0]]
                                         for i in range(n) for j in range(n) if i != j) <= vehicles[v].working_time
                             for v in range(nv))
        # Constraint 7: capacity limit
        cons7 = m.addConstrs(gp.quicksum(-1 * p[i, v] * p_vars[i, v] for i in range(n)) <= vehicles[v].capacity
                             for v in range(nv))
        # Constraint 8: penalty
        cons8 = m.addConstrs(1 - p_vars.sum(i, "*") == pty_vars[i] for i in range(n) if i != 0)
        # # Constraint 9: pickup-dropoff pairs
        cons9 = m.addConstrs(p_vars[stops_idx[(user.pu, user)], v] == p_vars[stops_idx[(user.do, user)], v]
                             for user in self.users for v in range(nv))
        # Constraint 10: sequences
        cons10_1 = m.addConstrs(s_vars[i] <= s_vars[j] + n * (1 - e_vars[i, j, k]) - 1
                                for i, j, k in e_vars.keys() if i != 0 and j != 0)
        cons10_2 = m.addConstrs(s_vars[stops_idx[(user.pu, user)]] + 1 <= s_vars[stops_idx[(user.do, user)]]
                                for user in self.users)

        def subtourlim(model, where):
            if where == GRB.Callback.MIPSOL:
                # make a list of edges selected in the solution
                vals = model.cbGetSolution(model._vars)
                selected = gp.tuplelist((i, j, k) for i, j, k in model._vars.keys() if vals[i, j, k] > 0.5)
                # find the shortest cycle in the selected edge list
                tour = subtour(selected)
                for v in range(nv):
                    if tour[v]:
                        for tv in tour[v]:
                            if len(tv) < n:
                                # add subtour elimination constraint for every pair of cities in tour
                                model.cbLazy(gp.quicksum(model._vars[i, j, v] for i, j in itertools.permutations(tv, 2))
                                             <= len(tv) - 1)

        def subtour(edges, exclude_depot=True):
            cycle = [[] for v in range(nv)]

            for v in range(nv):
                unvisited = list(np.arange(0, n))

                while unvisited:  # true if list is non-empty
                    this_cycle = []
                    neighbors = unvisited

                    while neighbors:
                        current = neighbors[0]
                        this_cycle.append(current)
                        unvisited.remove(current)
                        neighbors = [j for i, j, k in edges.select(current, '*', '*') if (j in unvisited) and (k == v)]

                    if len(this_cycle) > 1:
                        if exclude_depot:
                            if not (0 in this_cycle):
                                cycle[v].append(this_cycle)
            return cycle

        m._vars = e_vars
        m._dvars = p_vars
        m._ddvars = penalty
        m._svars = s_vars
        m.Params.lazyConstraints = 1
        m.optimize(subtourlim)

        # status
        self.logger.info("Solved (%s)", status_dict[m.status])

        if m.status != 2:
            if m.status == 3:
                m.computeIIS()
                m.write("model.ilp")
            sys.exit("There is no solution. Check constraints again.")

        e_vals = m.getAttr('x', e_vars)

        # get solutions
        sol = {}
        for car in range(nv):
            sol[car] = {}
        for i, j, k in e_vals.keys():
            if e_vals[i, j, k] > 0.5:
                sol[k][i] = j

        for v_idx, v in enumerate(self.vehicles):
            v.reset()
            route = sol[v_idx]
            station = stops_idx[(v.loc, v)]

            travel_time = 0
            over_system_time = False
            while True:
                station_ = copy.copy(station)
                station = route[station]
                travel_time += self.distance[idx_stops[station_][0]][idx_stops[station][0]]
                if station == 0:
                    break
                elif station in pickups:
                    user = idx_stops[station][1]
                    user.set_waiting_time(travel_time)
                    v.accept_user(user)
                elif station in dropoffs:
                    user = idx_stops[station][1]
                    user.set_travel_time(travel_time - user.expected_waiting_time)
                    v.detour_ratio[user] = user.expected_travel_time / user.shortest_time
                v.add_route((idx_stops[station][0], idx_stops[station][1], travel_time))

                if not over_system_time:
                    if self.time <= v.time + datetime.timedelta(seconds=travel_time):
                        over_system_time = True

                v.travel_time = travel_time

            # is detour
            while v.is_detour(detour):
                users_over_limit = []
                booking = None
                for user, dr in v.detour_ratio.items():
                    if dr > detour:
                        if user != v.next_loc[1]:
                            users_over_limit.append(user)
                        else:
                            booking = user

                if booking is not None:
                    v.detour_ratio.pop(booking)

                if len(users_over_limit) > 0:
                    min_cap_user = min(users_over_limit, key=lambda x: x.cap)
                    min_cap_user.stopover(v)
                    v.reject_user(min_cap_user, self.distance, self.time)

            v.move(self.time)