# built-in
import sys
import time
import logging
import argparse
import datetime
import pandas as pd

# solver
from gurobipy import *
from tabulate import tabulate

# my own
from user import User
from system import System
from vehicle import Vehicle
from astar.planner import *

# initialize the logger
logger = logging.getLogger("Multi objective mod system")
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
logger.setLevel(logging.DEBUG)

formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


def show_all_users(users):
    print_pudo = {"request ID": [],
                  "pickup location": [],
                  "dropoff location": [],
                  "# users": [],
                  "waiting time": [],
                  "travel time": [],
                  "request time": [],
                  "detour ratio": []}

    for user in users:
        print_pudo["request ID"].append(user.id)
        print_pudo["pickup location"].append(user.pu)
        print_pudo["dropoff location"].append(user.do)
        print_pudo["# users"].append(user.cap)
        print_pudo["waiting time"].append(round(user.expected_waiting_time, 2))
        print_pudo["travel time"].append(round(user.expected_travel_time, 2))
        print_pudo["request time"].append(user.time)
        print_pudo["detour ratio"].append(user.expected_travel_time / user.shortest_time)

    # print out
    print("[Calls]")
    print(tabulate(print_pudo, headers="keys", tablefmt="fancy_grid", missingval="N/A"))


def show_all_vehicles(vehicles, time=None):
    print_veh = {"vehicle ID": [],
                 "origin": [],
                 "capacity": [],
                 "users": [],
                 "# users": [],
                 "path": [],
                 "travel time": [],
                 "start time": [],
                 "location": []}

    for veh in vehicles:
        print_veh["vehicle ID"].append(veh.id)
        print_veh["origin"].append(veh.loc)
        print_veh["capacity"].append(veh.capacity)
        print_veh["users"].append(veh.on_board)
        print_veh["# users"].append(veh.num_users)
        print_veh["path"].append([stop for stop, u, tt in veh.route])
        print_veh["travel time"].append(round(veh.travel_time, 2))
        print_veh["start time"].append(veh.time)
        if veh.next_loc is None:
            print_veh["location"].append(str(veh.here[0]))
        else:
            print_veh["location"].append(str(veh.here[0]) + "->" + str(veh.next_loc[0]))

    # print out
    print("[Vehicles]")
    print(tabulate(print_veh, headers="keys", tablefmt="fancy_grid", missingval="N/A"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="three weights (alpha, beta, gamma), "
                                                 "penalty given for not visiting a location, detour ratio limit")
    parser.add_argument("-w", "--weight", type=int, required=True, nargs="+", help="alpha, beta, gamma")
    parser.add_argument("-p", "--penalty", type=int, required=True, help="penalty for unvisited stations")
    parser.add_argument("-d", "--detour", type=float, required=True, help="detour ratio")

    args = parser.parse_args()

    # system initialization
    system = System(logger)

    # routing planner
    planner = RoutingPlanner(args.weight[0], args.weight[1], args.weight[2])
    system.set_planner(planner)

    # load passengers calls
    call = pd.read_csv("./input/call-test.csv")
    call["time"] = pd.to_datetime(call["time"], format="%Y/%m/%d %H:%M:%S")

    # load vehicles
    veh = pd.read_csv("./input/veh-test.csv")
    veh["time"] = pd.to_datetime(veh["time"], format="%Y/%m/%d %H:%M:%S")

    now = datetime.datetime.strptime("2024/08/16 00:00:00", "%Y/%m/%d %H:%M:%S")
    start_time = datetime.datetime.now()
    '''
    now = datetime.datetime.now()
    '''
    time_limit = datetime.datetime.strptime("2024/08/16 00:08:00", "%Y/%m/%d %H:%M:%S")
    system.set_time(now)

    count = 0
    while now < time_limit:
        now = now + datetime.timedelta(seconds=(datetime.datetime.now() - start_time).seconds)
        start_time = datetime.datetime.now()
        '''
        now = datetime.datetime.now()
        '''
        system.set_time(now)
        logger.info("The current time is %s.", now)

        # initial inputs
        call_ = call[call["time"] <= now]
        # initial vehicles
        veh_ = veh[veh["time"] <= now]
        logger.info("The number of calls are %s and the number of vehicles entering service is %s.",
                    len(call_), len(veh_))

        if len(call_) == 0 and len(veh_) == 0:
            logger.info("There is no demands and vehicles.")
            time.sleep(5)
            continue

        # add new passengers
        for idx, row in call_.iterrows():
            if row["id"] in system.users_ids:
                logger.info("%s is duplicate user ID.", row["id"])
                sys.exit()
            system.add_users(User(row["time"],
                                  row["id"],
                                  row["pick up"],
                                  row["drop off"],
                                  row["num"],
                                  logger))
        # add new vehicles
        for idx, row in veh_.iterrows():
            if row["id"] in system.vehicles_ids:
                logger.info("%s is duplicate vehicle ID.", row["id"])
                sys.exit()
            system.add_vehicles(Vehicle(row["time"],
                                        row["id"],
                                        row["location"],
                                        row["working time"],
                                        row["capacity"],
                                        args.detour,
                                        logger))

        print("##### INPUT #####")
        show_all_users(system.users)
        show_all_vehicles(system.vehicles)

        # optimize
        system.opt(args.penalty, args.detour)

        # optimization result
        print("##### RESULT #####")
        show_all_users(system.users)
        show_all_vehicles(system.vehicles, now)

        # table drop
        call = call.drop(call_.index, axis=0)
        veh = veh.drop(veh_.index, axis=0)

        count += 1

