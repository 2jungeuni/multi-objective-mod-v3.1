import datetime


class Vehicle:
    def __init__(self, time, id, loc, working_time, cap, detour_ratio, logger):
        super(Vehicle, self).__init__()
        self.time = time
        self.id = id
        self.loc = loc
        self.working_time = working_time
        self.capacity = cap
        self.detour_ratio = detour_ratio

        self.route = []
        self.on_board = []
        self.num_users = 0
        self.detour_ratio = {}
        self.travel_time = 0
        self.have_to_service = []

        self.logger = logger

    def __repr__(self):
        return "Vehicle " + str(self.id)

    # RESET
    def reset(self):
        self.route = [self.loc]
        self.on_board = []
        self.detour_ratio = {}
        self.travel_time = 0
        self.num_users = 0
        self.have_to_service = []

    # ADD
    def add_route(self, stop):
        self.route.append(stop[0])

    def accept_user(self, user):
        self.on_board.append(user)
        if user is not self.on_board:
            self.num_users += user.cap

    def reject_user(self, user, distance, system_time):
        self.logger.info("%s tried to board vehicle %s, but was rejected because the detour ratio was %s", user, self.id, round(self.detour_ratio[user], 2))
        self.route.remove(user.pu)
        self.route.remove(user.do)
        self.on_board.remove(user)
        self.detour_ratio.pop(user)
        self.num_users -= user.cap
        if user in self.have_to_service:
            self.have_to_service.remove(user)
            self.travel_time = 0
            over_system_time = False
            for i in range(len(self.route) - 1):
                self.travel_time += distance[self.route[i]][self.route[i + 1]]
                if not over_system_time:
                    if system_time <= self.time + datetime.timedelta(seconds=self.travel_time):
                        over_system_time = True
                        self.have_to_service = self.on_board.copy()
        else:
            self.travel_time = sum([distance[self.route[i]][self.route[i + 1]] for i in range(len(self.route) - 1)])

    def is_detour(self, limit):
        if any(detour > limit for detour in self.detour_ratio.values()):
            return True
        return False
