import datetime


class Vehicle:
    def __init__(self, time, id, loc, working_time, cap, detour_ratio, logger):
        super(Vehicle, self).__init__()
        self.time = time
        self.id = id
        self.loc = loc
        self.here = (loc, self)
        self.next_loc = None
        self.working_time = working_time
        self.capacity = cap
        self.detour_ratio = detour_ratio

        self.route = []
        self.on_board = []
        self.num_users = 0
        self.detour_ratio = {}
        self.travel_time = 0

        self.logger = logger

    def __repr__(self):
        return "Vehicle " + str(self.id)

    # RESET
    def reset(self):
        self.route = [(self.loc, self, 0)]
        self.on_board = []
        self.detour_ratio = {}
        self.travel_time = 0
        self.num_users = 0

    # ADD
    def add_route(self, stop):
        self.route.append(stop)

    def accept_user(self, user):
        self.on_board.append(user)
        if user is not self.on_board:
            self.num_users += user.cap

    def move(self, now):
        for idx, (loc, u, tt) in enumerate(self.route):
            if self.time + datetime.timedelta(seconds=tt) > now:
                self.here = (self.route[idx-1][0], self.route[idx-1][1])
                self.next_loc = (loc, u)
                break

    def reject_user(self, user, distance, system_time):
        self.logger.info("%s tried to board vehicle %s, but was rejected because the detour ratio was %s", user, self.id, round(self.detour_ratio[user], 2))
        self.on_board.remove(user)
        self.detour_ratio.pop(user)
        self.num_users -= user.cap
        revised = [(loc, u, tt) for loc, u, tt in self.route
                   if not (u == user and (loc == user.pu or loc == user.do))]
        for idx, (loc, u, tt) in enumerate(revised):
            if u == self:
                continue
            flag = (loc, u, revised[idx-1][2] + distance[revised[idx-1][0]][revised[idx][0]])
            revised[idx] = flag
        self.route = revised
        self.travel_time = self.route[-1][2]

    def is_detour(self, limit):
        if any(detour > limit for detour in self.detour_ratio.values()):
            return True
        return False
