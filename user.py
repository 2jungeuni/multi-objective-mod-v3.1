class User:
    def __init__(self, time, id, pu, do, cap, logger):
        super(User, self).__init__()
        self.time = time
        self.id = id
        self.pu = pu
        self.do = do
        self.cap = cap

        self.shortest_time = 0
        self.expected_waiting_time = 0
        self.expected_travel_time = 0

        self.vehicle = None
        self.pick_up = False
        self.drop_off = False

        self.logger = logger

    def __repr__(self):
        return "User " + str(self.id)

    # RESET
    def reset(self):
        self.expected_waiting_time = 0
        self.expected_travel_time = 0

    # SET
    def set_waiting_time(self, time):
        self.expected_waiting_time = time

    def set_travel_time(self, time):
        self.expected_travel_time = time

    def match_car(self, veh):
        if veh is not None:
            print("Already in another vehicle.")
        else:
            self.vehicle = veh

    def fix_car(self, veh):
        self.vehicle = veh

    def get_out_car(self, veh):
        # assert self.vehicle == veh, "The car is different with riding car."
        self.drop_off = True
        self.expected_waiting_time = 0
        self.expected_travel_time = 0

    def stopover(self, veh):
        # assert self.vehicle == veh, "The car is different with riding car."
        self.expected_waiting_time = 0
        self.expected_travel_time = 0
        self.vehicle = None


