import numpy as np
import matplotlib.pyplot as plt

# =========================
# Parameters
# =========================
GRID_SIZE = 5
SIM_MINUTES = 500
MAX_WAIT = 7
MAX_RIDERS_PER_CELL = 10
MAX_DRIVERS_PER_CELL = 10

BASE_LAM = 0.05  # normal rider arrival per cell per minute
SURGES = [(50,100), (200,230), (350,380)]  # surge time windows
SURGE_LAM = 0.5  # additional Poisson mean during surge

BASE_RATE = 1.5
DIST_DECAY = 0.1
EPS = 1

# spawn sensitivity
KR = 0.8   # rider spawn sensitivity
KD = 0.8   # driver spawn sensitivity

# price sensitivity
KB = 0.5  # rider bid sensitivity
KA = 0.5   # driver ask sensitivity

# =========================
# Utils
# =========================
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# =========================
# Agents
# =========================
class Rider:
    _id = 0
    def __init__(self, t, loc, ref_price, pressure):
        self.id = Rider._id; Rider._id += 1
        self.loc = loc
        self.spawn_time = t
        self.wait = 0
        self.matched = False
        self.active_requests = set()

        noise = np.random.normal(-0.1, 0.1)
        self.bid = ref_price * (pressure ** -KB) * (1 + noise)
        self.bid = max(0.3, self.bid)

    def expired(self):
        return self.wait >= MAX_WAIT

class Driver:
    _id = 0
    def __init__(self, t, loc, ref_price, pressure):
        self.id = Driver._id; Driver._id += 1
        self.loc = loc
        self.spawn_time = t
        self.wait = 0
        self.matched = False
        self.inbox = {}
        self.inbox_age = 0

        noise = np.random.normal(-0.1, 0.1)
        # drivers ask DOWN when pressure < 1 (competition)
        self.ask = ref_price * (pressure ** (-KA)) * (1 + noise)
        self.ask = max(0.2, self.ask)

        self.patience = int(MAX_WAIT / (1 + pressure))

    def expired(self):
        return self.wait >= MAX_WAIT

# =========================
# Market Cell
# =========================
class MarketCell:
    def __init__(self):
        self.recent = []
        self.recent_asks = []
        self.recent_bids = []
        

    def record_trade(self, bid, ask):
        # self.recent.append(bid)
        self.recent_bids.append(bid)
        self.recent_asks.append(ask)
        # if len(self.recent) > 10:
        #     self.recent.pop(0)
        if len(self.recent_asks) > 10:
            self.recent_asks.pop(0)
        if len(self.recent_bids) > 10:
            self.recent_bids.pop(0)

    def avg_rate(self):
        return np.mean(self.recent) if self.recent else BASE_RATE

    def avg_bid(self):
        return np.mean(self.recent_bids) if self.recent_bids else BASE_RATE
    
    def avg_ask(self):
        return np.mean(self.recent_asks) if self.recent_asks else BASE_RATE


# =========================
# Simulation
# =========================
class Simulation:
    def __init__(self):
        self.time = 0
        self.riders = {}
        self.drivers = {}

        self.market = [[MarketCell() for _ in range(GRID_SIZE)]
                       for _ in range(GRID_SIZE)]

        self.rider_grid = np.zeros((GRID_SIZE, GRID_SIZE), int)
        self.driver_grid = np.zeros((GRID_SIZE, GRID_SIZE), int)

        # tracking
        self.rider_counts = []
        self.driver_counts = []
        self.avg_rates = []
        self.dropped_riders = []
        self.dropped_drivers = []
        self.riders_spawned = []
        self.drivers_spawned = []
        self.pressure_list = []

    # -------------------------
    # Local pressure
    # -------------------------
    def local_pressure(self, i, j, r=1):
        riders = drivers = 0
        for di in range(-r, r+1):
            for dj in range(-r, r+1):
                ni, nj = i+di, j+dj
                if 0 <= ni < GRID_SIZE and 0 <= nj < GRID_SIZE:
                    riders += self.rider_grid[ni, nj]
                    drivers += self.driver_grid[ni, nj]
        return riders, drivers   # <- MUST return a tuple
    
    def local_price_avg(self, i, j, order="bid",r=int(GRID_SIZE//3)):
        total = 0
        count = 0
        for di in range(-r, r+1):
            for dj in range (-r, r+1):
                ni, nj = di+i, dj+j
                if 0 <= ni < GRID_SIZE and 0 <= nj < GRID_SIZE:
                    if order == "bid":
                        cell_price = self.market[ni][nj].avg_bid()
                        total += cell_price
                    elif order == "ask":
                        cell_price = self.market[ni][nj].avg_ask()
                        total += cell_price
                    count += 1
        
        return total/max(1, count)
            




    # -------------------------
    # Spawn riders
    # -------------------------
    def spawn_riders(self):
        spawned = 0
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                if self.rider_grid[i,j] >= MAX_RIDERS_PER_CELL:
                    continue

                # start with baseline lambda
                lam = BASE_LAM

                # add surge lambda if inside any surge period
                for s in SURGES:
                    if s[0] <= self.time <= s[1]:
                        lam += SURGE_LAM

                # sample number of new riders
                n = np.random.poisson(lam)
                n = min(n, MAX_RIDERS_PER_CELL - self.rider_grid[i,j])

                # spawn riders
                for _ in range(n):
                    r, d = self.local_pressure(i,j)
                    pressure = (d + EPS) / (r + 1)  # add 1 to avoid zero
                    avg_price = self.local_price_avg(i,j,"bid")
                    rider = Rider(self.time, (i,j), avg_price, pressure)
                    self.riders[rider.id] = rider
                    self.rider_grid[i,j] += 1
                    spawned += 1
        self.riders_spawned.append(spawned)

        total_pressure = 0
        cells_counted = 0
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                r, d = self.local_pressure(i,j)
                if r + 1 > 0:
                    total_pressure += (d + EPS) / (r + 1)
                cells_counted += 1
        avg_rider_pressure = total_pressure / max(1, cells_counted)
        self.pressure_list.append(avg_rider_pressure)


    # -------------------------
    # Spawn drivers
    # -------------------------
    def spawn_drivers(self):
        spawned = 0
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                if self.driver_grid[i, j] >= MAX_DRIVERS_PER_CELL:
                    continue

                riders_count, drivers_count = self.local_pressure(i,j)
                pressure = (drivers_count + EPS) / (riders_count + 1)  # add 1 to avoid zero
                lam = 0.2 * (pressure ** -KD)  # positive exponent
                lam = min(lam, 5.0)            # cap max Poisson lambda
                n = np.random.poisson(lam)

                for _ in range(min(n, MAX_DRIVERS_PER_CELL - self.driver_grid[i, j])):
                    ref = self.local_price_avg(i,j, "ask")
                    driver = Driver(self.time, (i, j), ref, pressure)
                    self.drivers[driver.id] = driver
                    self.driver_grid[i, j] += 1
                    spawned += 1
        self.drivers_spawned.append(spawned)

        # total_pressure = 0
        # cells_counted = 0
        # for i in range(GRID_SIZE):
        #     for j in range(GRID_SIZE):
        #         r, d = self.local_pressure(i,j)
        #         if r + 1 > 0:
        #             total_pressure += (d + EPS) / (r + 1)
        #             cells_counted += 1
        # avg_driver_pressure = total_pressure / max(1, cells_counted)
        # self.pressure_list.append(avg_driver_pressure)

    # -------------------------
    # Send requests
    # -------------------------
    def send_requests(self, rider):
        candidates = []
        for d in self.drivers.values():
            if d.matched or d.ask > rider.bid:
                continue
            dist = manhattan(d.loc, rider.loc)
            w = np.exp(-DIST_DECAY * dist)
            candidates.append((w, d))

        candidates.sort(key=lambda x: x[0], reverse=True)
        for _, d in candidates[:3]:
            d.inbox[rider.id] = rider
            rider.active_requests.add(d.id)

    # -------------------------
    # Matching
    # -------------------------
    def matching(self):
        for r in self.riders.values():
            if not r.matched:
                self.send_requests(r)

    # -------------------------
    # Driver choice
    # -------------------------
    def driver_choices(self):
        for d in sorted(self.drivers.values(), key=lambda x: x.ask):
            if d.matched or not d.inbox:
                continue

            d.inbox_age += 1
            if d.inbox_age >= d.patience:
                best = max(d.inbox.values(), key=lambda r: r.bid)
                if best.id in self.riders and not best.matched:
                    self.complete_match(d, best)
                    # d.inbox.clear()
                    # d.inbox_age = 0

    # -------------------------
    # Complete match
    # -------------------------
    def complete_match(self, d, r):
        bid = r.bid
        ask = d.ask

        d.matched = True
        r.matched = True

        i, j = d.loc
        self.market[i][j].record_trade(bid,ask)

        del self.drivers[d.id]
        del self.riders[r.id]
        self.driver_grid[i, j] -= 1
        self.rider_grid[r.loc[0], r.loc[1]] -= 1

    # -------------------------
    # Cleanup
    # -------------------------
    def cleanup(self):
        dr = dd = 0

        for r in list(self.riders.values()):
            r.wait += 1
            if r.expired():
                del self.riders[r.id]
                self.rider_grid[r.loc[0], r.loc[1]] -= 1
                dr += 1

        for d in list(self.drivers.values()):
            d.wait += 1
            if d.expired():
                del self.drivers[d.id]
                self.driver_grid[d.loc[0], d.loc[1]] -= 1
                dd += 1

        self.dropped_riders.append(dr)
        self.dropped_drivers.append(dd)

    # -------------------------
    # Step
    # -------------------------
    def step(self):
        self.spawn_riders()
        self.spawn_drivers()
        self.matching()
        self.driver_choices()
        self.cleanup()

        self.rider_counts.append(len(self.riders))
        self.driver_counts.append(len(self.drivers))
        self.avg_rates.append(np.mean([c.avg_bid() for row in self.market for c in row]))
        self.time += 1

    def run(self):
        for _ in range(SIM_MINUTES):
            self.step()

# =========================
# Run + Plot
# =========================
sim = Simulation()
sim.run()

fig, axes = plt.subplots(6, 1, figsize=(12, 20), sharex=True)

axes[0].plot(sim.rider_counts, label="Active Riders")
axes[0].plot(sim.driver_counts, label="Active Drivers")
axes[0].legend(); axes[0].set_title("Active Agents")

axes[1].plot(sim.riders_spawned, label="Riders Spawned")
axes[1].plot(sim.drivers_spawned, label="Drivers Spawned")
axes[1].legend(); axes[1].set_title("Spawned per Minute")

axes[2].plot(sim.avg_rates, label="Average Rate", color="green")
axes[2].legend(); axes[2].set_title("Average Rate")

axes[3].plot(sim.dropped_riders, label="Riders Dropped", color="red")
axes[3].legend(); axes[3].set_title("Dropped Riders")

axes[4].plot(sim.dropped_drivers, label="Drivers Dropped", color="purple")
axes[4].legend(); axes[4].set_title("Dropped Drivers")

axes[5].plot(sim.pressure_list, label = "Pressure", color="brown")
axes[5].legend(); axes[5].set_title("Pressure")

plt.tight_layout()
plt.show()
