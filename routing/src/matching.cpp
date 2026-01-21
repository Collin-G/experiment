#include "matching.h"

// ---- Modified post_driver ----
void MatchingEngine::post_driver(int driver_id, double ask, Location loc){
    auto driver = std::make_shared<Driver>();
    driver->id = driver_id;
    driver->loc = loc;
    driver->ask = ask;
    driver->state.store(State::OPEN);

    {
        std::lock_guard<std::mutex> lk(drivers_mutex);
        drivers[driver_id] = driver;
    }

    // Add to H3 index
    H3Index cell = geo_to_h3(driver->loc.lat, driver->loc.lon, H3_RESOLUTION);
    {
        std::lock_guard<std::mutex> lk(cell_mutex);
        drivers_by_cell[cell].push_back(driver);
    }
}

// ---- Modified matching_worker ----
void MatchingEngine::matching_worker() {
    while (running.load()) {
        auto rider = match_queue.pop();
        if (!rider || !running.load()) continue;
        if (rider->state.load() != State::OPEN) continue;

        // 1. Get rider's H3 cell and nearby cells
        H3Index rider_cell = geo_to_h3(rider->loc.lat, rider->loc.lon, H3_RESOLUTION);
        H3Index nearby[K * 3]; // allocate for k-ring (max size depends on kRing)
        int n_cells = kRing(rider_cell, 1, nearby); // radius = 1 cell

        std::vector<std::shared_ptr<Driver>> candidates;

        {
            std::lock_guard<std::mutex> lk(cell_mutex);
            for (int i = 0; i < n_cells; ++i) {
                auto it = drivers_by_cell.find(nearby[i]);
                if (it == drivers_by_cell.end()) continue;

                for (auto& driver : it->second) {
                    if (driver->state.load() != State::OPEN) continue;
                    if (driver->ask > rider->bid) continue;
                    candidates.push_back(driver);
                    if (candidates.size() == K) break;
                }
                if (candidates.size() == K) break;
            }
        }

        // 2. Send offers
        for (auto& driver : candidates) {
            if (!running.load()) break;
            try_offer(driver, rider);
        }
    }
}

// ---- try_offer remains same, optionally clean inbox first ----
bool MatchingEngine::try_offer(std::shared_ptr<Driver> driver, std::shared_ptr<Rider> rider){
    clean_driver_inbox(driver);

    if (driver->state.load() != State::OPEN) return false;
    if (rider->state.load() != State::OPEN) return false;

    std::lock_guard<std::mutex> lk(driver->inbox_mutex);
    for (int rid : driver->inbox) if (rid == rider->id) return false;
    driver->inbox.push_back(rider->id);
    return true;
}

// ---- driver_accept ----
void MatchingEngine::driver_accept(int driver_id, int rider_id){
    std::shared_ptr<Driver> driver;
    std::shared_ptr<Rider> rider;

    {
        std::lock_guard<std::mutex> lk(drivers_mutex);
        auto it = drivers.find(driver_id);
        if (it == drivers.end()) return;
        driver = it->second;
    }

    {
        std::lock_guard<std::mutex> lk(riders_mutex);
        auto it = riders.find(rider_id);
        if (it == riders.end()) return;
        rider = it->second;
    }

    {
        std::lock_guard<std::mutex> inbox_lk(driver->inbox_mutex);
        bool offered = std::find(driver->inbox.begin(), driver->inbox.end(), rider_id) != driver->inbox.end();
        if (!offered) return;

        State expected = State::OPEN;
        if (!driver->state.compare_exchange_strong(expected, State::MATCHED)) return;

        expected = State::OPEN;
        if (!rider->state.compare_exchange_strong(expected, State::MATCHED)) {
            driver->state.store(State::OPEN); // rollback
            return;
        }
    }

    // Remove from global maps
    {
        std::lock_guard<std::mutex> dlk(drivers_mutex);
        drivers.erase(driver_id);
    }
    {
        std::lock_guard<std::mutex> rlk(riders_mutex);
        riders.erase(rider_id);
    }

    // Remove driver from H3 index
    {
        std::lock_guard<std::mutex> lk(cell_mutex);
        H3Index cell = geo_to_h3(driver->loc.lat, driver->loc.lon, H3_RESOLUTION);
        auto& vec = drivers_by_cell[cell];
        vec.erase(std::remove(vec.begin(), vec.end(), driver), vec.end());
    }

    event_queue.push(Event{Event::Type::DRIVER_MATCHED, rider_id, driver_id});
}

// ---- driver_cancel ----
void MatchingEngine::driver_cancel(int driver_id){
    std::shared_ptr<Driver> driver;
    {
        std::lock_guard<std::mutex> lk(drivers_mutex);
        auto it = drivers.find(driver_id);
        if (it == drivers.end()) return;
        driver = it->second;
    }

    State expected = State::OPEN;
    if (!driver->state.compare_exchange_strong(expected, State::CANCELLED)) return;

    {
        std::lock_guard<std::mutex> lk(drivers_mutex);
        drivers.erase(driver_id);
    }

    // Remove from H3 index
    {
        std::lock_guard<std::mutex> lk(cell_mutex);
        H3Index cell = geo_to_h3(driver->loc.lat, driver->loc.lon, H3_RESOLUTION);
        auto& vec = drivers_by_cell[cell];
        vec.erase(std::remove(vec.begin(), vec.end(), driver), vec.end());
    }
}

// ---- event_worker ----
void MatchingEngine::event_worker() {
    while (running.load()) {
        Event ev = event_queue.pop();
        if (!running.load()) break;

        switch (ev.type){
            case Event::Type::DRIVER_MATCHED:
                // notify external system
                break;
            case Event::Type::RIDER_MATCHED:
                break;
        }
    }
}

// ---- start & stop remain same ----


void MatchingEngine::start() {
    running.store(true);

    // Start matching workers
    for (int i = 0; i < worker_threads.capacity(); ++i) {
        worker_threads.emplace_back(&MatchingEngine::matching_worker, this);
    }

    // Start event worker
    event_thread = std::thread(&MatchingEngine::event_worker, this);
}

void MatchingEngine::stop() {
    running.store(false);

    // Wake all threads so they can exit
    for (size_t i = 0; i < worker_threads.size(); ++i) {
        match_queue.push(nullptr);
    }

    event_queue.push(Event{});

    for (auto& t : worker_threads) {
        if (t.joinable()) t.join();
    }

    if (event_thread.joinable()) {
        event_thread.join();
    }
}
