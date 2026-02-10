#include "matching.h"
#include "router.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdint>

// Constructor/Destructor
MatchingEngine::MatchingEngine(RoutingEngine* router, int num_threads) 
    : router_(router), num_threads_(num_threads) {}

MatchingEngine::~MatchingEngine() {
    stop();
}

// H3 helper functions
H3Index MatchingEngine::location_to_h3(const Location& loc, int res) {
    LatLng coord = {loc.lat, loc.lon};
    H3Index cell;
    latLngToCell(&coord, res, &cell);
    return cell;
}

std::vector<H3Index> MatchingEngine::get_neighboring_cells(H3Index center, int radius) {
    int64_t max_neighbors;
    maxGridDiskSize(radius, &max_neighbors);
    std::vector<H3Index> neighbors(max_neighbors);
    gridDisk(center, radius, neighbors.data());
    return neighbors;
}

double MatchingEngine::calculate_distance(const Location& a, const Location& b) {
    if (router_) {
        return router_->route(a.lat, a.lon, b.lat, b.lon);
    } else {
        // Haversine fallback
        double lat1 = a.lat * M_PI / 180.0;
        double lon1 = a.lon * M_PI / 180.0;
        double lat2 = b.lat * M_PI / 180.0;
        double lon2 = b.lon * M_PI / 180.0;
        
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        
        double haversine = sin(dlat/2) * sin(dlat/2) + 
                          cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
        
        return 2 * 6371000 * atan2(sqrt(haversine), sqrt(1 - haversine));
    }
}

// Public API
void MatchingEngine::add_rider(int id, double bid, double lat, double lon) {
    std::unique_lock<std::mutex> lock(riders_mutex_);
    
    auto result = riders_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(id, bid, Location(lat, lon))
    );
    
    if (!result.second) {
        std::cerr << "Rider " << id << " already exists\n";
        return;
    }
    
    Rider& rider = result.first->second;
    rider.post_time = std::chrono::steady_clock::now();
    
    {
        std::lock_guard<std::mutex> qlock(queue_mutex_);
        pending_riders_.push(id);
    }
    
    std::cout << "Rider " << id << " added (bid: $" << bid << ")\n";
}

void MatchingEngine::add_driver(int id, double ask, double lat, double lon) {
    std::unique_lock<std::mutex> lock(drivers_mutex_);
    
    auto result = drivers_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(id, ask, Location(lat, lon))
    );
    
    if (!result.second) {
        std::cerr << "Driver " << id << " already exists\n";
        return;
    }
    
    Driver& driver = result.first->second;
    
    H3Index cell = location_to_h3(driver.loc, H3_RES);
    
    {
        std::lock_guard<std::mutex> clock(cells_mutex_);
        drivers_by_cell_[cell].push_back(id);
    }
    
    std::cout << "Driver " << id << " added (ask: $" << ask << ")\n";
}

void MatchingEngine::driver_accept(int driver_id, int rider_id) {
    std::unique_lock<std::mutex> rlock(riders_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> dlock(drivers_mutex_, std::defer_lock);
    std::lock(rlock, dlock);
    
    auto driver_it = drivers_.find(driver_id);
    if (driver_it == drivers_.end()) {
        std::cout << "Driver " << driver_id << " not found\n";
        return;
    }
    
    auto rider_it = riders_.find(rider_id);
    if (rider_it == riders_.end()) {
        std::cout << "Rider " << rider_id << " not found\n";
        return;
    }
    
    Driver& driver = driver_it->second;
    Rider& rider = rider_it->second;
    
    // Check inbox
    {
        std::lock_guard<std::mutex> inbox_lock(driver.inbox_mutex);
        auto it = std::find(driver.inbox.begin(), driver.inbox.end(), rider_id);
        if (it == driver.inbox.end()) {
            std::cout << "Rider " << rider_id << " not in driver " << driver_id << "'s inbox\n";
            return;
        }
    }
    
    // Check states
    if (driver.state != State::OPEN) {
        std::cout << "Driver " << driver_id << " not open (state: " 
                  << static_cast<int>(driver.state.load()) << ")\n";
        return;
    }
    
    if (rider.state != State::OPEN) {
        std::cout << "Rider " << rider_id << " not open (state: " 
                  << static_cast<int>(rider.state.load()) << ")\n";
        return;
    }
    
    // Update states
    driver.state = State::MATCHED;
    rider.state = State::MATCHED;
    
    // Clean up
    cleanup_after_match(rider_id, driver_id);
    
    std::cout << "✓ MATCH: Driver " << driver_id << " accepted Rider " << rider_id 
              << " ($" << driver.ask << " <= $" << rider.bid << ")\n";
}

void MatchingEngine::driver_cancel(int driver_id) {
    std::lock_guard<std::mutex> lock(drivers_mutex_);
    auto it = drivers_.find(driver_id);
    if (it == drivers_.end()) return;
    
    it->second.state = State::CANCELLED;
    
    H3Index cell = location_to_h3(it->second.loc, H3_RES);
    {
        std::lock_guard<std::mutex> clock(cells_mutex_);
        auto& cell_drivers = drivers_by_cell_[cell];
        cell_drivers.erase(std::remove(cell_drivers.begin(), cell_drivers.end(), driver_id), 
                          cell_drivers.end());
    }
    
    drivers_.erase(it);
    std::cout << "Driver " << driver_id << " cancelled\n";
}

void MatchingEngine::rider_cancel(int rider_id) {
    // Get rider and pending drivers
    std::vector<int> pending_drivers;
    {
        std::lock_guard<std::mutex> lock(riders_mutex_);
        auto it = riders_.find(rider_id);
        if (it == riders_.end()) return;
        
        it->second.state = State::CANCELLED;
        pending_drivers = it->second.pending_drivers;
        
        riders_.erase(it);
    }
    
    // Clean up inboxes
    std::lock_guard<std::mutex> dlock(drivers_mutex_);
    for (int driver_id : pending_drivers) {
        auto driver_it = drivers_.find(driver_id);
        if (driver_it == drivers_.end()) continue;
        
        std::lock_guard<std::mutex> inbox_lock(driver_it->second.inbox_mutex);
        auto& inbox = driver_it->second.inbox;
        inbox.erase(std::remove(inbox.begin(), inbox.end(), rider_id), inbox.end());
    }
    
    std::cout << "Rider " << rider_id << " cancelled\n";
}

// Matching worker
void MatchingEngine::matching_worker() {
    while (running_) {
        int rider_id = -1;
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!pending_riders_.empty()) {
                rider_id = pending_riders_.front();
                pending_riders_.pop();
            }
        }
        
        if (rider_id == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // Get rider
        Rider* rider = nullptr;
        {
            std::lock_guard<std::mutex> lock(riders_mutex_);
            auto it = riders_.find(rider_id);
            if (it == riders_.end() || it->second.state != State::OPEN) {
                continue;
            }
            rider = &it->second;
        }
        
        if (!rider) continue;
        
        // Find drivers
        auto driver_ids = find_k_closest_drivers(*rider, K);
        
        if (!driver_ids.empty()) {
            send_offers(rider_id, driver_ids);
        }
        
        // Mark as processed
        mark_rider_processed(rider_id);
    }
}

// Find closest drivers
std::vector<int> MatchingEngine::find_k_closest_drivers(const Rider& rider, int k) {
    std::vector<std::pair<double, int>> candidates;
    
    H3Index rider_cell = location_to_h3(rider.loc, H3_RES);
    std::vector<H3Index> neighboring_cells = get_neighboring_cells(rider_cell, 1);
    
    std::lock_guard<std::mutex> clock(cells_mutex_);
    std::lock_guard<std::mutex> dlock(drivers_mutex_);
    
    for (H3Index cell : neighboring_cells) {
        auto cell_it = drivers_by_cell_.find(cell);
        if (cell_it == drivers_by_cell_.end()) continue;
        
        for (int driver_id : cell_it->second) {
            auto driver_it = drivers_.find(driver_id);
            if (driver_it == drivers_.end()) continue;
            
            const Driver& driver = driver_it->second;
            
            if (driver.state != State::OPEN) continue;
            if (driver.ask > rider.bid) continue;
            
            double distance = calculate_distance(rider.loc, driver.loc);
            if (distance < 0) continue;
            
            candidates.emplace_back(distance, driver_id);
        }
    }
    
    std::sort(candidates.begin(), candidates.end());
    
    std::vector<int> result;
    for (int i = 0; i < std::min(k, (int)candidates.size()); i++) {
        result.push_back(candidates[i].second);
    }
    
    return result;
}

void MatchingEngine::send_offers(int rider_id, const std::vector<int>& driver_ids) {
    std::scoped_lock lock(drivers_mutex_, riders_mutex_);  // Lock both safely
    
    for (int driver_id : driver_ids) {
        auto it = drivers_.find(driver_id);
        if (it == drivers_.end()) continue;
        
        Driver& driver = it->second;
        std::lock_guard<std::mutex> inbox_lock(driver.inbox_mutex);
        driver.inbox.push_back(rider_id);
    }
    
    // Update rider's pending drivers list
    auto rider_it = riders_.find(rider_id);
    if (rider_it != riders_.end()) {
        rider_it->second.pending_drivers = driver_ids;
        std::cout << "Sent " << driver_ids.size() << " offers for rider " << rider_id << "\n";
    }
}

void MatchingEngine::mark_rider_processed(int rider_id) {
    std::lock_guard<std::mutex> lock(riders_mutex_);
    auto it = riders_.find(rider_id);
    if (it != riders_.end()) {
        it->second.processed = true;
        it->second.processed_cv.notify_all();
    }
}

void MatchingEngine::cleanup_after_match(int rider_id, int driver_id) {
    // ASSUMPTION: Caller already holds riders_mutex_ and drivers_mutex_
    
    // Remove driver from H3 index
    auto driver_it = drivers_.find(driver_id);
    if (driver_it != drivers_.end()) {
        H3Index cell = location_to_h3(driver_it->second.loc, H3_RES);
        
        std::lock_guard<std::mutex> clock(cells_mutex_);  // Different mutex, OK
        auto& cell_drivers = drivers_by_cell_[cell];
        cell_drivers.erase(std::remove(cell_drivers.begin(), cell_drivers.end(), driver_id), 
                          cell_drivers.end());
    }
    
    // Get rider's pending drivers
    std::vector<int> pending_drivers;
    auto rider_it = riders_.find(rider_id);
    if (rider_it != riders_.end()) {
        pending_drivers = rider_it->second.pending_drivers;
    }
    
    // Clean up other drivers' inboxes
    for (int other_driver_id : pending_drivers) {
        if (other_driver_id == driver_id) continue;
        
        auto other_driver_it = drivers_.find(other_driver_id);
        if (other_driver_it == drivers_.end()) continue;
        
        std::lock_guard<std::mutex> inbox_lock(other_driver_it->second.inbox_mutex);
        auto& inbox = other_driver_it->second.inbox;
        inbox.erase(std::remove(inbox.begin(), inbox.end(), rider_id), inbox.end());
    }
    
    // Remove from maps (already have locks)
    drivers_.erase(driver_id);
    riders_.erase(rider_id);
}

// Timeout worker
void MatchingEngine::timeout_worker() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        auto now = std::chrono::steady_clock::now();
        std::vector<int> expired_riders;
        
        {
            std::lock_guard<std::mutex> lock(riders_mutex_);
            for (const auto& pair : riders_) {
                const Rider& rider = pair.second;
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - rider.post_time);
                
                if (duration.count() >= TIMEOUT_SEC && rider.state == State::OPEN) {
                    expired_riders.push_back(rider.id);
                }
            }
        }
        
        for (int rider_id : expired_riders) {
            std::cout << "Rider " << rider_id << " expired (timeout)\n";
            rider_cancel(rider_id);
        }
    }
}

// Debug/Test helpers
void MatchingEngine::debug_print_state() {
    // For const methods, we need to use const_cast or mutable mutexes
    // Simplest: remove const from method declaration in header
    // Change: void debug_print_state() const;
    // To:     void debug_print_state();
    
    // Or use const_cast (not ideal but works):
    std::unique_lock<std::mutex> rlock(const_cast<std::mutex&>(riders_mutex_));
    std::unique_lock<std::mutex> dlock(const_cast<std::mutex&>(drivers_mutex_));
    
    std::cout << "\n=== DEBUG STATE ===\n";
    std::cout << "Riders: " << riders_.size() << "\n";
    for (const auto& pair : riders_) {
        std::cout << "  Rider " << pair.first << ": bid=$" << pair.second.bid
                  << ", state=" << static_cast<int>(pair.second.state.load())
                  << ", pending=" << pair.second.pending_drivers.size() << "\n";
    }
    
    std::cout << "Drivers: " << drivers_.size() << "\n";
    for (const auto& pair : drivers_) {
        std::lock_guard<std::mutex> inbox_lock(const_cast<std::mutex&>(pair.second.inbox_mutex));
        std::cout << "  Driver " << pair.first << ": ask=$" << pair.second.ask
                  << ", state=" << static_cast<int>(pair.second.state.load())
                  << ", inbox=" << pair.second.inbox.size() << "\n";
    }
    std::cout << "==================\n";
}

void MatchingEngine::wait_for_rider_processed(int rider_id, int timeout_ms) {
    std::unique_lock<std::mutex> lock(riders_mutex_);
    auto it = riders_.find(rider_id);
    if (it == riders_.end()) return;
    
    it->second.processed_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
        [&]() { return it->second.processed || !running_; });
}

// Start/stop
void MatchingEngine::start() {
    if (running_) return;
    
    running_ = true;
    
    for (int i = 0; i < num_threads_; ++i) {
        workers_.emplace_back(&MatchingEngine::matching_worker, this);
    }
    
    timeout_thread_ = std::thread(&MatchingEngine::timeout_worker, this);
    
    std::cout << "MatchingEngine started with " << num_threads_ << " threads\n";
}

void MatchingEngine::stop() {
    if (!running_) return;
    
    running_ = false;
    
    // Wake up any waiting threads
    {
        std::lock_guard<std::mutex> lock(riders_mutex_);
        for (auto& pair : riders_) {
            pair.second.processed_cv.notify_all();
        }
    }
    
    for (auto& thread : workers_) {
        if (thread.joinable()) thread.join();
    }
    
    if (timeout_thread_.joinable()) {
        timeout_thread_.join();
    }
    
    workers_.clear();
    std::cout << "MatchingEngine stopped\n";
}