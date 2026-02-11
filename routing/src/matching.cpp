#include "matching.h"
#include "router.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdint>

// Constructor/Destructor
MatchingEngine::MatchingEngine(RoutingEngine* router) 
    : router_(router) {}

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
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (riders_.count(id)) {
            std::cerr << "Rider " << id << " already exists\n";
            return;
        }
        
        riders_.emplace(std::piecewise_construct,
                std::forward_as_tuple(id),
                std::forward_as_tuple(id, bid, Location(lat, lon)));
        riders_[id].post_time = std::chrono::steady_clock::now();
    }
    
    // Add to queue (separate lock for queue)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_riders_.push(id);
        queue_cv_.notify_one();
    }
    
    std::cout << "Rider " << id << " added (bid: $" << bid << ")\n";
}

void MatchingEngine::add_driver(int id, double ask, double lat, double lon) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (drivers_.count(id)) {
        std::cerr << "Driver " << id << " already exists\n";
        return;
    }
    
    drivers_.emplace(std::piecewise_construct,
                 std::forward_as_tuple(id),
                 std::forward_as_tuple(id, ask, Location(lat, lon)));
    
    H3Index cell = location_to_h3(drivers_[id].loc, H3_RES);
    drivers_by_cell_[cell].push_back(id);
    
    std::cout << "Driver " << id << " added (ask: $" << ask << ")\n";
}

void MatchingEngine::driver_accept(int driver_id, int rider_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto driver_it = drivers_.find(driver_id);
    auto rider_it = riders_.find(rider_id);
    
    if (driver_it == drivers_.end()) {
        std::cout << "Driver " << driver_id << " not found\n";
        return;
    }
    
    if (rider_it == riders_.end()) {
        std::cout << "Rider " << rider_id << " not found\n";
        return;
    }
    
    Driver& driver = driver_it->second;
    Rider& rider = rider_it->second;
    
    // Check if rider is in driver's inbox
    auto inbox_it = std::find(driver.inbox.begin(), driver.inbox.end(), rider_id);
    if (inbox_it == driver.inbox.end()) {
        std::cout << "Rider " << rider_id << " not in driver " << driver_id << "'s inbox\n";
        return;
    }
    
    // Check both are open
    if (driver.state != State::OPEN || rider.state != State::OPEN) {
        std::cout << "Cannot match - not both open\n";
        return;
    }
    
    // Match them
    driver.state = State::MATCHED;
    rider.state = State::MATCHED;
    
    cleanup_after_match(rider_id, driver_id);
    
    std::cout << "✓ MATCH: Driver " << driver_id << " accepted Rider " << rider_id 
              << " ($" << driver.ask << " <= $" << rider.bid << ")\n";
}

void MatchingEngine::driver_cancel(int driver_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = drivers_.find(driver_id);
    if (it == drivers_.end()) return;
    
    it->second.state = State::CANCELLED;
    
    // Remove from H3 index
    H3Index cell = location_to_h3(it->second.loc, H3_RES);
    auto& cell_drivers = drivers_by_cell_[cell];
    cell_drivers.erase(std::remove(cell_drivers.begin(), cell_drivers.end(), driver_id), 
                      cell_drivers.end());
    
    drivers_.erase(it);
    std::cout << "Driver " << driver_id << " cancelled\n";
}

void MatchingEngine::rider_cancel(int rider_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = riders_.find(rider_id);
    if (it == riders_.end()) return;
    
    it->second.state = State::CANCELLED;
    
    // Clean up from drivers' inboxes
    for (int driver_id : it->second.pending_drivers) {
        auto driver_it = drivers_.find(driver_id);
        if (driver_it != drivers_.end()) {
            auto& inbox = driver_it->second.inbox;
            inbox.erase(std::remove(inbox.begin(), inbox.end(), rider_id), inbox.end());
        }
    }
    
    riders_.erase(it);
    std::cout << "Rider " << rider_id << " cancelled\n";
}

// Matching worker
void MatchingEngine::matching_worker() {
    while (running_) {
        int rider_id = -1;
        
        // Wait for a rider
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { 
                return !running_ || !pending_riders_.empty(); 
            });
            
            if (!running_) break;
            
            rider_id = pending_riders_.front();
            pending_riders_.pop();
        }
        
        if (rider_id == -1) continue;
        
        // Process the rider (with data lock)
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        auto rider_it = riders_.find(rider_id);
        if (rider_it == riders_.end() || rider_it->second.state != State::OPEN) {
            continue;
        }
        
        const Rider& rider = rider_it->second;
        auto driver_ids = find_k_closest_drivers(rider, K);
        
        if (!driver_ids.empty()) {
            send_offers(rider_id, driver_ids);
        }
    }
}

// Find closest drivers
std::vector<int> MatchingEngine::find_k_closest_drivers(const Rider& rider, int k) {
    std::vector<std::pair<double, int>> candidates;
    
    H3Index rider_cell = location_to_h3(rider.loc, H3_RES);
    std::vector<H3Index> neighboring_cells = get_neighboring_cells(rider_cell, 1);
    
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
    // Add rider to drivers' inboxes
    for (int driver_id : driver_ids) {
        auto it = drivers_.find(driver_id);
        if (it == drivers_.end()) continue;
        it->second.inbox.push_back(rider_id);
    }
    
    // Update rider's pending drivers
    auto rider_it = riders_.find(rider_id);
    if (rider_it != riders_.end()) {
        rider_it->second.pending_drivers = driver_ids;
        std::cout << "Sent " << driver_ids.size() << " offers for rider " << rider_id << "\n";
    }
}

void MatchingEngine::cleanup_after_match(int rider_id, int driver_id) {
    // Remove driver from H3 index
    auto driver_it = drivers_.find(driver_id);
    if (driver_it != drivers_.end()) {
        H3Index cell = location_to_h3(driver_it->second.loc, H3_RES);
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
        
        auto& inbox = other_driver_it->second.inbox;
        inbox.erase(std::remove(inbox.begin(), inbox.end(), rider_id), inbox.end());
    }
    
    // Remove from maps
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
            std::lock_guard<std::mutex> lock(data_mutex_);
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

// Debug
void MatchingEngine::print_state() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::cout << "\n=== MATCHING ENGINE STATE ===\n";
    std::cout << "Riders: " << riders_.size() << "\n";
    for (const auto& pair : riders_) {
        std::cout << "  Rider " << pair.first << ": bid=$" << pair.second.bid
                  << ", state=" << static_cast<int>(pair.second.state.load())
                  << ", pending_drivers=" << pair.second.pending_drivers.size() << "\n";
    }
    
    std::cout << "Drivers: " << drivers_.size() << "\n";
    for (const auto& pair : drivers_) {
        std::cout << "  Driver " << pair.first << ": ask=$" << pair.second.ask
                  << ", state=" << static_cast<int>(pair.second.state.load())
                  << ", inbox=" << pair.second.inbox.size() << "\n";
    }
    std::cout << "============================\n";
}

// Start/stop
void MatchingEngine::start(int num_threads) {
    if (running_) return;
    
    running_ = true;
    
    // Start matching workers
    for (int i = 0; i < num_threads; ++i) {
        workers_.emplace_back(&MatchingEngine::matching_worker, this);
    }
    
    // Start timeout worker
    timeout_thread_ = std::thread(&MatchingEngine::timeout_worker, this);
    
    std::cout << "MatchingEngine started with " << num_threads << " threads\n";
}

void MatchingEngine::stop() {
    if (!running_) return;
    
    running_ = false;
    
    // Wake up all workers
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_cv_.notify_all();
    }
    
    // Join all threads
    for (auto& thread : workers_) {
        if (thread.joinable()) thread.join();
    }
    
    if (timeout_thread_.joinable()) {
        timeout_thread_.join();
    }
    
    workers_.clear();
    std::cout << "MatchingEngine stopped\n";
}