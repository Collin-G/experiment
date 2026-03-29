#include "matching.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <curl/curl.h>
#include <sstream>
#include <thread>

// -------------------- CURL helper --------------------
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// -------------------- MatchingEngine --------------------
MatchingEngine::MatchingEngine() {
    // No router needed
}

MatchingEngine::~MatchingEngine() {
    stop();
}

// -------------------- H3 helpers --------------------
H3Index MatchingEngine::location_to_h3(const Location& loc, int res) const {
    LatLng coord = {loc.lat, loc.lon};
    H3Index cell;
    latLngToCell(&coord, res, &cell);
    return cell;
}

std::vector<H3Index> MatchingEngine::get_neighboring_cells(H3Index center, int radius) const {
    int64_t max_neighbors;
    maxGridDiskSize(radius, &max_neighbors);
    std::vector<H3Index> neighbors(max_neighbors);
    gridDisk(center, radius, neighbors.data());
    return neighbors;
}

// -------------------- Distance using OSRM --------------------
double MatchingEngine::calculate_distance(const Location& a, const Location& b) const {
    CURL* curl = curl_easy_init();
    if (!curl) return -1;

    std::ostringstream url;
    url << osrm_url_ << "/" << a.lon << "," << a.lat << ";" << b.lon << "," << b.lat
        << "?overview=false";

    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, url.str().c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) return -1;

    size_t dist_pos = response.find("\"distance\":");
    if (dist_pos == std::string::npos) return -1;

    dist_pos += 11;
    size_t end_pos = response.find(",", dist_pos);
    if (end_pos == std::string::npos) return -1;

    try {
        return std::stod(response.substr(dist_pos, end_pos - dist_pos));
    } catch (...) {
        return -1;
    }
}

// -------------------- Public API --------------------
void MatchingEngine::add_rider(int id, double bid, double lat, double lon) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (riders_.count(id)) return;

        Rider& rider = riders_[id];
        rider.id = id;
        rider.bid = bid;
        rider.loc = Location(lat, lon);
        rider.post_time = std::chrono::steady_clock::now();
        rider.state = State::OPEN;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_riders_.push(id);
        queue_cv_.notify_one();
    }

    std::cout << "Rider " << id << " added (bid: $" << bid << ")\n";
}


void MatchingEngine::add_rider(int id, double bid, double lat, double lon) {
    

}

void MatchingEngine::add_driver(int id, double ask, double lat, double lon) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (drivers_.count(id)) return;

    Driver& driver = drivers_[id];
    driver.id = id;
    driver.ask = ask;
    driver.loc = Location(lat, lon);
    driver.state = State::OPEN;

    H3Index cell = location_to_h3(driver.loc, H3_RES);
    drivers_by_cell_[cell].push_back(id);

    std::cout << "Driver " << id << " added (ask: $" << ask << ")\n";
}

void MatchingEngine::driver_accept(int driver_id, int rider_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto driver_it = drivers_.find(driver_id);
    auto rider_it = riders_.find(rider_id);
    if (driver_it == drivers_.end() || rider_it == riders_.end()) return;

    Driver& driver = driver_it->second;
    Rider& rider = rider_it->second;

    if (std::find(driver.inbox.begin(), driver.inbox.end(), rider_id) == driver.inbox.end())
        return;

    if (driver.state != State::OPEN || rider.state != State::OPEN) return;

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

// -------------------- Worker threads --------------------
void MatchingEngine::matching_worker() {
    while (running_) {
        int rider_id = -1;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { return !running_ || !pending_riders_.empty(); });
            if (!running_) break;
            rider_id = pending_riders_.front();
            pending_riders_.pop();
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        auto rider_it = riders_.find(rider_id);
        if (rider_it == riders_.end() || rider_it->second.state != State::OPEN) continue;

        const Rider& rider = rider_it->second;
        auto driver_ids = find_k_closest_drivers(rider, K);

        if (!driver_ids.empty()) send_offers(rider_id, driver_ids);
    }
}

std::vector<int> MatchingEngine::find_k_closest_drivers(const Rider& rider, int k) {
    std::vector<std::pair<double, int>> candidates;
    H3Index rider_cell = location_to_h3(rider.loc, H3_RES);
    std::vector<H3Index> neighboring_cells = get_neighboring_cells(rider_cell, SEARCH_RADIUS);

    for (H3Index cell : neighboring_cells) {
        auto cell_it = drivers_by_cell_.find(cell);
        if (cell_it == drivers_by_cell_.end()) continue;

        for (int driver_id : cell_it->second) {
            auto driver_it = drivers_.find(driver_id);
            if (driver_it == drivers_.end()) continue;

            const Driver& driver = driver_it->second;
            if (driver.state != State::OPEN || driver.ask > rider.bid) continue;

            double distance = calculate_distance(rider.loc, driver.loc);
            if (distance < 0) continue;

            candidates.emplace_back(distance, driver_id);
        }
    }

    std::sort(candidates.begin(), candidates.end());
    std::vector<int> result;
    for (int i = 0; i < std::min(k, (int)candidates.size()); i++)
        result.push_back(candidates[i].second);

    return result;
}

void MatchingEngine::send_offers(int rider_id, const std::vector<int>& driver_ids) {
    for (int driver_id : driver_ids) {
        auto it = drivers_.find(driver_id);
        if (it == drivers_.end()) continue;
        it->second.inbox.push_back(rider_id);
        std::cout << "DEBUG: Sent offer from rider " << rider_id << " to driver " << driver_id
                  << std::endl;
    }

    auto rider_it = riders_.find(rider_id);
    if (rider_it != riders_.end()) rider_it->second.pending_drivers = driver_ids;

    std::cout << "Sent " << driver_ids.size() << " offers for rider " << rider_id << "\n";
}

void MatchingEngine::cleanup_after_match(int rider_id, int driver_id) {
    auto driver_it = drivers_.find(driver_id);
    if (driver_it != drivers_.end()) {
        H3Index cell = location_to_h3(driver_it->second.loc, H3_RES);
        auto& cell_drivers = drivers_by_cell_[cell];
        cell_drivers.erase(std::remove(cell_drivers.begin(), cell_drivers.end(), driver_id),
                           cell_drivers.end());
    }

    std::vector<int> pending_drivers;
    auto rider_it = riders_.find(rider_id);
    if (rider_it != riders_.end()) pending_drivers = rider_it->second.pending_drivers;

    for (int other_driver_id : pending_drivers) {
        if (other_driver_id == driver_id) continue;
        auto other_driver_it = drivers_.find(other_driver_id);
        if (other_driver_it == drivers_.end()) continue;

        auto& inbox = other_driver_it->second.inbox;
        inbox.erase(std::remove(inbox.begin(), inbox.end(), rider_id), inbox.end());
    }

    drivers_.erase(driver_id);
    riders_.erase(rider_id);
}

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
                if (duration.count() >= TIMEOUT_SEC && rider.state == State::OPEN)
                    expired_riders.push_back(rider.id);
            }
        }

        for (int rider_id : expired_riders) {
            std::cout << "Rider " << rider_id << " expired (timeout)\n";
            rider_cancel(rider_id);
        }
    }
}

// -------------------- Debug / Control --------------------
void MatchingEngine::print_state() const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::cout << "\n=== MATCHING ENGINE STATE ===\n";
    std::cout << "Riders: " << riders_.size() << "\n";
    for (const auto& pair : riders_)
        std::cout << "  Rider " << pair.first << ": bid=$" << pair.second.bid
                  << ", state=" << static_cast<int>(pair.second.state.load())
                  << ", pending_drivers=" << pair.second.pending_drivers.size() << "\n";

    std::cout << "Drivers: " << drivers_.size() << "\n";
    for (const auto& pair : drivers_)
        std::cout << "  Driver " << pair.first << ": ask=$" << pair.second.ask
                  << ", state=" << static_cast<int>(pair.second.state.load())
                  << ", inbox=" << pair.second.inbox.size() << "\n";

    std::cout << "============================\n";
}

// -------------------- Start / Stop --------------------
void MatchingEngine::start(int num_threads) {
    if (running_) return;
    running_ = true;

    for (int i = 0; i < num_threads; ++i)
        workers_.emplace_back(&MatchingEngine::matching_worker, this);

    timeout_thread_ = std::thread(&MatchingEngine::timeout_worker, this);

    std::cout << "MatchingEngine started with " << num_threads << " threads\n";
}

void MatchingEngine::stop() {
    if (!running_) return;
    running_ = false;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_cv_.notify_all();
    }

    for (auto& thread : workers_)
        if (thread.joinable()) thread.join();

    if (timeout_thread_.joinable())
        timeout_thread_.join();

    workers_.clear();
    std::cout << "MatchingEngine stopped\n";
}
