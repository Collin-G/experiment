#pragma once

#include <unordered_map>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <condition_variable>
#include <h3/h3api.h>

// Forward declaration
class RoutingEngine;

struct Location {
    double lat;
    double lon;
    
    Location(double lat_ = 0, double lon_ = 0) : lat(lat_), lon(lon_) {}
};

// Convert to H3 LatLng
inline LatLng toLatLng(const Location& loc) {
    return {loc.lat, loc.lon};
}

enum class State { OPEN, MATCHED, CANCELLED, TIMEOUT };

struct Rider {
    int id;
    double bid;
    Location loc;
    std::atomic<State> state{State::OPEN};
    std::chrono::steady_clock::time_point post_time;
    std::vector<int> pending_drivers;
    bool processed{false};
    std::condition_variable processed_cv;
    
    Rider(int id_ = 0, double bid_ = 0, Location loc_ = Location()) 
        : id(id_), bid(bid_), loc(loc_) {}
};

struct Driver {
    int id;
    double ask;
    Location loc;
    std::atomic<State> state{State::OPEN};
    std::vector<int> inbox;
    std::mutex inbox_mutex;
    
    Driver(int id_ = 0, double ask_ = 0, Location loc_ = Location())
        : id(id_), ask(ask_), loc(loc_) {}
};

class MatchingEngine {
public:
    MatchingEngine(RoutingEngine* router = nullptr, int num_threads = 4);
    ~MatchingEngine();
    
    // Public API
    void start();
    void stop();
    
    void add_rider(int id, double bid, double lat, double lon);
    void add_driver(int id, double ask, double lat, double lon);
    void driver_accept(int driver_id, int rider_id);
    void driver_cancel(int driver_id);
    void rider_cancel(int rider_id);
    
    // Debug/Test helpers
    void debug_print_state();
    void wait_for_rider_processed(int rider_id, int timeout_ms = 1000);
    
private:
    // Worker threads
    void matching_worker();
    void timeout_worker();
    
    // Helper functions
    std::vector<int> find_k_closest_drivers(const Rider& rider, int k);
    void send_offers(int rider_id, const std::vector<int>& driver_ids);
    void cleanup_after_match(int rider_id, int driver_id);
    void mark_rider_processed(int rider_id);
    
    // H3 functions
    H3Index location_to_h3(const Location& loc, int res);
    std::vector<H3Index> get_neighboring_cells(H3Index center, int radius);
    double calculate_distance(const Location& a, const Location& b);
    
    // Data storage
    std::unordered_map<int, Rider> riders_;
    std::unordered_map<int, Driver> drivers_;
    std::unordered_map<H3Index, std::vector<int>> drivers_by_cell_;
    
    // Threading
    std::vector<std::thread> workers_;
    std::thread timeout_thread_;
    std::atomic<bool> running_{false};
    
    // Synchronization
    std::mutex riders_mutex_;
    std::mutex drivers_mutex_;
    std::mutex cells_mutex_;
    std::mutex queue_mutex_;
    
    // Queues
    std::queue<int> pending_riders_;
    
    // External
    RoutingEngine* router_;
    
    // Configuration
    static constexpr int H3_RES = 10;
    static constexpr int K = 5;
    static constexpr int TIMEOUT_SEC = 300;
    int num_threads_;
};