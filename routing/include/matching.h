#pragma once

#include <unordered_map>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <h3/h3api.h>
#include <string>

// No forward declaration for RoutingEngine needed anymore

struct Location {
    double lat;
    double lon;

    Location(double lat_ = 0, double lon_ = 0) : lat(lat_), lon(lon_) {}
};

enum class State { OPEN, MATCHED, CANCELLED };

struct Rider {
    int id;
    double bid;
    Location loc;
    std::atomic<State> state{State::OPEN};
    std::chrono::steady_clock::time_point post_time;
    std::vector<int> pending_drivers;

    Rider(int id_ = 0, double bid_ = 0, Location loc_ = Location())
        : id(id_), bid(bid_), loc(loc_) {}
};

struct Driver {
    int id;
    double ask;
    Location loc;
    std::atomic<State> state{State::OPEN};
    std::vector<int> inbox;

    Driver(int id_ = 0, double ask_ = 0, Location loc_ = Location())
        : id(id_), ask(ask_), loc(loc_) {}
};

class MatchingEngine {
public:
    MatchingEngine();  // No router needed
    ~MatchingEngine();

    void start(int num_threads = 4);
    void stop();

    void add_rider(int id, double bid, double lat, double lon);
    void add_driver(int id, double ask, double lat, double lon);
    void driver_accept(int driver_id, int rider_id);
    void driver_cancel(int driver_id);
    void rider_cancel(int rider_id);

    void print_state() const;

private:
    void matching_worker();
    void timeout_worker();

    std::vector<int> find_k_closest_drivers(const Rider& rider, int k);
    void send_offers(int rider_id, const std::vector<int>& driver_ids);
    void cleanup_after_match(int rider_id, int driver_id);

    H3Index location_to_h3(const Location& loc, int res) const;
    std::vector<H3Index> get_neighboring_cells(H3Index center, int radius) const;
    double calculate_distance(const Location& a, const Location& b) const;

    std::unordered_map<int, Rider> riders_;
    std::unordered_map<int, Driver> drivers_;
    std::unordered_map<H3Index, std::vector<int>> drivers_by_cell_;

    std::vector<std::thread> workers_;
    std::thread timeout_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex data_mutex_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::queue<int> pending_riders_;

    // Configuration
    static constexpr int H3_RES = 8;
    static constexpr int SEARCH_RADIUS = 2;
    static constexpr int K = 5;
    static constexpr int TIMEOUT_SEC = 300;

    // OSRM service
    std::string osrm_url_ = "http://localhost:5000/route/v1/driving";
};
