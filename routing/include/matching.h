#pragma once

#include <atomic>          // std::atomic
#include <thread>          // std::thread
#include <vector>          // std::vector
#include <unordered_map>   // std::unordered_map
#include <mutex>           // std::mutex, std::lock_guard
#include <condition_variable> // blocking queues
#include <queue>           // internal queue storage
#include <memory>          // std::shared_ptr (if used later)
#include <cstddef>         // size_t
#include <h3api.h> // Uber H3 library
#include <algorithm>


enum class State { OPEN, MATCHED, CANCELLED };


struct Location {
    double lat;
    double lon;
};

struct Rider {
    int id;
    double bid;
    std::atomic<State> state{State::OPEN};
    Location loc;
};

struct Driver {
    int id;
    double ask;
    std::atomic<State> state{State::OPEN};
    Location loc;
    std::vector<int> inbox;
    std::mutex inbox_mutex;
};

struct Event {
    enum class Type { RIDER_MATCHED, DRIVER_MATCHED } type;
    int rider_id;
    int driver_id;
};

class MatchingEngine {
public:
    MatchingEngine(int num_matching_threads = 4);
    ~MatchingEngine();

    void start(); // starts worker threads
    void stop();  // stops threads cleanly

    // Public API for external calls
    void post_rider(int rider_id, double bid,  Location loc);
    void post_driver(int driver_id, double ask,Location loc);
    void driver_accept(int driver_id, int rider_id);
    void driver_cancel(int player_id);
    bool try_offer(std::shared_ptr<Driver> driver, std::shared_ptr<Rider> rider);
    void clean_driver_inbox(std::shared_ptr<Driver> driver);

   
    // H3 driver index
    std::unordered_map<H3Index, std::vector<std::shared_ptr<Driver>>> drivers_by_cell;
    std::mutex cell_mutex;

    constexpr int H3_RESOLUTION = 9; // tune for city-level grids
    constexpr int K = 5;             // number of live offers per rider


private:
    void matching_worker(); // infinite loop for matching riders → drivers
    void event_worker();    // infinite loop for events

    std::unordered_map<int, std::shared_ptr<Rider>> riders;
    std::unordered_map<int, std::shared_ptr<Driver>> drivers;

    std::mutex riders_mutex;
    std::mutex drivers_mutex;

    BlockingQueue<std::shared_ptr<Rider>> match_queue;
    BlockingQueue<Event> event_queue;

    std::vector<std::thread> worker_threads;
    std::thread event_thread;

    std::atomic<bool> running{false};
};


template<typename T>
class BlockingQueue {
    std::queue<T> q;
    std::mutex m;
    std::condition_variable cv;
public:
    void push(T item) {
        std::lock_guard lk(m);
        q.push(std::move(item));
        cv.notify_one();
    }

    T pop() {
        std::unique_lock lk(m);
        cv.wait(lk, [&]{ return !q.empty(); });
        T item = std::move(q.front());
        q.pop();
        return item;
    }
};