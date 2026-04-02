#include "matching_engine.h"
#include <iostream>
#include <vector>
#include <random>
#include <chrono>

int main() {
    MatchingEngine engine;

    std::vector<int> driver_ids;
    std::vector<int> rider_ids;

    const int NUM_DRIVERS = 5000;
    const int NUM_RIDERS = 10000;

    std::mt19937 rng(42); // fixed seed
    std::uniform_real_distribution<double> lat_dist(43.69, 43.70);
    std::uniform_real_distribution<double> lon_dist(-79.33, -79.32);
    std::uniform_real_distribution<double> bid_dist(5.0, 50.0);

    // Add drivers
    for (int i = 0; i < NUM_DRIVERS; ++i){
        int int_id = engine.add_driver(i+1, lat_dist(rng), lon_dist(rng));
        driver_ids.push_back(int_id);
    }

    // Add riders
    for (int i = 0; i < NUM_RIDERS; ++i){
        int int_id = engine.add_rider(100000 + i, bid_dist(rng), lat_dist(rng), lon_dist(rng));
        rider_ids.push_back(int_id);
    }

    // Random driver interest simulation
    std::uniform_int_distribution<int> driver_pick(0, NUM_DRIVERS-1);
    std::uniform_int_distribution<int> rider_pick(0, NUM_RIDERS-1);

    for (int iter = 0; iter < 10000; ++iter){
        int d_idx = driver_pick(rng);
        int r_idx = rider_pick(rng);

        double ask = bid_dist(rng); // random ask
        engine.driver_interest(driver_ids[d_idx], rider_ids[r_idx], ask);

        if (iter % 500 == 0){ // every 500 interactions, make matches
            auto start = std::chrono::high_resolution_clock::now();
            engine.make_matches();
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end - start;
            std::cout << "make_matches() took " << diff.count() << " s\n";
        }
    }

    return 0;
}