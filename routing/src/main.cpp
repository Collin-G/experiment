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
    const int NUM_RIDERS = 5000;

    std::mt19937 rng(42); // fixed seed
    std::uniform_real_distribution<double> lat_dist(43.69, 43.70);
    std::uniform_real_distribution<double> lon_dist(-79.33, -79.32);
    std::uniform_real_distribution<double> bid_dist(5.0, 50.0);

    // Add drivers
    auto start_drivers = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_DRIVERS; ++i) {
        int int_id = engine.add_driver(i + 1, lat_dist(rng), lon_dist(rng));
        driver_ids.push_back(int_id);
    }
    auto end_drivers = std::chrono::high_resolution_clock::now();
    std::cout << "Adding " << NUM_DRIVERS << " drivers took "
              << std::chrono::duration<double>(end_drivers - start_drivers).count() << " s\n";

    // Add riders
    auto start_riders = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_RIDERS; ++i) {
        int int_id = engine.add_rider(100000 + i, bid_dist(rng), lat_dist(rng), lon_dist(rng));
        rider_ids.push_back(int_id);
    }
    auto end_riders = std::chrono::high_resolution_clock::now();
    std::cout << "Adding " << NUM_RIDERS << " riders took "
              << std::chrono::duration<double>(end_riders - start_riders).count() << " s\n";

    double total_interest_time = 0.0;
    double total_match_time = 0.0;

    // Run simulation
    for (int iter = 0; iter < 10000; ++iter) {

        // Pick a random driver
        int d_idx = rng() % driver_ids.size();
        int driver_id = driver_ids[d_idx];

        // Find top K nearby riders for that driver
        auto nearby_riders = engine.get_top_k_riders_for_driver(driver_id); // you'll need a getter in engine

        if (nearby_riders.empty()) continue;

        // Pick a random rider from nearby
        int rider_id = nearby_riders[rng() % nearby_riders.size()];

        double ask = bid_dist(rng); // always acceptable for testing

        auto start_interest = std::chrono::high_resolution_clock::now();
        engine.driver_interest(driver_id, rider_id, ask);
        auto end_interest = std::chrono::high_resolution_clock::now();
        total_interest_time += std::chrono::duration<double>(end_interest - start_interest).count();

        // Every 500 iterations, make matches
        if (iter % 500 == 0) {
            auto start_match = std::chrono::high_resolution_clock::now();
            engine.make_matches();
            auto end_match = std::chrono::high_resolution_clock::now();
            total_match_time += std::chrono::duration<double>(end_match - start_match).count();
            std::cout << "[Iteration " << iter << "] make_matches() took "
                      << std::chrono::duration<double>(end_match - start_match).count() << " s\n";
        }
    }

    std::cout << "Total driver_interest() time: " << total_interest_time << " s\n";
    std::cout << "Total make_matches() time: " << total_match_time << " s\n";

    return 0;
}