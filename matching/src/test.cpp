#include "matching_engine.h"
#include <iostream>
#include <chrono>
#include <random>
#include <iomanip>
#include <thread>
#include <atomic>

using namespace std::chrono;

static constexpr double BASE_LAT = 40.7128;
static constexpr double BASE_LON = -74.0060;
static constexpr double LAT_SPREAD = 0.1;  // ~11km spread
static constexpr double LON_SPREAD = 0.1;

class StressTest {
public:  // Make Metrics public
    struct Metrics {
        size_t total_riders = 0;
        size_t total_drivers = 0;
        size_t total_interests = 0;
        size_t total_matches = 0;
        double add_rider_time_ms = 0;
        double add_driver_time_ms = 0;
        double interest_time_ms = 0;
        double match_time_ms = 0;
        double total_time_ms = 0;
    };
    
private:
    MatchingEngine engine;
    std::mt19937 rng;
    std::vector<int> rider_ids;
    std::vector<int> driver_ids;
    std::vector<int> rider_ext_ids;
    std::vector<int> driver_ext_ids;
    
    double random_double(double min, double max) {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(rng);
    }
    
    int random_int(int min, int max) {
        std::uniform_int_distribution<int> dist(min, max);
        return dist(rng);
    }
    
    Location random_location() {
        return Location(
            BASE_LAT + random_double(-LAT_SPREAD, LAT_SPREAD),
            BASE_LON + random_double(-LON_SPREAD, LON_SPREAD)
        );
    }
    
    void print_progress_bar(int current, int total, const std::string& label) {
        const int bar_width = 50;
        float progress = (float)current / total;
        int pos = bar_width * progress;
        
        std::cout << "\r" << label << " [";
        for (int i = 0; i < bar_width; ++i) {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << "% " << std::flush;
    }
    
public:
    StressTest() : rng(std::random_device{}()) {}
    
    Metrics run_scenario(const std::string& name, 
                        size_t num_riders, 
                        size_t num_drivers,
                        float interest_probability = 0.3f,
                        size_t batch_size = 100) {
        
        std::cout << "\n\n" << std::string(70, '=') << "\n";
        std::cout << "SCENARIO: " << name << "\n";
        std::cout << std::string(70, '=') << "\n";
        std::cout << "Riders: " << num_riders << " | Drivers: " << num_drivers;
        std::cout << " | Interest probability: " << (interest_probability * 100) << "%\n";
        std::cout << std::string(70, '-') << "\n";
        
        Metrics metrics;
        metrics.total_riders = num_riders;
        metrics.total_drivers = num_drivers;
        
        auto total_start = high_resolution_clock::now();
        
        // Phase 1: Add all drivers
        std::cout << "\nPhase 1: Adding drivers...\n";
        auto phase_start = high_resolution_clock::now();
        
        driver_ext_ids.clear();
        driver_ids.clear();
        driver_ext_ids.reserve(num_drivers);
        driver_ids.reserve(num_drivers);
        
        for (size_t i = 0; i < num_drivers; ++i) {
            if (i % batch_size == 0) {
                print_progress_bar(i, num_drivers, "Adding drivers");
            }
            
            int ext_id = 10000 + i;
            Location loc = random_location();
            driver_ext_ids.push_back(ext_id);
            driver_ids.push_back(engine.add_driver(ext_id, loc.lat, loc.lon, {}));
        }
        print_progress_bar(num_drivers, num_drivers, "Adding drivers");
        
        auto phase_end = high_resolution_clock::now();
        metrics.add_driver_time_ms = duration<double, std::milli>(phase_end - phase_start).count();
        std::cout << "\n✓ Added " << num_drivers << " drivers in " 
                  << std::fixed << std::setprecision(2) << metrics.add_driver_time_ms << "ms";
        std::cout << " (" << (num_drivers * 1000 / std::max(1.0, metrics.add_driver_time_ms)) << " drivers/sec)\n";
        
        // Phase 2: Add all riders
        std::cout << "\nPhase 2: Adding riders...\n";
        phase_start = high_resolution_clock::now();
        
        rider_ext_ids.clear();
        rider_ids.clear();
        rider_ext_ids.reserve(num_riders);
        rider_ids.reserve(num_riders);
        
        for (size_t i = 0; i < num_riders; ++i) {
            if (i % batch_size == 0) {
                print_progress_bar(i, num_riders, "Adding riders");
            }
            
            int ext_id = 20000 + i;
            Location loc = random_location();
            double bid = random_double(5.0, 25.0);
            
            rider_ext_ids.push_back(ext_id);
            rider_ids.push_back(engine.add_rider(ext_id, bid, loc.lat, loc.lon, {}));
        }
        print_progress_bar(num_riders, num_riders, "Adding riders");
        
        phase_end = high_resolution_clock::now();
        metrics.add_rider_time_ms = duration<double, std::milli>(phase_end - phase_start).count();
        std::cout << "\n✓ Added " << num_riders << " riders in " 
                  << std::fixed << std::setprecision(2) << metrics.add_rider_time_ms << "ms";
        std::cout << " (" << (num_riders * 1000 / std::max(1.0, metrics.add_rider_time_ms)) << " riders/sec)\n";
        
        // Phase 3: Generate interests
        std::cout << "\nPhase 3: Generating driver interests...\n";
        phase_start = high_resolution_clock::now();
        
        std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);
        size_t interest_count = 0;
        
        for (size_t i = 0; i < num_drivers; ++i) {
            if (i % batch_size == 0) {
                print_progress_bar(i, num_drivers, "Generating interests");
            }
            
            // Each driver expresses interest in some nearby riders
            for (size_t j = 0; j < num_riders; ++j) {
                if (prob_dist(rng) < interest_probability) {
                    double ask = random_double(3.0, 30.0);
                    engine.driver_interest(driver_ext_ids[i], rider_ext_ids[j], ask);
                    interest_count++;
                }
            }
        }
        print_progress_bar(num_drivers, num_drivers, "Generating interests");
        
        phase_end = high_resolution_clock::now();
        metrics.interest_time_ms = duration<double, std::milli>(phase_end - phase_start).count();
        metrics.total_interests = interest_count;
        std::cout << "\n✓ Generated " << interest_count << " interests in " 
                  << std::fixed << std::setprecision(2) << metrics.interest_time_ms << "ms";
        std::cout << " (" << (interest_count * 1000 / std::max(1.0, metrics.interest_time_ms)) << " interests/sec)\n";
        
        // Phase 4: Run matching
        std::cout << "\nPhase 4: Running matching engine...\n";
        phase_start = high_resolution_clock::now();
        
        engine.make_matches();
        
        phase_end = high_resolution_clock::now();
        metrics.match_time_ms = duration<double, std::milli>(phase_end - phase_start).count();
        
        // Count matches
        metrics.total_matches = 0;
        for (int id : rider_ids) {
            if (engine.get_riders()[id].state == OrderState::Matched) {
                metrics.total_matches++;
            }
        }
        
        std::cout << "✓ Matching completed in " 
                  << std::fixed << std::setprecision(2) << metrics.match_time_ms << "ms\n";
        
        auto total_end = high_resolution_clock::now();
        metrics.total_time_ms = duration<double, std::milli>(total_end - total_start).count();
        
        // Print summary
        std::cout << "\n" << std::string(70, '-') << "\n";
        std::cout << "SUMMARY:\n";
        std::cout << "  Total time:          " << std::setw(10) << std::fixed << std::setprecision(2) 
                  << metrics.total_time_ms << " ms\n";
        std::cout << "  Matches created:     " << std::setw(10) << metrics.total_matches;
        std::cout << " (" << (metrics.total_matches * 100.0 / std::max(1UL, std::min(num_riders, num_drivers))) << "% of capacity)\n";
        std::cout << "  Avg match rate:      " << std::setw(10) << std::fixed << std::setprecision(0) 
                  << (metrics.total_matches * 1000 / std::max(1.0, metrics.total_time_ms)) << " matches/sec\n";
        
        std::cout << "\n  Breakdown:\n";
        std::cout << "    Add drivers:       " << std::setw(10) << metrics.add_driver_time_ms << " ms ("
                  << std::setw(5) << (metrics.add_driver_time_ms * 100 / std::max(1.0, metrics.total_time_ms)) << "%)\n";
        std::cout << "    Add riders:        " << std::setw(10) << metrics.add_rider_time_ms << " ms ("
                  << std::setw(5) << (metrics.add_rider_time_ms * 100 / std::max(1.0, metrics.total_time_ms)) << "%)\n";
        std::cout << "    Generate interests:" << std::setw(10) << metrics.interest_time_ms << " ms ("
                  << std::setw(5) << (metrics.interest_time_ms * 100 / std::max(1.0, metrics.total_time_ms)) << "%)\n";
        std::cout << "    Match execution:   " << std::setw(10) << metrics.match_time_ms << " ms ("
                  << std::setw(5) << (metrics.match_time_ms * 100 / std::max(1.0, metrics.total_time_ms)) << "%)\n";
        
        return metrics;
    }
    
    void run_concurrent_test(size_t num_threads, size_t ops_per_thread) {
        std::cout << "\n\n" << std::string(70, '=') << "\n";
        std::cout << "CONCURRENT STRESS TEST\n";
        std::cout << std::string(70, '=') << "\n";
        std::cout << "Threads: " << num_threads << " | Ops per thread: " << ops_per_thread << "\n";
        std::cout << std::string(70, '-') << "\n";
        
        std::vector<std::thread> threads;
        std::atomic<size_t> total_ops{0};
        std::atomic<size_t> successful_matches{0};
        
        auto start = high_resolution_clock::now();
        
        for (size_t t = 0; t < num_threads; ++t) {
            threads.emplace_back([&, t]() {
                MatchingEngine local_engine;
                std::mt19937 local_rng(std::random_device{}() + t);
                
                for (size_t i = 0; i < ops_per_thread; ++i) {
                    // Create a rider-driver pair
                    int rider_ext = t * ops_per_thread + i + 30000;
                    int driver_ext = t * ops_per_thread + i + 40000;
                    
                    Location loc = Location(
                        BASE_LAT + std::uniform_real_distribution<double>(-LAT_SPREAD, LAT_SPREAD)(local_rng),
                        BASE_LON + std::uniform_real_distribution<double>(-LON_SPREAD, LON_SPREAD)(local_rng)
                    );
                    
                    int rider = local_engine.add_rider(rider_ext, 
                        std::uniform_real_distribution<double>(5.0, 25.0)(local_rng), 
                        loc.lat, loc.lon, {});
                    
                    int driver = local_engine.add_driver(driver_ext, loc.lat, loc.lon, {});
                    
                    local_engine.driver_interest(driver_ext, rider_ext, 
                        std::uniform_real_distribution<double>(3.0, 30.0)(local_rng));
                    
                    local_engine.make_matches();
                    
                    if (local_engine.get_riders()[rider].state == OrderState::Matched) {
                        successful_matches++;
                    }
                    
                    total_ops++;
                    
                    if (total_ops % 1000 == 0) {
                        std::cout << "\rProgress: " << total_ops << " operations completed" << std::flush;
                    }
                }
            });
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
        
        auto end = high_resolution_clock::now();
        double total_time_ms = duration<double, std::milli>(end - start).count();
        
        std::cout << "\n\n" << std::string(70, '-') << "\n";
        std::cout << "CONCURRENT RESULTS:\n";
        std::cout << "  Total operations:    " << total_ops << "\n";
        std::cout << "  Total time:          " << std::fixed << std::setprecision(2) << total_time_ms << " ms\n";
        std::cout << "  Throughput:          " << std::fixed << std::setprecision(0) 
                  << (total_ops * 1000 / std::max(1.0, total_time_ms)) << " ops/sec\n";
        std::cout << "  Successful matches:  " << successful_matches;
        std::cout << " (" << (successful_matches * 100.0 / std::max(1UL, total_ops.load())) << "%)\n";
        std::cout << "  Avg latency:         " << std::fixed << std::setprecision(3) 
                  << (total_time_ms / std::max(1.0, (double)total_ops)) << " ms/op\n";
    }
};

int main() {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "+--------------------------------------------------------------------------+\n";
    std::cout << "|              MATCHING ENGINE STRESS TEST SUITE                            |\n";
    std::cout << "+--------------------------------------------------------------------------+\n";
    
    StressTest tester;
    std::vector<StressTest::Metrics> all_metrics;
    
    // Test 1: Small scale - 1K riders, 500 drivers
    all_metrics.push_back(tester.run_scenario(
        "Small Scale (1K riders, 500 drivers, 30% interest)",
        1000, 500, 0.3f
    ));
    
    // Test 2: Medium scale - 10K riders, 5K drivers
    all_metrics.push_back(tester.run_scenario(
        "Medium Scale (10K riders, 5K drivers, 10% interest)",
        10000, 5000, 0.1f
    ));
    
    // Test 3: Large scale - 50K riders, 25K drivers (sparse interest)
    // all_metrics.push_back(tester.run_scenario(
    //     "Large Scale (50K riders, 25K drivers, 5% interest)",
    //     50000, 25000, 0.05f, 500
    // ));
    
    // // Test 4: Dense interest - 5K riders, 5K drivers, high interest
    // all_metrics.push_back(tester.run_scenario(
    //     "Dense Interest (5K riders, 5K drivers, 80% interest)",
    //     5000, 5000, 0.8f
    // ));
    
    // // Test 5: Concurrent operations
    // tester.run_concurrent_test(4, 2500);  // 4 threads, 2500 ops each = 10K total
    
    // Final summary table
    std::cout << "\n\n" << std::string(70, '=') << "\n";
    std::cout << "                         FINAL SUMMARY TABLE\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << std::left << std::setw(35) << "Scenario" 
              << std::right << std::setw(10) << "Time(ms)" 
              << std::setw(12) << "Matches" 
              << std::setw(15) << "Match Rate/s\n";
    std::cout << std::string(70, '-') << "\n";
    
    for (size_t i = 0; i < all_metrics.size(); ++i) {
        auto& m = all_metrics[i];
        std::string scenario = "Test " + std::to_string(i+1) + ": " + 
                              std::to_string(m.total_riders) + "R/" + 
                              std::to_string(m.total_drivers) + "D";
        
        std::cout << std::left << std::setw(35) << scenario
                  << std::right << std::setw(10) << std::fixed << std::setprecision(0) << m.total_time_ms
                  << std::setw(12) << m.total_matches
                  << std::setw(15) << std::setprecision(0) << (m.total_matches * 1000 / std::max(1.0, m.total_time_ms)) << "\n";
    }
    
    std::cout << std::string(70, '=') << "\n";
    
    return 0;
}