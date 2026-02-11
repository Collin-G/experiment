#include "osm_parser.h"
#include "graphbuilder.h"
#include "astar.h"
#include "router.h"
#include "matching.h"
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <random>

void basic_routing_test(RoutingEngine& routing_engine) {
    std::cout << "\n=== Basic Routing Test ===\n";
    
    // Test route calculation
    double lat1 = 43.69;
    double lon1 = -79.32;
    double lat2 = 43.6845;
    double lon2 = -79.339;
    
    double distance = routing_engine.route(lat1, lon1, lat2, lon2);
    std::cout << "Route from (" << lat1 << ", " << lon1 << ") to ("
              << lat2 << ", " << lon2 << "): " << distance << " meters\n";
    
    // Test edge update
    double middle_lat = (43.692 + 43.6896) / 2;
    double middle_lon = (-79.322 - 79.3221) / 2;
    
    double original_distance = distance;
    routing_engine.update_edge(middle_lat, middle_lon, 999, Direction::BOTH);
    
    distance = routing_engine.route(lat1, lon1, lat2, lon2);
    std::cout << "After blocking middle edge: " << distance << " meters\n";
    
    if (distance > original_distance) {
        std::cout << "✓ Routing engine successfully reroutes around blocked edge\n";
    }
}

void test_matching_engine(RoutingEngine& routing_engine) {
    std::cout << "\n=== Testing Matching Engine ===\n";
    
    // Create matching engine (constructor only takes router now)
    MatchingEngine engine(&routing_engine);
    
    // Start the engine with 4 threads
    engine.start(4);
    
    // Give it a moment to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "\n--- Adding Drivers ---\n";
    engine.add_driver(1, 10.0, 43.69, -79.32);    // $10 ask
    engine.add_driver(2, 15.0, 43.685, -79.33);   // $15 ask  
    engine.add_driver(3, 8.0, 43.688, -79.325);   // $8 ask (cheapest)
    engine.add_driver(4, 12.0, 43.683, -79.335);  // $12 ask
    engine.add_driver(5, 25.0, 43.695, -79.31);   // $25 ask (expensive)
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "\n--- Adding Riders ---\n";
    engine.add_rider(101, 30.0, 43.69, -79.32);   // $30 bid (can afford all)
    engine.add_rider(102, 12.0, 43.684, -79.33);  // $12 bid (can afford drivers 1,3)
    engine.add_rider(103, 5.0, 43.686, -79.327);  // $5 bid (too low for anyone)
    
    std::cout << "\n--- Checking State After 1 Second ---\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    engine.print_state();
    
    std::cout << "\n--- Testing Acceptances ---\n";
    
    // Test 1: Driver 3 accepts rider 101 (should match: $8 <= $30)
    std::cout << "1. Driver 3 accepting Rider 101...\n";
    engine.driver_accept(3, 101);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    engine.print_state();
    
    // Test 2: Driver 1 accepts rider 102 (should match: $10 <= $12)
    std::cout << "\n2. Driver 1 accepting Rider 102...\n";
    engine.driver_accept(1, 102);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    engine.print_state();
    
    // Test 3: Driver 2 tries rider 103 (should fail: $15 > $5)
    std::cout << "\n3. Driver 2 attempting Rider 103...\n";
    engine.driver_accept(2, 103);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    engine.print_state();
    
    std::cout << "\n--- Testing Cancellations ---\n";
    std::cout << "Cancelling Driver 4...\n";
    engine.driver_cancel(4);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    engine.print_state();
    
    std::cout << "\n--- Testing Timeout (simulated) ---\n";
    std::cout << "Adding rider that should timeout...\n";
    engine.add_rider(104, 20.0, 43.692, -79.322);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    engine.print_state();
    
    // In real system, timeout is 5 minutes (300 seconds)
    // For testing, we can simulate shorter timeout by modifying TIMEOUT_SEC
    // or just show the concept
    
    std::cout << "\n--- Dynamic Test: Price Matching ---\n";
    
    engine.add_driver(6, 7.0, 43.691, -79.321);   // $7 ask
    engine.add_driver(7, 40.0, 43.689, -79.323);  // $40 ask (very expensive)
    
    engine.add_rider(105, 10.0, 43.690, -79.322);  // $10 bid
    engine.add_rider(106, 50.0, 43.688, -79.324);  // $50 bid
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    engine.print_state();
    
    std::cout << "\nDriver 6 accepting Rider 105...\n";
    engine.driver_accept(6, 105);  // Should match ($7 <= $10)
    
    std::cout << "\nDriver 7 accepting Rider 105...\n";
    engine.driver_accept(7, 105);  // Should fail ($40 > $10) - rider already matched anyway
    
    std::cout << "\nDriver 7 accepting Rider 106...\n";
    engine.driver_accept(7, 106);  // Should match ($40 <= $50)
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    engine.print_state();
    
    std::cout << "\n--- Final Cleanup ---\n";
    engine.stop();
    
    std::cout << "\n=== Matching Engine Test Complete ===\n";
}

void interactive_test(RoutingEngine& routing_engine) {
    std::cout << "\n=== Interactive Matching Engine Test ===\n";
    std::cout << "Commands:\n";
    std::cout << "  driver ID ASK LAT LON  - Add a driver\n";
    std::cout << "  rider ID BID LAT LON   - Add a rider\n";
    std::cout << "  accept DRIVER RIDER    - Driver accepts rider\n";
    std::cout << "  cancel-driver ID       - Cancel driver\n";
    std::cout << "  cancel-rider ID        - Cancel rider\n";
    std::cout << "  state                  - Show current state\n";
    std::cout << "  quit                   - Exit\n\n";
    
    MatchingEngine engine(&routing_engine);
    engine.start(2);
    
    std::string command;
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, command);
        
        if (command.empty()) continue;
        
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "quit" || cmd == "exit") {
            break;
        }
        else if (cmd == "driver") {
            int id;
            double ask, lat, lon;
            if (iss >> id >> ask >> lat >> lon) {
                engine.add_driver(id, ask, lat, lon);
            } else {
                std::cout << "Usage: driver ID ASK LAT LON\n";
            }
        }
        else if (cmd == "rider") {
            int id;
            double bid, lat, lon;
            if (iss >> id >> bid >> lat >> lon) {
                engine.add_rider(id, bid, lat, lon);
            } else {
                std::cout << "Usage: rider ID BID LAT LON\n";
            }
        }
        else if (cmd == "accept") {
            int driver_id, rider_id;
            if (iss >> driver_id >> rider_id) {
                engine.driver_accept(driver_id, rider_id);
            } else {
                std::cout << "Usage: accept DRIVER_ID RIDER_ID\n";
            }
        }
        else if (cmd == "cancel-driver") {
            int id;
            if (iss >> id) {
                engine.driver_cancel(id);
            }
        }
        else if (cmd == "cancel-rider") {
            int id;
            if (iss >> id) {
                engine.rider_cancel(id);
            }
        }
        else if (cmd == "state") {
            engine.print_state();
        }
        else if (cmd == "help") {
            std::cout << "Commands: driver, rider, accept, cancel-driver, cancel-rider, state, quit\n";
        }
        else {
            std::cout << "Unknown command. Type 'help' for commands.\n";
        }
        
        // Small delay to let processing happen
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    engine.stop();
    std::cout << "Interactive test complete.\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <osm_file.osm.pbf> [interactive|performance]\n";
        std::cerr << "Examples:\n";
        std::cerr << "  " << argv[0] << " toronto.osm.pbf           # Basic test\n";
        std::cerr << "  " << argv[0] << " toronto.osm.pbf interactive # Interactive mode\n";
        std::cerr << "  " << argv[0] << " toronto.osm.pbf performance # Performance test\n";
        return 1;
    }
    
    std::string osm_file = argv[1];
    
    try {
        // 1. Load OSM data
        std::cout << "Loading OSM data from " << osm_file << "...\n";
        
        OSMHandler handler;
        osmium::io::Reader reader(osm_file);
        osmium::apply(reader, handler);
        reader.close();
        
        std::cout << "Loaded " << handler.nodes.size() << " nodes and " 
                  << handler.ways.size() << " ways.\n";
        
        // 2. Build graph
        GraphBuilder builder(std::move(handler.nodes), std::move(handler.ways));
        Graph graph = builder.build_graph();
        
        std::cout << "Built graph with " << graph.nodes().size() << " nodes.\n";
        
        // 3. Create routing engine
        RoutingEngine routing_engine(graph);
        std::cout << "Routing engine created.\n";
        
        // 4. Test basic routing
        basic_routing_test(routing_engine);
        
        // 5. Run tests based on command line argument
        std::string mode = (argc > 2) ? argv[2] : "basic";
        
        if (mode == "interactive") {
            interactive_test(routing_engine);
        }
        else if (mode == "performance") {
            // Simple performance test
            std::cout << "\n=== Performance Test ===\n";
            
            MatchingEngine engine(&routing_engine);
            engine.start(8);  // Use 8 threads for performance
            
            auto start_time = std::chrono::steady_clock::now();
            
            // Add 100 drivers
            for (int i = 1; i <= 100; i++) {
                engine.add_driver(i, 10.0 + (i % 20), 
                                43.68 + (i % 100) * 0.0002, 
                                -79.33 + (i % 100) * 0.0002);
            }
            
            // Add 50 riders
            for (int i = 101; i <= 150; i++) {
                engine.add_rider(i, 20.0 + (i % 15),
                               43.68 + (i % 100) * 0.0002,
                               -79.33 + (i % 100) * 0.0002);
            }
            
            // Simulate some matches
            for (int i = 1; i <= 20; i++) {
                if (i <= 100) {  // Ensure driver exists
                    engine.driver_accept(i, 100 + (i % 50 + 1));
                }
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
            engine.stop();
            
            std::cout << "Performance test completed in " << duration.count() << "ms\n";
        }
        else {  // basic mode (default)
            test_matching_engine(routing_engine);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}