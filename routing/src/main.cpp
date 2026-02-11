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
#include <iomanip>

// Debug helper to print current offers
void print_offers_debug(const MatchingEngine& engine) {
    engine.print_state();
    // Add a small delay to ensure all output is flushed
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

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
    
    // Create matching engine
    MatchingEngine engine(&routing_engine);
    
    // Start the engine with 4 threads
    engine.start(4);
    
    // Give it a moment to start
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "Engine started, waiting for initialization...\n";
    
    std::cout << "\n--- Adding Drivers ---\n";
    // Use very close coordinates (same H3 cell)
    engine.add_driver(1, 10.0, 43.690000, -79.320000);    // $10 ask
    engine.add_driver(2, 15.0, 43.690001, -79.320001);   // $15 ask  
    engine.add_driver(3, 8.0, 43.690002, -79.320002);    // $8 ask (cheapest)
    engine.add_driver(4, 12.0, 43.690003, -79.320003);   // $12 ask
    engine.add_driver(5, 25.0, 43.690004, -79.320004);   // $25 ask (expensive)
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "\n--- All drivers added ---\n";
    
    std::cout << "\n--- Adding Riders ---\n";
    engine.add_rider(101, 30.0, 43.690000, -79.320000);   // $30 bid (can afford all)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    engine.add_rider(102, 12.0, 43.690001, -79.320001);   // $12 bid (can afford drivers 1,3,4)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    engine.add_rider(103, 5.0, 43.690002, -79.320002);   // $5 bid (too low for anyone)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "\n--- Waiting for offers to be processed (2 seconds) ---\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "\n--- Current State After Processing ---\n";
    print_offers_debug(engine);
    
    std::cout << "\n--- Testing Acceptances ---\n";
    
    // Wait a bit more to ensure all workers have processed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Test 1: Try to match - we need to know which drivers got offers
    std::cout << "1. Trying to find a valid driver-rider pair...\n";
    
    // For debugging, let's try multiple combinations
    // First check what the current state looks like
    print_offers_debug(engine);
    
    // Try matching - we need to check which drivers have which riders in inbox
    std::cout << "\n2. Testing Driver 3 accepting Rider 101...\n";
    engine.driver_accept(3, 101);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_offers_debug(engine);
    
    std::cout << "\n3. Testing Driver 1 accepting Rider 102...\n";
    engine.driver_accept(1, 102);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_offers_debug(engine);
    
    std::cout << "\n--- Testing Cancellations ---\n";
    std::cout << "Cancelling Driver 4...\n";
    engine.driver_cancel(4);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_offers_debug(engine);
    
    std::cout << "\n--- Testing New Rider ---\n";
    engine.add_rider(104, 20.0, 43.690005, -79.320005);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    print_offers_debug(engine);
    
    // Try to accept the new rider
    std::cout << "\n4. Testing Driver 2 accepting Rider 104...\n";
    engine.driver_accept(2, 104);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_offers_debug(engine);
    
    std::cout << "\n--- Final Cleanup ---\n";
    engine.stop();
    
    std::cout << "\n=== Matching Engine Test Complete ===\n";
}

void simple_matching_test(RoutingEngine& routing_engine) {
    std::cout << "\n=== SIMPLE MATCHING TEST (One driver, one rider) ===\n";
    
    MatchingEngine engine(&routing_engine);
    engine.start(2);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Test 1: Simple exact location match
    std::cout << "\n--- Test 1: Exact location match ---\n";
    engine.add_driver(100, 10.0, 43.69, -79.32);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    engine.add_rider(200, 20.0, 43.69, -79.32);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "State before acceptance:\n";
    print_offers_debug(engine);
    
    std::cout << "Driver 100 accepting Rider 200...\n";
    engine.driver_accept(100, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "State after acceptance:\n";
    print_offers_debug(engine);
    
    // Test 2: Price check - driver too expensive
    std::cout << "\n--- Test 2: Driver too expensive ---\n";
    engine.add_driver(101, 50.0, 43.69, -79.32);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    engine.add_rider(201, 30.0, 43.69, -79.32);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "State (should see no match due to price):\n";
    print_offers_debug(engine);
    
    std::cout << "Driver 101 attempting Rider 201 (should fail due to price)...\n";
    engine.driver_accept(101, 201);
    
    // Test 3: Multiple drivers, one rider
    std::cout << "\n--- Test 3: Multiple drivers, one rider ---\n";
    engine.add_driver(102, 5.0, 43.69, -79.32);
    engine.add_driver(103, 8.0, 43.69, -79.32);
    engine.add_driver(104, 12.0, 43.69, -79.32);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    engine.add_rider(202, 10.0, 43.69, -79.32);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "State with multiple drivers:\n";
    print_offers_debug(engine);
    
    // Try to accept with driver 103 ($8 <= $10)
    std::cout << "Driver 103 accepting Rider 202...\n";
    engine.driver_accept(103, 202);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Final state:\n";
    print_offers_debug(engine);
    
    engine.stop();
}

void diagnostic_test(RoutingEngine& routing_engine) {
    std::cout << "\n=== DIAGNOSTIC TEST (Debug matching logic) ===\n";
    
    MatchingEngine engine(&routing_engine);
    
    // Temporarily adjust settings if possible
    // Note: In real implementation, you'd need to expose these settings
    
    engine.start(1); // Single thread for easier debugging
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    std::cout << "\n1. Adding 5 drivers at SAME location:\n";
    for (int i = 1; i <= 5; i++) {
        double ask = i * 5.0; // $5, $10, $15, $20, $25
        engine.add_driver(i, ask, 43.69, -79.32);
        std::cout << "  Driver " << i << ": ask=$" << ask << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "\n2. Adding rider with $30 bid at SAME location:\n";
    engine.add_rider(100, 30.0, 43.69, -79.32);
    std::cout << "  Rider 100: bid=$30\n";
    
    std::cout << "\n3. Waiting for matching worker to process...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    std::cout << "\n4. Current state:\n";
    print_offers_debug(engine);
    
    std::cout << "\n5. Testing acceptances:\n";
    // Try each driver in order
    for (int i = 1; i <= 5; i++) {
        std::cout << "  Driver " << i << " accepting Rider 100... ";
        engine.driver_accept(i, 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    engine.stop();
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
    std::cout << "  wait SECONDS           - Wait for N seconds\n";
    std::cout << "  quit                   - Exit\n\n";
    
    MatchingEngine engine(&routing_engine);
    engine.start(2);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
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
            print_offers_debug(engine);
        }
        else if (cmd == "wait") {
            int seconds;
            if (iss >> seconds) {
                std::cout << "Waiting " << seconds << " seconds...\n";
                std::this_thread::sleep_for(std::chrono::seconds(seconds));
                std::cout << "Done waiting.\n";
            }
        }
        else if (cmd == "help") {
            std::cout << "Commands: driver, rider, accept, cancel-driver, cancel-rider, state, wait, quit\n";
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
        std::cerr << "Usage: " << argv[0] << " <osm_file.osm.pbf> [test_mode]\n";
        std::cerr << "Test modes:\n";
        std::cerr << "  basic       - Basic routing test (default)\n";
        std::cerr << "  simple      - Simple matching test\n";
        std::cerr << "  diagnostic  - Diagnostic matching test\n";
        std::cerr << "  interactive - Interactive mode\n";
        std::cerr << "  performance - Performance test\n";
        std::cerr << "\nExamples:\n";
        std::cerr << "  " << argv[0] << " toronto.osm.pbf\n";
        std::cerr << "  " << argv[0] << " toronto.osm.pbf simple\n";
        std::cerr << "  " << argv[0] << " toronto.osm.pbf diagnostic\n";
        return 1;
    }
    
    std::string osm_file = argv[1];
    std::string mode = (argc > 2) ? argv[2] : "basic";
    
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
        
        // 4. Run tests based on command line argument
        if (mode == "simple") {
            simple_matching_test(routing_engine);
        }
        else if (mode == "diagnostic") {
            basic_routing_test(routing_engine);
            diagnostic_test(routing_engine);
        }
        else if (mode == "interactive") {
            basic_routing_test(routing_engine);
            interactive_test(routing_engine);
        }
        else if (mode == "performance") {
            std::cout << "\n=== Performance Test ===\n";
            
            MatchingEngine engine(&routing_engine);
            engine.start(8);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            auto start_time = std::chrono::steady_clock::now();
            
            // Add 100 drivers
            for (int i = 1; i <= 100; i++) {
                engine.add_driver(i, 10.0 + (i % 20), 
                                43.69 + (i % 100) * 0.0001, 
                                -79.32 + (i % 100) * 0.0001);
            }
            
            // Add 50 riders
            for (int i = 101; i <= 150; i++) {
                engine.add_rider(i, 20.0 + (i % 15),
                               43.69 + (i % 100) * 0.0001,
                               -79.32 + (i % 100) * 0.0001);
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            std::cout << "Added 100 drivers and 50 riders in " << duration.count() << "ms\n";
            
            std::this_thread::sleep_for(std::chrono::seconds(3));
            std::cout << "\nFinal state after processing:\n";
            print_offers_debug(engine);
            
            engine.stop();
        }
        else {  // basic mode (default)
            basic_routing_test(routing_engine);
            test_matching_engine(routing_engine);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}