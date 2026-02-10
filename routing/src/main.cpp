#include "osm_parser.h"
#include "graphbuilder.h"
#include "astar.h"
#include "router.h"
#include "matching.h"  // Include your matching engine
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <random>

void test_matching_engine(RoutingEngine& routing_engine) {
    std::cout << "\n=== Testing Matching Engine ===\n";
    
    // Create matching engine with routing engine
    MatchingEngine engine(&routing_engine, 4);  // 4 worker threads
    
    // Start the engine
    engine.start();
    
    // Add some test drivers
    std::cout << "\nAdding drivers...\n";
    engine.add_driver(1, 10.0, 43.69, -79.32);    // Driver 1: ask $10
    engine.add_driver(2, 15.0, 43.685, -79.33);   // Driver 2: ask $15
    engine.add_driver(3, 8.0, 43.688, -79.325);   // Driver 3: ask $8
    engine.add_driver(4, 12.0, 43.683, -79.335);  // Driver 4: ask $12
    engine.add_driver(5, 20.0, 43.695, -79.31);   // Driver 5: ask $20 (far away)
    
    // Wait a bit for drivers to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Add test riders
    std::cout << "\nAdding riders...\n";
    engine.add_rider(101, 25.0, 43.69, -79.32);   // Rider 1: bid $25
    engine.add_rider(102, 12.0, 43.684, -79.33);  // Rider 2: bid $12
    engine.add_rider(103, 18.0, 43.686, -79.327); // Rider 3: bid $18
    
    // Wait for matching to happen
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Test driver accepting offers
    std::cout << "\nTesting driver acceptances...\n";
    
    // Driver 1 accepts rider 101 (should match)
    std::cout << "Driver 1 accepting Rider 101...\n";
    engine.driver_accept(1, 101);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Driver 2 accepts rider 102 (should match)
    std::cout << "Driver 2 accepting Rider 102...\n";
    engine.driver_accept(2, 102);
    
    // Test cancellation
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "\nCancelling Driver 3...\n";
    engine.driver_cancel(3);
    
    // Add more riders to test timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "\nAdding rider that will timeout...\n";
    engine.add_rider(104, 30.0, 43.692, -79.322); // Rider 4: bid $30
    
    // Wait for timeout (should be 5 minutes = 300 seconds)
    std::cout << "Waiting 6 seconds (in real system would be 300s for timeout)...\n";
    std::this_thread::sleep_for(std::chrono::seconds(6));
    
    // Add more dynamic test
    std::cout << "\n=== Dynamic Test ===\n";
    
    // Add drivers in different price ranges
    engine.add_driver(6, 5.0, 43.69, -79.32);    // Cheap driver
    engine.add_driver(7, 30.0, 43.685, -79.33);  // Expensive driver
    
    // Add riders with different bids
    engine.add_rider(105, 8.0, 43.691, -79.321);   // Low bid
    engine.add_rider(106, 25.0, 43.689, -79.323);  // Medium bid
    engine.add_rider(107, 35.0, 43.687, -79.325);  // High bid
    
    // Wait and test acceptances
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout << "\nDriver 6 accepting Rider 105...\n";
    engine.driver_accept(6, 105);  // Should match (bid $8 > ask $5)
    
    std::cout << "Driver 7 accepting Rider 106...\n";
    engine.driver_accept(7, 106);  // Should NOT match (bid $25 < ask $30)
    
    // Wait a bit to see if timeout catches unmatched riders
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Stop the engine
    std::cout << "\nStopping matching engine...\n";
    engine.stop();
    
    std::cout << "\n=== Test Complete ===\n";
}

void run_performance_test(RoutingEngine& routing_engine) {
    std::cout << "\n=== Performance Test ===\n";
    
    MatchingEngine engine(&routing_engine, 8);  // 8 worker threads for stress test
    
    engine.start();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> lat_dist(43.68, 43.70);
    std::uniform_real_distribution<> lon_dist(-79.34, -79.32);
    std::uniform_real_distribution<> price_dist(5.0, 50.0);
    
    const int NUM_DRIVERS = 100;
    const int NUM_RIDERS = 50;
    
    std::cout << "Adding " << NUM_DRIVERS << " drivers...\n";
    for (int i = 1; i <= NUM_DRIVERS; i++) {
        engine.add_driver(i, price_dist(gen), lat_dist(gen), lon_dist(gen));
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Adding " << NUM_RIDERS << " riders...\n";
    for (int i = 101; i <= 100 + NUM_RIDERS; i++) {
        engine.add_rider(i, price_dist(gen), lat_dist(gen), lon_dist(gen));
    }
    
    std::cout << "Simulating matches...\n";
    
    // Simulate some driver acceptances
    for (int i = 1; i <= std::min(20, NUM_RIDERS); i++) {
        if (i <= NUM_DRIVERS) {
            engine.driver_accept(i, 100 + i);
        }
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Stopping performance test...\n";
    engine.stop();
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <osm_file.osm.pbf>\n";
        return 1;
    }
    
    std::string osm_file = argv[1];
    
    try {
        // 1. Read OSM file
        OSMHandler handler;
        osmium::io::Reader reader(osm_file);
        osmium::apply(reader, handler);
        reader.close();
        
        std::cout << "Loaded " << handler.nodes.size() << " nodes and " 
                  << handler.ways.size() << " ways.\n";
        
        // 2. Build the graph
        GraphBuilder builder(std::move(handler.nodes), std::move(handler.ways));
        Graph graph = builder.build_graph();
        
        std::cout << "Graph has " << graph.nodes().size() << " nodes.\n";
        
        // 3. Test basic A* routing
        std::cout << "\n=== Testing Basic Routing ===\n";
        if (graph.nodes().size() >= 2) {
            AStar astar;
            int start_idx = 0;
            int goal_idx = graph.nodes().size() - 1;
            
            auto result = astar.shortest_path(graph, start_idx, goal_idx);
            std::cout << "A* test path distance: " << result.total_cost << "\n";
        }
        
        // 4. Create routing engine
        RoutingEngine routing_engine(graph);
        
        // 5. Test routing engine
        std::cout << "\n=== Testing Routing Engine ===\n";
        double lat1 = 43.69;
        double lon1 = -79.32;
        double lat2 = 43.6845;
        double lon2 = -79.339;
        
        double distance = routing_engine.route(lat1, lon1, lat2, lon2);
        std::cout << "Route distance: " << distance << " meters\n";
        
        // 6. Test matching engine with routing
        test_matching_engine(routing_engine);
        
        // 7. Optional: Performance test
        if (argc > 2 && std::string(argv[2]) == "--performance") {
            run_performance_test(routing_engine);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}