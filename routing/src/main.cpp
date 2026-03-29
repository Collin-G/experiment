#include "matching_engine.h"
#include <iostream>
#include <thread>
#include <chrono>




void test_matching_engine(){
    MatchingEngine engine = MatchingEngine();



    // Riders and drivers added already
engine.add_driver(1, 43.6930, -79.3230);
engine.add_driver(2, 43.6940, -79.3240);
engine.add_driver(3, 43.6950, -79.3250);
engine.add_driver(4, 43.6960, -79.3260);
engine.add_driver(5, 43.6970, -79.3270);


engine.add_rider(101, 30.0, 43.6900, -79.3200);
engine.add_rider(102, 12.0, 43.6910, -79.3210);
engine.add_rider(103, 5.0, 43.6920, -79.3220);



// --- Make some drivers show interest ---
engine.driver_interest(1, 101, 1.0);
engine.driver_interest(1, 102, 2.0);
engine.driver_interest(1, 103, 3.0);

engine.driver_interest(2, 101, 0.8);
engine.driver_interest(2, 102, 1.5);
engine.driver_interest(2, 103, 2.5);

engine.driver_interest(3, 101, 1.2);
engine.driver_interest(3, 102, 0.9);
engine.driver_interest(3, 103, 1.8);

engine.driver_interest(4, 101, 2.0);
engine.driver_interest(4, 102, 1.0);
engine.driver_interest(4, 103, 0.5);

engine.driver_interest(5, 101, 1.5);
engine.driver_interest(5, 102, 1.2);
engine.driver_interest(5, 103, 0.8);

// Now process matches




engine.make_matches();




}

// -------------------- Main --------------------
int main() {
    test_matching_engine();
    return 0;
}
