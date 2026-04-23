#include "matching_engine.h"
#include <curl/curl.h>
#include <string>
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <nlohmann/json.hpp>
#include <iostream>



AddResult MatchingEngine::add_rider(int ext_rider_id, double bid, double lat, double lon, RiderMode mode) {
    Location loc(lat, lon);
    Rider rider_item(ext_rider_id, bid, loc, mode);
    int int_rider_id = riders_.allocate(rider_item);
    rider_ext_to_int[ext_rider_id] = int_rider_id;
    return {int_rider_id, riders_.seq_no(int_rider_id)};
}

AddResult MatchingEngine::add_driver(int ext_driver_id, double lat, double lon) {
    Location loc(lat, lon);
    Driver driver_item(ext_driver_id, loc);
    int int_driver_id = drivers_.allocate(driver_item);
    driver_ext_to_int[ext_driver_id] = int_driver_id;
    return {int_driver_id, drivers_.seq_no(int_driver_id)};
}


void MatchingEngine::driver_interest(int ext_driver_id, int dsn, int ext_rider_id, int rsn, double ask){
    // auto r_iter = riders_.find(rider_id);
    // auto d_iter = drivers_.find(driver_id);

    auto r_it = rider_ext_to_int.find(ext_rider_id);
    if (r_it == rider_ext_to_int.end()) {
        return;
    }
    int int_rider_id = r_it->second;


    auto d_it = driver_ext_to_int.find(ext_driver_id);
    if (d_it == driver_ext_to_int.end()) {
        return;
    }
    int int_driver_id = d_it->second;

    // interest_map_.push_back(rider);
    int d_seq_no = drivers_.seq_no(int_driver_id);
    int r_seq_no = riders_.seq_no(int_rider_id);

    Driver &driver = drivers_[int_driver_id];
    Rider &rider = riders_[int_rider_id];

    if (rider.mode == RiderMode::Instant) return;
    if (driver.state != OrderState::Active || rider.state != OrderState::Active) return;
    if (d_seq_no != dsn || r_seq_no != rsn) return;

    if (rider.bid < ask){
        return;
    }

    
    
    rider.interested_drivers.push_back({int_driver_id, d_seq_no, ask});
    if (!rider.interesting){
        interest_map_.push_back({int_rider_id, r_seq_no});
        rider.interesting = true;
    }

}

// matching_engine.cpp
bool MatchingEngine::cancel_rider(int ext_rider_id, int sn) {
    auto it = rider_ext_to_int.find(ext_rider_id);
    if (it == rider_ext_to_int.end()) return false;
    int rider_id = it->second;
    if (sn != riders_.seq_no(rider_id)) return false;
    riders_[rider_id].state = OrderState::Canceled;
    clean_rider(rider_id);
    return true;
}

bool MatchingEngine::cancel_driver(int ext_driver_id, int sn) {
    auto it = driver_ext_to_int.find(ext_driver_id);
    if (it == driver_ext_to_int.end()) return false;
    int driver_id = it->second;
    if (sn != drivers_.seq_no(driver_id)) return false;
    drivers_[driver_id].state = OrderState::Canceled;
    clean_driver(driver_id);
    return true;
}

void MatchingEngine::clean_driver(int int_driver_id) {
    driver_ext_to_int.erase(drivers_[int_driver_id].ext_id);
    drivers_.free(int_driver_id);
}

void MatchingEngine::clean_rider(int int_rider_id) {
    rider_ext_to_int.erase(riders_[int_rider_id].ext_id);
    riders_.free(int_rider_id);
}


// matching_engine.cpp
std::vector<MatchResult> MatchingEngine::make_matches() {
    std::vector<MatchResult> results;

    for (size_t i = 0; i < interest_map_.size();) {
        auto& [id, seq_no] = interest_map_[i];
        Rider& rider = riders_[id];
        if (rider.state != OrderState::Active || seq_no != riders_.seq_no(id)) {
            interest_map_.free(i);
            continue;
        }

        double bid = rider.bid;
        int best_driver = -1;
        int second_driver = -1;
        double best_ask = std::numeric_limits<double>::infinity();
        double second_ask = std::numeric_limits<double>::infinity();

        for (size_t k = 0; k < rider.interested_drivers.size(); ++k) {
            int driver_id = std::get<0>(rider.interested_drivers[k]);
            int d_seq_no  = std::get<1>(rider.interested_drivers[k]);
            if (drivers_[driver_id].state != OrderState::Active ||
                d_seq_no != drivers_.seq_no(driver_id)) continue;

            auto ask = std::get<2>(rider.interested_drivers[k]);
            if (ask < best_ask) {
                second_driver = best_driver;
                second_ask    = best_ask;
                best_ask      = ask;
                best_driver   = driver_id;
            } else if (ask < second_ask) {
                second_driver = driver_id;
                second_ask    = ask;
            }
        }

        if (best_driver != -1) {
            // second price auction clearing price
            // if only one driver, they pay their own ask
            // if multiple drivers, winner pays second lowest ask
            double clearing = (second_ask == std::numeric_limits<double>::infinity())
                ? best_ask
                : second_ask;

            results.push_back({
                riders_[id].ext_id,
                drivers_[best_driver].ext_id,
                clearing
            });

            match_pair(best_driver, id, best_ask, second_ask, bid);
            interest_map_.free(i);
        } else {
            ++i;
        }
    }

    return results;
}

std::optional<MatchResult> MatchingEngine::instant_match(
    int ext_rider_id, int rsn, int ext_driver_id, int dsn) {

    auto r_it = rider_ext_to_int.find(ext_rider_id);
    if (r_it == rider_ext_to_int.end()) {
        fprintf(stderr, "instant_match: rider %d not found\n", ext_rider_id);
        return std::nullopt;
    }
    int int_rider_id = r_it->second;

    auto d_it = driver_ext_to_int.find(ext_driver_id);
    if (d_it == driver_ext_to_int.end()) {
        fprintf(stderr, "instant_match: driver %d not found\n", ext_driver_id);
        return std::nullopt;
    }
    int int_driver_id = d_it->second;

    int d_seq_no = drivers_.seq_no(int_driver_id);
    int r_seq_no = riders_.seq_no(int_rider_id);

    Driver& driver = drivers_[int_driver_id];
    Rider&  rider  = riders_[int_rider_id];

    if (rider.mode != RiderMode::Instant) {
        fprintf(stderr, "instant_match: rider %d is not in instant mode\n", ext_rider_id);
        return std::nullopt;
    }
    if (driver.state != OrderState::Active || rider.state != OrderState::Active) {
        fprintf(stderr, "instant_match: driver or rider not active\n");
        return std::nullopt;
    }
    if (d_seq_no != dsn || r_seq_no != rsn) {
        fprintf(stderr, "instant_match: seq_no mismatch. driver expected %d got %d, rider expected %d got %d\n",
            dsn, d_seq_no, rsn, r_seq_no);
        return std::nullopt;
    }

    double price = rider.bid;
    match_pair(int_driver_id, int_rider_id, price, price, price);
    return MatchResult{ext_rider_id, ext_driver_id, price};
}

void MatchingEngine::match_pair(int driver_id, int rider_id, double best_ask, double second_ask, double bid){
    if (riders_[rider_id].state != OrderState::Active || drivers_[driver_id].state != OrderState::Active){
        return;
    }
    
    riders_[rider_id].state = OrderState::Matched;
    drivers_[driver_id].state = OrderState::Matched;
    clean_driver(driver_id);
    clean_rider(rider_id);
}




