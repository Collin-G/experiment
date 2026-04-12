#include "matching_engine.h"
#include <curl/curl.h>
#include <string>
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <nlohmann/json.hpp>
#include <iostream>



int MatchingEngine::add_rider(int ext_id, double bid, double lat, double lon) {
    Location loc(lat, lon);
    Rider rider_item(ext_id, bid, loc);

    std::vector<int> closest_drivers = find_top_k_from_cells(loc, drivers_by_cell_, drivers_, K, drivers_cursor_);
    // std::cout << closest_drivers.size();

    int int_rider_id = riders_.allocate(rider_item);
    Rider &rider = riders_[int_rider_id];
    int r_seq_no = riders_.seq_no(int_rider_id);
    for (auto int_driver_id : closest_drivers){
        Driver &driver = drivers_[int_driver_id];
        if (driver.state != OrderState::Active) continue; // <--- skip inactive
        int d_seq_no = drivers_.seq_no(int_driver_id);
        
      
        driver.inbox.allocate({int_rider_id, r_seq_no});
        rider.inbox.allocate({int_driver_id, d_seq_no});
        
        
    }

    H3Index cell = location_to_h3(loc, H3_RES);
    riders_by_cell_[cell].swap_list.push_back({int_rider_id, r_seq_no});

    return int_rider_id;
}


int MatchingEngine::add_driver(int ext_driver_id, double lat, double lon) {
    Location loc(lat, lon);
    Driver driver_item(ext_driver_id, loc);

    std::vector<int> closest_riders = find_top_k_from_cells(loc, riders_by_cell_, riders_, K, riders_cursor_);
    // std::cout<< closest_rider

    int int_driver_id = drivers_.allocate(driver_item);
    Driver &driver = drivers_[int_driver_id];
    int d_seq_no = drivers_.seq_no(int_driver_id);

    for (auto int_rider_id : closest_riders){
        Rider &rider = riders_[int_rider_id];
        if (rider.state != OrderState::Active) continue; // <--- skip inactive
        int r_seq_no = riders_.seq_no(int_rider_id);
      
        driver.inbox.allocate({int_rider_id, r_seq_no});
        rider.inbox.allocate({int_driver_id, d_seq_no});
        
    }

    H3Index cell = location_to_h3(loc, H3_RES);
    drivers_by_cell_[cell].swap_list.push_back({int_driver_id, d_seq_no});

    return int_driver_id;
}


void MatchingEngine::driver_interest(int int_driver_id, int int_rider_id, double ask){
    // auto r_iter = riders_.find(rider_id);
    // auto d_iter = drivers_.find(driver_id);
    Driver &driver = drivers_[int_driver_id];
    Rider &rider = riders_[int_rider_id];
    if (driver.state != OrderState::Active || rider.state != OrderState::Active) return;
    if (rider.bid < ask){
        return;
    }

    // interest_map_.push_back(rider);
    int d_seq_no = drivers_.seq_no(int_driver_id);
    int r_seq_no = riders_.seq_no(int_rider_id);
    
    rider.interested_drivers.push_back({int_driver_id, d_seq_no, ask});
    if (!rider.interesting){
        interest_map_.push_back({int_rider_id, r_seq_no});
        rider.interesting = true;
    }

}

void MatchingEngine::cancel_driver(int driver_id){
    drivers_[driver_id].state = OrderState::Canceled;
    clean_driver(driver_id);
}

void MatchingEngine::cancel_rider(int rider_id){
    riders_[rider_id].state = OrderState::Canceled;
    clean_rider(rider_id);
}

void MatchingEngine::clean_driver(int int_driver_id) {
    


    drivers_.free(int_driver_id);
}

void MatchingEngine::clean_rider(int int_rider_id) {

    riders_.free(int_rider_id);
}



void MatchingEngine::make_matches() {

    for (size_t i = 0; i < interest_map_.size(); ++i){
        auto& [id, seq_no] = interest_map_[i];
        Rider &rider = riders_[id];
        if (rider.state != OrderState::Active || seq_no != riders_.seq_no(id)){
            continue;
        }

        double bid = rider.bid;

        int best_driver = -1;
        int second_driver = -1;
        double best_ask = std::numeric_limits<double>::infinity();
        double second_ask = std::numeric_limits<double>::infinity();

        // std::cout << rider.inbox.size() << std::endl;
        for (size_t k = 0; k < rider.interested_drivers.size(); ++k){
            int driver_id = std::get<0>(rider.interested_drivers[k]);
            int d_seq_no = std::get<1>(rider.interested_drivers[k]);
            if (drivers_[driver_id].state != OrderState::Active || d_seq_no != drivers_.seq_no(driver_id)) {
                // if(d_seq_no != drivers_.seq_no(driver_id)){
                //     rider.inbox.free(k);
                // }
                continue;
            }
            auto ask = std::get<2>(rider.interested_drivers[k]);

            if (ask < best_ask) {
                second_driver = best_driver;
                second_ask = best_ask;

                best_ask = ask;
                best_driver = driver_id;
            }
            else if (ask < second_ask && ask >= best_ask) {
                second_driver = driver_id;
                second_ask = ask;
            }
        }

        if (best_driver != -1) {
            
            match_pair(best_driver, id, best_ask, second_ask, bid);

        }
        else{
            rider.interesting = false;
        }
        
    }
    interest_map_.clear();

}

void MatchingEngine::match_pair(int driver_id, int rider_id, double best_ask, double second_ask, double bid){
    if (riders_[rider_id].state != OrderState::Active || drivers_[driver_id].state != OrderState::Active){
        return;
    }
    // std::cout << "Matched driver " << driver_id 
    //           << " with rider " << rider_id << std::endl;

    

    
    riders_[rider_id].state = OrderState::Matched;
    drivers_[driver_id].state = OrderState::Matched;
    clean_driver(driver_id);
    clean_rider(rider_id);
}

double MatchingEngine::deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

double MatchingEngine::fast_approx_distance_km(const Location &a, const Location &b) {
    // Approximate meters per degree
    constexpr double KM_PER_DEG_LAT = 111.0; // 1 deg lat ~ 111 km
    double delta_lat = b.lat - a.lat;
    double delta_lon = b.lon - a.lon;
    
    // Scale longitude by cos of average latitude
    double avg_lat_rad = (a.lat + b.lat) * 0.5 * M_PI / 180.0;
    double delta_x = delta_lon * std::cos(avg_lat_rad) * KM_PER_DEG_LAT;
    double delta_y = delta_lat * KM_PER_DEG_LAT;
    
    return delta_x * delta_x + delta_y * delta_y; // return squared distance
}


template<typename FreeListType>
std::vector<int> MatchingEngine::find_top_k_from_cells(
    Location loc,
    ankerl::unordered_dense::map<H3Index, CellState>& cell_map,
    FreeListType& free_list,
    size_t k,
    size_t &cursor)
{
    std::vector<int> result;
    result.reserve(k);

    // auto& cell_state = cell_map.

    auto cells = get_neighboring_cells(location_to_h3(loc, H3_RES), SEARCH_RADIUS);

    for (auto cell : cells) {
        auto iter = cell_map.find(cell);
        if (iter == cell_map.end()) continue;

        auto& cell_state = iter->second;
        auto& person_ids = cell_state.swap_list;
        if (person_ids.empty()) continue;
        int start = cell_state.cursor;
        
        cell_state.cursor = (cell_state.cursor+1) % person_ids.size();
        

        for (size_t i = 0; i < person_ids.size();) {
            int idx = (start+i) % person_ids.size();
            int id = std::get<0>(person_ids[idx]);
            int seq_no = std::get<1>(person_ids[idx]);

            auto& person = free_list[id];

            if (person.state != OrderState::Active || seq_no != free_list.seq_no(id)) {
                if (seq_no != free_list.seq_no(id)) {
                    person_ids.free(i);
                    continue;
                }
                ++i;
                continue;
            }

            result.push_back(id);

            if (result.size() >= k) {
                return result;
            }

            ++i;
        }
    }

    if (result.empty() && free_list.size() > 0){
        size_t start = cursor;
        cursor = (cursor + 1) % free_list.size();
        for (size_t i = 0; i < free_list.size(); ++i){
            size_t idx = (start+i) % free_list.size();
            auto& person  = free_list[idx];
            if (person.state != OrderState::Active){
                continue;
            }
            result.push_back(idx);
            
            if (result.size() >= k) {
                return result;
            }
        }
    }

    return result;
}

H3Index MatchingEngine::location_to_h3(Location loc, int res) const {
    LatLng coord = {loc.lat, loc.lon};
    H3Index cell;
    latLngToCell(&coord, res, &cell);
    return cell;

}

std::vector<H3Index> MatchingEngine::get_neighboring_cells(H3Index center, int radius) const {
    int64_t max_neighbors;
    maxGridDiskSize(radius, &max_neighbors);
    std::vector<H3Index> neighbors(max_neighbors);
    gridDisk(center, radius, neighbors.data());
    return neighbors;
}



std::vector<int> MatchingEngine::get_top_k_riders_for_driver(int int_driver_id) {
    const Driver& driver = drivers_[int_driver_id];
    std::vector<int> result;

    auto driver_h3 = location_to_h3(driver.loc, H3_RES);
    auto neighbors = get_neighboring_cells(driver_h3, SEARCH_RADIUS);

    for (auto h3 : neighbors) {
        auto it = riders_by_cell_.find(h3);
        if (it != riders_by_cell_.end()) {
            const SwapList<std::tuple<int, int>>& flist = it->second.swap_list;
            for (size_t i = 0; i < flist.size(); ++i) {
                int idx = std::get<0>(flist[i]);
                int seq_no = std::get<1>(flist[i]);
                if (riders_[idx].state == OrderState::Active && riders_.seq_no(idx) == seq_no) {
                    result.push_back(idx);
                }
            }
        }
    }

    return result;
}