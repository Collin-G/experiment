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

    std::vector<int> closest_drivers = find_top_k_from_cells(loc, drivers_by_cell_, drivers_, K);
    // std::cout << closest_drivers.size();

    int int_rider_id = riders_.allocate(rider_item);
    Rider &rider = riders_[int_rider_id];

    for (auto int_driver_id : closest_drivers){
        Driver &driver = drivers_[int_driver_id];
        if (driver.state != OrderState::Active) continue; // <--- skip inactive
        driver.inbox.allocate({int_rider_id, bid, std::nullopt});
        rider.inbox.allocate({int_driver_id, std::nullopt});
    }

    H3Index cell = location_to_h3(loc, H3_RES);
    riders_by_cell_[cell].allocate(int_rider_id);

    

    return int_rider_id;
}


int MatchingEngine::add_driver(int ext_driver_id, double lat, double lon) {
    Location loc(lat, lon);
    Driver driver_item(ext_driver_id, loc);

    std::vector<int> closest_riders = find_top_k_from_cells(loc, riders_by_cell_, riders_, K);
    // std::cout<< closest_rider

    int int_driver_id = drivers_.allocate(driver_item);
    Driver &driver = drivers_[int_driver_id];

    for (auto int_rider_id : closest_riders){
        Rider &rider = riders_[int_rider_id];
        if (rider.state != OrderState::Active) continue; // <--- skip inactive
        driver.inbox.allocate({int_driver_id, rider.bid, std::nullopt});
        rider.inbox.allocate({int_driver_id, std::nullopt});
    }

    H3Index cell = location_to_h3(loc, H3_RES);
    drivers_by_cell_[cell].allocate(int_driver_id);

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


    for (size_t i = 0; i < driver.inbox.size(); ++i){
        if (std::get<0>(driver.inbox[i]) == int_rider_id) {
            std::get<2>(driver.inbox[i]) = ask;
            break;
        }
    }

    for (size_t i = 0; i < rider.inbox.size(); ++i){
        if (std::get<0>(rider.inbox[i]) == int_driver_id) {
            std::get<1>(rider.inbox[i]) = ask;
            break;
        }
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
    

    Driver &driver = drivers_[int_driver_id];

    // 1. Remove driver from H3 map
    H3Index cell = location_to_h3(driver.loc, H3_RES);
    auto h3_iter = drivers_by_cell_.find(cell);
    if (h3_iter != drivers_by_cell_.end()) {
        auto &lst = h3_iter->second;
        for (size_t i = 0; i < lst.size(); ++i){
            if (lst[i] == int_driver_id){
                lst.free(i);
                break;
            }
        }
       
    }

    // 2. Remove driver from riders' driver lists
    for (size_t i = 0; i < driver.inbox.size(); ++i){
        int int_rider_id = std::get<0>(driver.inbox[i]);
        Rider &rider = riders_[int_rider_id];
        for (size_t k = 0; k < rider.inbox.size(); ++k){
            if (std::get<0>(rider.inbox[k]) == int_driver_id){
                rider.inbox.free(k);
                break;
            }
        }
   
    }
    
    // 3. Finally, remove driver from drivers_ map
    drivers_.free(int_driver_id);
}

void MatchingEngine::clean_rider(int int_rider_id) {


    Rider &rider = riders_[int_rider_id];





    for (size_t i = 0; i < rider.inbox.size(); ++i){
        int int_driver_id = std::get<0>(rider.inbox[i]);
        Driver &driver = drivers_[int_driver_id];
        for (size_t k = 0; k < driver.inbox.size(); ++k){
            if (std::get<0>(driver.inbox[k]) == int_rider_id){
                driver.inbox.free(k);
                break;
            }
        }

    }

    


    H3Index cell = location_to_h3(rider.loc, H3_RES);
    auto h3_iter = riders_by_cell_.find(cell);
    if (h3_iter != riders_by_cell_.end()) {
        auto &lst = h3_iter->second;
        for (size_t i = 0; i < lst.size(); ++i){
            if (lst[i] == int_rider_id){
                lst.free(i);
                break;
            }
        }
    }

    // Finally, remove from main riders map
    riders_.free(int_rider_id);
}



void MatchingEngine::make_matches() {


    for (size_t i = 0; i < riders_.size(); ++i){
        Rider &rider = riders_[i];
        if (rider.state != OrderState::Active){
            continue;
        }
        double bid = rider.bid;

        int best_driver = -1;
        int second_driver = -1;
        double best_ask = std::numeric_limits<double>::infinity();
        double second_ask = std::numeric_limits<double>::infinity();

        // std::cout << rider.inbox.size() << std::endl;
        for (size_t i = 0; i < rider.inbox.size(); ++i){
            int driver_id = std::get<0>(rider.inbox[i]);
            auto ask_opt = std::get<1>(rider.inbox[i]);

            if (!ask_opt) continue;
            double ask = *ask_opt;
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
            
            match_pair(best_driver, i, best_ask, second_ask, bid);

        }

    }
}

void MatchingEngine::match_pair(int driver_id, int rider_id, double best_ask, double second_ask, double bid){
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

double MatchingEngine::haversine_distance(const Location a, const Location b) {

    constexpr double EARTH_RADIUS_KM = 6371.0;

    double lat1 = deg_to_rad(a.lat);
    double lon1 = deg_to_rad(a.lon);
    double lat2 = deg_to_rad(b.lat);
    double lon2 = deg_to_rad(b.lon);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double sin_dlat = std::sin(dlat * 0.5);
    double sin_dlon = std::sin(dlon * 0.5);

    double h =
        sin_dlat * sin_dlat +
        std::cos(lat1) * std::cos(lat2) *
        sin_dlon * sin_dlon;

    double c = 2.0 * std::atan2(std::sqrt(h), std::sqrt(1.0 - h));

    return EARTH_RADIUS_KM * c;   // distance in km
}


template<typename FreeListType>
std::vector<int> MatchingEngine::find_top_k_from_cells(
    Location loc,
    const std::unordered_map<H3Index, FreeList<int>>& h3_map,
    const FreeListType& free_list,
    size_t k)
{
    using Pair = std::pair<int, double>;
    auto comp = [](Pair a, Pair b){ return a.second < b.second; };

    std::priority_queue<Pair, std::vector<Pair>, decltype(comp)> heap(comp);

    auto cells = get_neighboring_cells(location_to_h3(loc, H3_RES), SEARCH_RADIUS);

    for (auto cell : cells) {
        auto iter = h3_map.find(cell);
        if (iter == h3_map.end()) continue;

        const auto& person_ids = iter->second;

        for (size_t i = 0; i < person_ids.size(); ++i) {
            int id = person_ids[i];
            const auto& person = free_list[id];

            double dist = haversine_distance(loc, person.loc);

            heap.emplace(id, dist);
            if (heap.size() > k) heap.pop();
        }
    }

    std::vector<int> result;
    while (!heap.empty()) {
        result.push_back(heap.top().first);
        heap.pop();
    }

    std::reverse(result.begin(), result.end());
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
            const FreeList<int>& flist = it->second;
            for (size_t i = 0; i < flist.size(); ++i) {
                int idx = flist[i];
                if (riders_[idx].state == OrderState::Active) {
                    result.push_back(idx);
                }
            }
        }
    }

    return result;
}