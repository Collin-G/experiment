#include "matching_engine.h"
#include <curl/curl.h>
#include <string>
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <nlohmann/json.hpp>


template<typename MapType>
std::unordered_map<int, Location> MatchingEngine::get_id_location_map(Location loc, const std::unordered_map<H3Index, std::list<int>>& h3_map, const MapType& person_map){
    std::unordered_map<int, Location> loc_map;
    std::vector<H3Index> cells = get_neighboring_cells(location_to_h3(loc, H3_RES), SEARCH_RADIUS);

    for (auto cell : cells){
       auto iter = h3_map.find(cell);
       if (iter != h3_map.end()) {
            const std::list<int>& person_ids = iter->second;
            for (int person_id : person_ids) {
                auto it = person_map.find(person_id);
                if (it != person_map.end()){
                    loc_map.emplace(person_id, it->second.loc);
                }
               
            }

        }
    }

    return loc_map;

}

void MatchingEngine::add_rider(int ext_id, double bid, double lat, double lon) {
    Location loc = Location(lat, lon);
    Rider rider = Rider(ext_id,  bid, loc);

    std::unordered_map<int, Location> id_loc_map = get_id_location_map(loc, drivers_by_cell_, drivers_);    
    std::vector<int> closest_drivers = find_top_k(loc, id_loc_map, K);

    int int_rider_id = riders_.allocate(rider);

    for (auto int_driver_id : closest_drivers){
        Driver &driver = drivers_[int_driver_id];
        driver.inbox.emplace(int_rider_id, std::make_pair(bid, std::nullopt));
        rider.inbox.emplace(int_driver_id, std::nullopt);

    }

    H3Index cell = location_to_h3(loc, H3_RES);
    auto& lst = riders_by_cell_[cell];
    if (std::find(lst.begin(), lst.end(), ext_id) == lst.end()) lst.push_back(int_rider_id);

}


void MatchingEngine::add_driver(int ext_driver_id, double lat, double lon){
    Location loc = Location(lat, lon);
    Driver driver = Driver(ext_driver_id, loc);

    std::unordered_map<int, Location> id_loc_map = get_id_location_map(loc, riders_by_cell_, riders_);
    std::vector<int> closest_riders = find_top_k(loc, id_loc_map, K);

    int int_driver_id = drivers_.allocate(driver);

    for (auto int_rider_id: closest_riders){
        Rider &rider = riders_[int_rider_id];
        driver.inbox.emplace(int_rider_id, std::make_pair(rider.bid, std::nullopt));
        rider.inbox.emplace(int_driver_id, std::nullopt);

    }

    H3Index cell = location_to_h3(loc, H3_RES);
    auto& lst = drivers_by_cell_[cell];
    if (std::find(lst.begin(), lst.end(), int_driver_id) == lst.end()) lst.push_back(int_driver_id);


}


void MatchingEngine::driver_interest(int int_driver_id, int int_rider_id, double ask){
    // auto r_iter = riders_.find(rider_id);
    // auto d_iter = drivers_.find(driver_id);
    Driver &driver = drivers_[int_driver_id];
    Rider &rider = riders_[int_rider_id];

    if (rider.bid < ask){
        return;
    }

    driver.inbox[int_rider_id].second = ask;
    rider.inbox[int_driver_id] = ask;
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
        lst.remove(int_driver_id); // std::list has remove
        if (lst.empty()) {
            drivers_by_cell_.erase(h3_iter);
        }
    }

    // 2. Remove driver from riders' driver lists
    
    for (const auto & [int_rider_id, _] : driver.inbox) {
        Rider &rider = riders_[int_rider_id];
        rider.inbox.erase(int_driver_id);
       
    }

    // 3. Finally, remove driver from drivers_ map
    drivers_.free(int_driver_id);
}

void MatchingEngine::clean_rider(int int_rider_id) {


    Rider &rider = riders_[int_rider_id];



    // Remove rider from each driver's inbox
    for (const auto & [int_driver_id, _] : rider.inbox) {
        Driver &driver = drivers_[int_driver_id];
        driver.inbox.erase(int_rider_id);
   
    }

    


    H3Index cell = location_to_h3(rider.loc, H3_RES);
    auto h3_iter = riders_by_cell_.find(cell);
    if (h3_iter != riders_by_cell_.end()) {
        auto &lst = h3_iter->second;
        lst.remove(int_rider_id); // std::list has remove
        if (lst.empty()) {
            riders_by_cell_.erase(h3_iter);
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

        for (auto &[driver_id, ask_opt] : rider.inbox) {
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
    std::cout << "Matched driver " << driver_id 
              << " with rider " << rider_id << std::endl;

    

    
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

// WriteCallback for curl
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}



std::vector<int> MatchingEngine::find_top_k(
    Location loc,
    const std::unordered_map<int, Location> &map,
    size_t k)
{
    if (map.empty() || k == 0) return {};

    
    // Create our max heap
    using Pair = std::pair<int, double>;
    auto comp = [](Pair a, Pair b){return a.second < b.second;};
    std::priority_queue<Pair, std::vector<Pair>, decltype(comp)> heap(comp);


    for (const auto& [person_id, person_loc] : map) {
        double distance = haversine_distance(person_loc, loc);
        heap.emplace(person_id, distance);
        if (heap.size() > k) heap.pop();
        // person_order.push_back(person_id);
    }

 

    std::vector<int> top_k ;

    while (!heap.empty()) {
        top_k.push_back(heap.top().first);
        heap.pop();
    }

    std::reverse(top_k.begin(), top_k.end());
    return top_k;
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




