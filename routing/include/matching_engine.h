#pragma once

#include <unordered_map>
#include <optional>
#include <vector>
#include <list>
#include <string>
#include <h3/h3api.h>
#include <memory>
#include <sstream>
#include <iostream>

struct Location {
    
    double lat;
    double lon;

    Location(double lat_ = 0, double lon_ = 0 ) : lat(lat_), lon(lon_) {}

};

struct Rider {

    Location loc;
    double bid;
    int id;
    // std::list<int> drivers;
    std::unordered_map<int, std::optional<double>> inbox;


    Rider(int id_ = 0, double bid_ = 0, Location loc_ = Location()) : id(id_), bid(bid_), loc(loc_) {}

};

struct Driver {
    Location loc;
    int id;
    std::unordered_map<int, std::pair<double, std::optional<double>>> inbox;

    Driver(int id_ = 0, Location loc_ = Location()) : id(id_), loc(loc_) {}

};


class MatchingEngine{
    public:
        // MatchingEngine();
        // ~MatchingEngine();
        void add_rider(int id, double bid, double lat, double lon);
        void add_driver(int id, double lat, double lon);
        void cancel_rider(int id);
        void cancel_driver(int id);
        
        void driver_interest(int driver_id, int rider_id, double ask);
        void make_matches();

      


    private:

        static constexpr int H3_RES = 2;
        static constexpr int SEARCH_RADIUS = 5;
        static constexpr int K = 5;
        static constexpr int TIMEOUT_SEC = 300;




        std::unordered_map<int, Rider> riders_;
        std::unordered_map<int, Driver> drivers_;
        std::unordered_map<int, std::list<int>> adj_mat;
        std::unordered_map<H3Index, std::list<int>> drivers_by_cell_;
        std::unordered_map<H3Index, std::list<int>> riders_by_cell_;

        void clean_rider(int id);
        void clean_driver(int id);

        double haversine_distance(const Location a, const Location b);
        double deg_to_rad(double deg);
            
        void match_pair(int driver_id, int rider_id, double best_ask, double second_ask, double bid);
        std::vector<int> find_top_k(Location loc, const std::unordered_map<int, Location>& id_loc_map, size_t k);
        template<typename MapType> std::unordered_map<int, Location>  get_id_location_map(Location loc, const std::unordered_map<H3Index, std::list<int>> &h3_map, const MapType& person_map);
       

        H3Index location_to_h3(Location loc, int res) const;
        std::vector<H3Index> get_neighboring_cells(H3Index center, int radius) const;





};

