#pragma once

#include <unordered_map>
#include <optional>
#include <tuple>
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


enum class OrderState{
    Active,
    Matched,
    Canceled,
};


template<typename T>
class FreeList {
    private:
        std::vector<T> pool;
        std::vector<int> free_list;
    
    public:

        int allocate (const T &item){
            if (!free_list.empty()){
                int idx = free_list.back();
                free_list.pop_back();
                pool[idx] = item;   
                return idx;             
            }

            else{
                pool.push_back(item);
                return pool.size()-1;
            }
        }

        void free(int idx){
            free_list.push_back(idx);
            // pool[idx].state = 
        }

        T& operator[](int idx){
            return pool[idx];
        }

        const T& operator[](int idx) const {
        return pool[idx];
        }


        size_t size() const { return pool.size(); }
};

struct Rider {

    Location loc;
    double bid;
    int ext_id;
    FreeList<std::tuple<int,std::optional<double>>> inbox;
    OrderState state = OrderState::Active;

    Rider(int ext_id_ = 0, double bid_ = 0, Location loc_ = Location()) : ext_id(ext_id_), bid(bid_), loc(loc_) {}

};

struct Driver {
    Location loc;
    int ext_id;
    FreeList<std::tuple<int, double, std::optional<double>>> inbox;
    OrderState state = OrderState::Active;


    Driver(int ext_id_ = 0, Location loc_ = Location()) : ext_id(ext_id_), loc(loc_) {}

};


class MatchingEngine{
    public:
        // MatchingEngine();
        // ~MatchingEngine();
        int add_rider(int ext_id, double bid, double lat, double lon);
        int add_driver(int ext_id, double lat, double lon);
        void cancel_rider(int int_id);
        void cancel_driver(int int_id);
        std::vector<int> get_top_k_riders_for_driver(int int_driver_id);
        
        void driver_interest(int int_driver_id, int int_rider_id, double ask);
        void make_matches();

      


    private:

        static constexpr int H3_RES = 9;
        static constexpr int SEARCH_RADIUS = 2;
        static constexpr int K = 5;
        static constexpr int TIMEOUT_SEC = 300;

        FreeList<Rider> riders_;
        FreeList<Driver> drivers_;

        
        // std::unordered_map<int, Driver> drivers_;
        std::unordered_map<int, std::list<int>> adj_mat;
        std::unordered_map<H3Index, FreeList<int>> drivers_by_cell_;
        std::unordered_map<H3Index, FreeList<int>> riders_by_cell_;

        void clean_rider(int int_id);
        void clean_driver(int int_id);

        double haversine_distance(const Location a, const Location b);
        double deg_to_rad(double deg);
            
        void match_pair(int int_driver_id, int int_rider_id, double best_ask, double second_ask, double bid);
        
        template<typename FreeListType>
        std::vector<int> find_top_k_from_cells(
        Location loc,
        const std::unordered_map<H3Index, FreeList<int>>& h3_map,
        const FreeListType& free_list,
        size_t k);

       

        H3Index location_to_h3(Location loc, int res) const;
        std::vector<H3Index> get_neighboring_cells(H3Index center, int radius) const;

        





};

