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
    #include "ankerl/unordered_dense.h"

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
            std::vector<int> seq_nos;
            size_t active_count = 0;
        
        public:

            int allocate (const T &item){
                ++active_count;
                if (!free_list.empty()){
                    int idx = free_list.back();
                    free_list.pop_back();
                    pool[idx] = item;            
                    return idx;             
                }

                else{
                    pool.push_back(item);
                    seq_nos.push_back(0);
                    return pool.size()-1;
                }
            }

            void free(int idx){
                free_list.push_back(idx);
                ++seq_nos[idx]; 
                --active_count;
            }

            T& operator[](int idx){
                return pool[idx];
            }

            const T& operator[](int idx) const {
            return pool[idx];
            }

            int seq_no(int idx) const{
                return seq_nos[idx];
            }

            size_t active_size() const {return active_count;}
            size_t size() const { return pool.size(); }
    };


    template<typename T>
    class SwapList{
        private:
            std::vector<T> pool;

        public:
            T& operator[](int idx) {
                return pool[idx];
            }

            const T& operator[](int idx) const {
                return pool[idx];
            }

            void push_back(const T& item){
                pool.push_back(item);
            }

            void free(int idx){
                T& temp = pool[idx];
                pool[idx] = pool[pool.size()-1];
                pool.pop_back();
            }

            T& front(){
                return pool.front();
            }

            void clear(){
                pool.clear();
            }

            bool empty(){
                return pool.empty();
            }

       
            size_t size() const { return pool.size(); }

     };



    struct CellState {
        size_t cursor = 0;
        SwapList<std::tuple<int, int>> swap_list;
    };
    
    struct Rider {

        Location loc;
        double bid;
        int ext_id;
        int h3_id;
        int inbox_id;
        bool interesting = false;
        FreeList<std::tuple<int, int, std::optional<double>>> inbox;
        SwapList<std::tuple<int, int, double>> interested_drivers;
        OrderState state = OrderState::Active;

        Rider(int ext_id_ = 0, double bid_ = 0, Location loc_ = Location()) : ext_id(ext_id_), bid(bid_), loc(loc_) {}

    };

    struct Driver {
        Location loc;
        int ext_id;
        int h3_id;
        int inbox_id;
        FreeList<std::tuple<int, int, double, std::optional<double>>> inbox;
        SwapList<std::tuple<int, double>> interesting_riders;
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

            static constexpr int H3_RES = 8;
            static constexpr int SEARCH_RADIUS = 1;
            static constexpr int K = 20;
            static constexpr int TIMEOUT_SEC = 300;

            FreeList<Rider> riders_;
            FreeList<Driver> drivers_;
            SwapList<std::tuple<int, int>> interest_map_; 

            
            // ankerl::unordered_dense::map<int, Driver> drivers_;
            ankerl::unordered_dense::map<H3Index, CellState> drivers_by_cell_;
            ankerl::unordered_dense::map<H3Index, CellState> riders_by_cell_;

            void clean_rider(int int_id);
            void clean_driver(int int_id);

            double fast_approx_distance_km(const Location &a, const Location &b);
            double deg_to_rad(double deg);
                
            void match_pair(int int_driver_id, int int_rider_id, double best_ask, double second_ask, double bid);
            
            template<typename FreeListType>
            std::vector<int> find_top_k_from_cells(
            Location loc,
            ankerl::unordered_dense::map<H3Index, CellState>& cell_map,
            FreeListType& free_list,
            size_t k);

        

            H3Index location_to_h3(Location loc, int res) const;
            std::vector<H3Index> get_neighboring_cells(H3Index center, int radius) const;

    };

