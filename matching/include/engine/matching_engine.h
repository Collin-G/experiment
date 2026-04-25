    #pragma once


    #include "engine/freelist.h"
    #include "engine/swaplist.h"

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

    
    struct AddResult {
    int int_id;
    int seq_no;
};

struct MatchResult {
    int ext_rider_id;
    int ext_driver_id;
    double clearing_price;
};


    enum class OrderState{
        Active,
        Matched,
        Canceled,
    };

    enum class RiderMode{
        Auction,
        Instant
    };



    


    struct CellState {
        size_t cursor = 0;
        SwapList<std::tuple<int, int>> swap_list;
    };
    
    struct Rider {

        Location loc;
        double bid;
        int ext_id;
        bool interesting = false;
        SwapList<std::tuple<int, int, double>> interested_drivers;
        OrderState state = OrderState::Active;
        RiderMode mode;

        Rider(int ext_id_ = 0, double bid_ = 0, Location loc_ = Location(), RiderMode mode_ = RiderMode::Auction) : ext_id(ext_id_), bid(bid_), loc(loc_), mode(mode_)  {}

    };

    struct Driver {
        Location loc;
        int ext_id;
        OrderState state = OrderState::Active;


        Driver(int ext_id_ = 0, Location loc_ = Location()) : ext_id(ext_id_), loc(loc_) {}

    };


    class MatchingEngine{
        public:
            // MatchingEngine();
            // ~MatchingEngine();
            AddResult add_rider(int ext_id, double bid, double lat, double lon, RiderMode mode);
            AddResult add_driver(int ext_id, double lat, double lon);
            bool cancel_rider(int ext_id, int sn);
            bool cancel_driver(int ext_id, int sn);
            
            std::optional<MatchResult> instant_match(int ext_rider_id, int rsn, int ext_driver_id, int dsn);
            void driver_interest(int ext_driver_id, int dsn, int ext_rider_id, int rsn, double ask);
            std::vector<MatchResult> make_matches();
            const FreeList<Driver>& get_drivers() const {
                return drivers_;
            }

            const FreeList<Rider>& get_riders() const {
                return riders_;
            }

        


        private:

            
            FreeList<Rider> riders_;
            FreeList<Driver> drivers_;
            SwapList<std::tuple<int, int>> interest_map_; 

            size_t riders_cursor_ = 0;
            size_t drivers_cursor_ = 0;



            ankerl::unordered_dense::map<int64_t, int> rider_ext_to_int;
            ankerl::unordered_dense::map<int64_t, int> driver_ext_to_int;

            void clean_rider(int int_id);
            void clean_driver(int int_id);

                
            void match_pair(int int_driver_id, int int_rider_id, double best_ask, double second_ask, double bid);
            

    };

