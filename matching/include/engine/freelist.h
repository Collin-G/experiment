#pragma once
    
#include <vector>
    #include <optional>
    #include <tuple>
    #include <vector>
    #include <list>
    #include <string>
    #include <h3/h3api.h>
    #include <memory>
    #include <sstream>
    #include <iostream>
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
