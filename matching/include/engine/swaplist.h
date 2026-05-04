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
