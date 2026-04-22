#include <atomic>
#include <thread>
#include <chrono>
#include <optional>
#include <array>

template<typename T, size_t SIZE>
class SPSCQueue {
    public:
        bool push(const T& item){
            size_t tail = tail_.load(std::memory_order_relaxed);
            size_t next = (tail+1) & (SIZE-1);
            if (next == head_.load(std::memory_order_acquire)) return false;
            buffer_[tail] = item;
            tail_.store(next, std::memory_order_release);
            return true;
        }

        std::optional<T> pop(){
            size_t head = head_.load(std::memory_order_relaxed);
            if (head == tail_.load(std::memory_order_acquire)) return std::nullopt;
            T item = buffer_[head];
            head_.store((head+1) & (SIZE-1), std::memory_order_release);
            return item;
        }

        bool empty(){
             return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
        }

    private:
        static_assert((SIZE & (SIZE-1)) == 0, "size must be power of 2");
        alignas(64) std::atomic<size_t> head_{0};
        alignas(64) std::atomic<size_t> tail_{0};

        std::array<T, SIZE> buffer_;

};