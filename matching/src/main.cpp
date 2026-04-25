#include "engine/matching_engine.h"
#include "kafka/kafka_producer.h"
#include "kafka/kafka_consumer.h"
#include "events/spsc_queue.h"
#include "events/events.h"
#include "globals.h"
#include "threads.h"
#include <nlohmann/json.hpp>






// ── main ──────────────────────────────────────────────────────────────────────
int main() {
    // handle ctrl+c gracefully
    std::signal(SIGINT,  [](int){ running = false; });
    std::signal(SIGTERM, [](int){ running = false; });

    SPSCQueue<EngineCommand, 65536> in_queue;
    SPSCQueue<EngineEvent,   65536> out_queue;

    std::thread t1([&]{ consumer_thread(in_queue); });
    std::thread t2([&]{ engine_thread(in_queue, out_queue); });
    std::thread t3([&]{ producer_thread(out_queue); });

    t1.join();
    t2.join();
    t3.join();

    return 0;
}