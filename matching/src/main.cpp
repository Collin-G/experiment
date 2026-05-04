#include "engine/matching_engine.h"
#include "kafka/kafka_producer.h"
#include "kafka/kafka_consumer.h"
#include "events/spsc_queue.h"
#include "events/events.h"
#include "globals.h"
#include "threads.h"
#include <nlohmann/json.hpp>






// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {

    int shard_id = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--shard_id") == 0 && i+1 < argc) {
            shard_id = atoi(argv[i+1]);
        }
    }

    // handle ctrl+c gracefully
    std::signal(SIGINT,  [](int){ running = false; });
    std::signal(SIGTERM, [](int){ running = false; });

    SPSCQueue<EngineCommand, 65536> in_queue;
    SPSCQueue<EngineEvent,   65536> out_queue;

    std::thread t1([&]{ consumer_thread(in_queue, shard_id); });
    std::thread t2([&]{ engine_thread(in_queue, out_queue, shard_id); });
    std::thread t3([&]{ producer_thread(out_queue, shard_id); });

    t1.join();
    t2.join();
    t3.join();

    return 0;
}