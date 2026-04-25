#include "engine/matching_engine.h"
#include "kafka/kafka_producer.h"
#include "kafka/kafka_consumer.h"
#include "events/spsc_queue.h"
#include "events/events.h"
#include "globals.h"
#include "threads.h"
#include "events/json_utils.h"
#include <nlohmann/json.hpp>

void pin_to_core(int core) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
}

// ── thread 1 — kafka consumer ─────────────────────────────────────────────────
// polls kafka, parses json, pushes EngineCommands into in_queue
void consumer_thread(SPSCQueue<EngineCommand, 65536>& in_queue) {
    pin_to_core(0);
    KafkaConsumer consumer("localhost:9092", "shard-0-worker", "shard-0-commands");

    while (running) {
        auto raw = consumer.poll(5);
        if (!raw) continue;

        try {
            EngineCommand cmd = parse(*raw);
            while (!in_queue.push(cmd)) {
                std::this_thread::yield();
            }
        } catch (const std::exception& e) {
            fprintf(stderr, "parse error: %s\n", e.what());
        }
    }
}

// ── thread 2 — engine ─────────────────────────────────────────────────────────
// drains in_queue, calls engine, pushes EngineEvents into out_queue
void engine_thread(SPSCQueue<EngineCommand, 65536>& in_queue,
                   SPSCQueue<EngineEvent,   65536>& out_queue) {
    pin_to_core(1);
    MatchingEngine engine;
    auto last_match = std::chrono::steady_clock::now();

    auto push = [&](EngineEvent ev) {
        while (!out_queue.push(ev)) std::this_thread::yield();
    };

    while (running) {
        while (auto cmd = in_queue.pop()) {
            EngineEvent ev;
            switch (cmd->type) {

                case EngineCommand::Type::ADD_RIDER: {
                    auto r = engine.add_rider(
                        cmd->add_rider.ext_id,
                        cmd->add_rider.bid,
                        cmd->add_rider.lat,
                        cmd->add_rider.lon,
                        cmd->add_rider.mode
                    );
                    ev.type = EngineEvent::Type::RIDER_ADDED;
                    ev.added_participant = {cmd->add_rider.ext_id, r.seq_no};
                    push(ev);
                    break;
                }

                case EngineCommand::Type::ADD_DRIVER: {
                    auto r = engine.add_driver(
                        cmd->add_driver.ext_id,
                        cmd->add_driver.lat,
                        cmd->add_driver.lon
                    );
                    ev.type = EngineEvent::Type::DRIVER_ADDED;
                    ev.added_participant = {cmd->add_driver.ext_id, r.seq_no};
                    push(ev);
                    break;
                }

                case EngineCommand::Type::DRIVER_INTEREST:
                    engine.driver_interest(
                        cmd->interest.ext_driver_id, cmd->interest.dsn,
                        cmd->interest.ext_rider_id,  cmd->interest.rsn,
                        cmd->interest.ask
                    );
                    break;

                case EngineCommand::Type::INSTANT_MATCH: {
                    auto r = engine.instant_match(
                        cmd->instant.ext_rider_id,  cmd->instant.rsn,
                        cmd->instant.ext_driver_id, cmd->instant.dsn
                    );
                    if (r) {
                        ev.type = EngineEvent::Type::MATCHED;
                        ev.matched_participant = {r->ext_rider_id, r->ext_driver_id, r->clearing_price};
                        push(ev);
                    }
                    break;
                }

                case EngineCommand::Type::CANCEL_RIDER: {
                    bool ok = engine.cancel_rider(
                        cmd->cancel.ext_id, cmd->cancel.sn
                    );
                    if (ok) {
                        ev.type = EngineEvent::Type::RIDER_CANCELED;
                        ev.canceled_participant = {cmd->cancel.ext_id};
                        push(ev);
                    }
                    break;
                }

                case EngineCommand::Type::CANCEL_DRIVER: {
                    bool ok = engine.cancel_driver(
                        cmd->cancel.ext_id, cmd->cancel.sn
                    );
                    if (ok) {
                        ev.type = EngineEvent::Type::DRIVER_CANCELED;
                        ev.canceled_participant = {cmd->cancel.ext_id};
                        push(ev);
                    }
                    break;
                }
            }
        }

        auto now     = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
                       (now - last_match).count();

        if (elapsed >= 30000) {
            auto results = engine.make_matches();
            for (auto& r : results) {
                EngineEvent ev;
                ev.type = EngineEvent::Type::MATCHED;
                ev.matched_participant = {r.ext_rider_id, r.ext_driver_id, r.clearing_price};
                push(ev);
            }
            last_match = now;
        }

        if (in_queue.empty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

// ── thread 3 — kafka producer ─────────────────────────────────────────────────
// drains out_queue, serializes events, produces to kafka
void producer_thread(SPSCQueue<EngineEvent, 65536>& out_queue) {
    pin_to_core(2);
    KafkaProducer producer("localhost:9092", "shard-0-events");

    while (running) {
        bool produced_any = false;

        while (auto ev = out_queue.pop()) {
            producer.produce(serialize(*ev));
            produced_any = true;
        }

        if (produced_any) {
            producer.poll(0);
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    producer.flush();
}