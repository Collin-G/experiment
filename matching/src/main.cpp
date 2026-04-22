#include "matching_engine.h"
#include "kafka_producer.h"
#include "kafka_consumer.h"
#include "spsc_queue.h"
#include <nlohmann/json.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include <optional>
#include <string>
#include <vector>

// ── shutdown flag ─────────────────────────────────────────────────────────────
static std::atomic<bool> running{true};

// ── command struct — parsed off wire, ready for engine ───────────────────────
struct EngineCommand {
    enum class Type : uint8_t {
        ADD_RIDER,
        ADD_DRIVER,
        DRIVER_INTEREST,
        INSTANT_MATCH,
        CANCEL_RIDER,
        CANCEL_DRIVER
    };

    Type type;

    union {
        struct { int ext_id; double bid; double lat; double lon; RiderMode mode; } add_rider;
        struct { int ext_id; double lat; double lon; }                             add_driver;
        struct { int ext_driver_id; int dsn; int ext_rider_id; int rsn; double ask; } interest;
        struct { int ext_rider_id; int rsn; int ext_driver_id; int dsn; }          instant;
        struct { int ext_id; int sn; }                                             cancel;
    };
};

// ── event struct — engine output, ready to serialize ─────────────────────────
struct EngineEvent {
    enum class Type : uint8_t {
        RIDER_ADDED,
        DRIVER_ADDED,
        MATCHED,
        RIDER_CANCELED,
        DRIVER_CANCELED
    };

    Type   type;
    int    ext_rider_id  = -1;
    int    ext_driver_id = -1;
    int    seq_no        = -1;
    double price         = 0.0;
};

// ── parse raw json string into EngineCommand ──────────────────────────────────
EngineCommand parse(const std::string& raw) {
    auto j = nlohmann::json::parse(raw);
    std::string type = j["type"];
    auto& p = j["payload"];

    EngineCommand cmd;

    if (type == "ADD_RIDER") {
        cmd.type = EngineCommand::Type::ADD_RIDER;
        cmd.add_rider = {
            p["ext_id"].get<int>(),
            p["bid"].get<double>(),
            p["lat"].get<double>(),
            p["lon"].get<double>(),
            p["mode"].get<std::string>() == "instant"
                ? RiderMode::Instant
                : RiderMode::Auction
        };
    } else if (type == "ADD_DRIVER") {
        cmd.type = EngineCommand::Type::ADD_DRIVER;
        cmd.add_driver = {
            p["ext_id"].get<int>(),
            p["lat"].get<double>(),
            p["lon"].get<double>()
        };
    } else if (type == "DRIVER_INTEREST") {
        cmd.type = EngineCommand::Type::DRIVER_INTEREST;
        cmd.interest = {
            p["ext_driver_id"].get<int>(),
            p["dsn"].get<int>(),
            p["ext_rider_id"].get<int>(),
            p["rsn"].get<int>(),
            p["ask"].get<double>()
        };
    } else if (type == "INSTANT_MATCH") {
        cmd.type = EngineCommand::Type::INSTANT_MATCH;
        cmd.instant = {
            p["ext_rider_id"].get<int>(),
            p["rsn"].get<int>(),
            p["ext_driver_id"].get<int>(),
            p["dsn"].get<int>()
        };
    } else if (type == "CANCEL_RIDER") {
        cmd.type = EngineCommand::Type::CANCEL_RIDER;
        cmd.cancel = {
            p["ext_id"].get<int>(),
            p["sn"].get<int>()
        };
    } else if (type == "CANCEL_DRIVER") {
        cmd.type = EngineCommand::Type::CANCEL_DRIVER;
        cmd.cancel = {
            p["ext_id"].get<int>(),
            p["sn"].get<int>()
        };
    }

    return cmd;
}

// ── serialize EngineEvent to json string ──────────────────────────────────────
std::string serialize(const EngineEvent& ev) {
    nlohmann::json j;
    switch (ev.type) {
        case EngineEvent::Type::RIDER_ADDED:
            j = {{"type", "RIDER_ADDED"},
                 {"ext_id", ev.ext_rider_id},
                 {"seq_no", ev.seq_no}};
            break;
        case EngineEvent::Type::DRIVER_ADDED:
            j = {{"type", "DRIVER_ADDED"},
                 {"ext_id", ev.ext_driver_id},
                 {"seq_no", ev.seq_no}};
            break;
        case EngineEvent::Type::MATCHED:
            j = {{"type",          "MATCHED"},
                 {"ext_rider_id",  ev.ext_rider_id},
                 {"ext_driver_id", ev.ext_driver_id},
                 {"price",         ev.price}};
            break;
        case EngineEvent::Type::RIDER_CANCELED:
            j = {{"type",   "RIDER_CANCELED"},
                 {"ext_id", ev.ext_rider_id}};
            break;
        case EngineEvent::Type::DRIVER_CANCELED:
            j = {{"type",   "DRIVER_CANCELED"},
                 {"ext_id", ev.ext_driver_id}};
            break;
    }
    return j.dump();
}

// ── pin thread to cpu core ────────────────────────────────────────────────────
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

    // helper to push event — spins if queue is full
    auto push = [&](EngineEvent ev) {
        while (!out_queue.push(ev)) std::this_thread::yield();
    };

    while (running) {
        // drain everything available in one tight loop
        EngineCommand cmd;
        while (auto cmd = in_queue.pop()) {
            switch (cmd->type) {

                case EngineCommand::Type::ADD_RIDER: {
                    auto r = engine.add_rider(
                        cmd->add_rider.ext_id,
                        cmd->add_rider.bid,
                        cmd->add_rider.lat,
                        cmd->add_rider.lon,
                        cmd->add_rider.mode
                    );
                    push(EngineEvent{
                        EngineEvent::Type::RIDER_ADDED,
                        cmd->add_rider.ext_id,
                        -1,
                        r.seq_no
                    });
                    break;
                }

                case EngineCommand::Type::ADD_DRIVER: {
                    auto r = engine.add_driver(
                        cmd->add_driver.ext_id,
                        cmd->add_driver.lat,
                        cmd->add_driver.lon
                    );
                    push(EngineEvent{
                        EngineEvent::Type::DRIVER_ADDED,
                        -1,
                        cmd->add_driver.ext_id,
                        r.seq_no
                    });
                    break;
                }

                case EngineCommand::Type::DRIVER_INTEREST:
                    engine.driver_interest(
                        cmd->interest.ext_driver_id, cmd->interest.dsn,
                        cmd->interest.ext_rider_id,  cmd->interest.rsn,
                        cmd->interest.ask
                    );
                    // no event produced — interest is internal engine state
                    break;

                case EngineCommand::Type::INSTANT_MATCH: {
                    auto r = engine.instant_match(
                        cmd->instant.ext_rider_id,  cmd->instant.rsn,
                        cmd->instant.ext_driver_id, cmd->instant.dsn
                    );
                    if (r) {
                        push(EngineEvent{
                            EngineEvent::Type::MATCHED,
                            r->ext_rider_id,
                            r->ext_driver_id,
                            -1,
                            r->clearing_price
                        });
                    }
                    break;
                }

                case EngineCommand::Type::CANCEL_RIDER: {
                    bool ok = engine.cancel_rider(
                        cmd->cancel.ext_id, cmd->cancel.sn
                    );
                    if (ok) {
                        push(EngineEvent{
                            EngineEvent::Type::RIDER_CANCELED,
                            cmd->cancel.ext_id
                        });
                    }
                    break;
                }

                case EngineCommand::Type::CANCEL_DRIVER: {
                    bool ok = engine.cancel_driver(
                        cmd->cancel.ext_id, cmd->cancel.sn
                    );
                    if (ok) {
                        push(EngineEvent{
                            EngineEvent::Type::DRIVER_CANCELED,
                            -1,
                            cmd->cancel.ext_id
                        });
                    }
                    break;
                }
            }
        }

        // check match interval
        auto now     = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
                       (now - last_match).count();

        if (elapsed >= 30000) {
            auto results = engine.make_matches();
            for (auto& r : results) {
                push(EngineEvent{
                    EngineEvent::Type::MATCHED,
                    r.ext_rider_id,
                    r.ext_driver_id,
                    -1,
                    r.clearing_price
                });
            }
            last_match = now;
        }

        // don't busy spin when idle
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