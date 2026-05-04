#include "engine/matching_engine.h"
#include "kafka/kafka_producer.h"
#include "kafka/kafka_consumer.h"
#include "events/spsc_queue.h"
#include "events/events.h"
#include "globals.h"
#include "threads.h"
#include "events/json_utils.h"
#include <nlohmann/json.hpp>

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
            {"ext_id", ev.added_participant.ext_id},
            {"seq_no", ev.added_participant.seq_no},
            {"shard_id", ev.added_participant.shard_id},   // add this line
            {"lat", ev.added_participant.lat},
            {"lon", ev.added_participant.lon}};
        break;

    case EngineEvent::Type::DRIVER_ADDED:
        j = {{"type", "DRIVER_ADDED"},
            {"ext_id", ev.added_participant.ext_id},
            {"seq_no", ev.added_participant.seq_no},
            {"shard_id", ev.added_participant.shard_id},  // add this line
            {"lat", ev.added_participant.lat},
            {"lon", ev.added_participant.lon}};
        break;

    case EngineEvent::Type::MATCHED:
        j = {{"type", "MATCHED"},
            {"ext_rider_id", ev.matched_participant.ext_rider_id},
            {"ext_driver_id", ev.matched_participant.ext_driver_id},
            {"price", ev.matched_participant.price},
            {"shard_id", ev.matched_participant.shard_id}};   // add this line
        break;

    case EngineEvent::Type::RIDER_CANCELED:
        j = {{"type", "RIDER_CANCELED"},
            {"ext_id", ev.canceled_participant.ext_id},
            {"shard_id", ev.canceled_participant.shard_id}};   // add this line
        break;

    case EngineEvent::Type::DRIVER_CANCELED:
        j = {{"type", "DRIVER_CANCELED"},
            {"ext_id", ev.canceled_participant.ext_id},
            {"shard_id", ev.canceled_participant.shard_id}};   // add this line
        break;
    }
    return j.dump();
}