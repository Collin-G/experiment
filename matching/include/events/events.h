#pragma once
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include <optional>
#include <string>
#include <vector>
#include "engine/matching_engine.h"

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
    union {
        struct {int ext_id; int seq_no; int shard_id; double lat; double lon;} added_participant;
        struct {int ext_rider_id; int ext_driver_id; double price; int shard_id;} matched_participant;
        struct {int ext_id; int shard_id;} canceled_participant;

    };

};