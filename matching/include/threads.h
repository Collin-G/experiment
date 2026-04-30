#pragma once

#include "events/spsc_queue.h"
#include "events/events.h"
#include "kafka/kafka_consumer.h"

// ── pin thread to cpu core ────────────────────────────────────────────────────


void pin_to_core(int core);

void consumer_thread(SPSCQueue<EngineCommand, 65536>& in_queue, int shard_id);

void engine_thread(SPSCQueue<EngineCommand, 65536>& in_queue,
                   SPSCQueue<EngineEvent,   65536>& out_queue, int shard_id);

void producer_thread(SPSCQueue<EngineEvent, 65536>& out_queue, int shard_id);

