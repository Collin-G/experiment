#!/bin/bash
set -e

COMPOSE="docker-compose -f docker/docker-compose.yml"
KAFKA="docker-kafka-1"

# ── start services ────────────────────────────────────────────────────────────
echo "starting kafka and redis..."
$COMPOSE up -d

# ── wait for kafka ────────────────────────────────────────────────────────────
echo "waiting for kafka..."
until docker exec $KAFKA \
    kafka-topics --bootstrap-server localhost:9092 --list > /dev/null 2>&1; do
    sleep 2
done
echo "kafka ready"

# ── create topics ─────────────────────────────────────────────────────────────
for TOPIC in shard-0-commands shard-0-events; do
    docker exec $KAFKA kafka-topics --create \
        --bootstrap-server localhost:9092 \
        --topic $TOPIC \
        --partitions 1 \
        --replication-factor 1 \
        --if-not-exists
    echo "topic $TOPIC ready"
done

# ── start engine ──────────────────────────────────────────────────────────────
echo "starting engine..."
exec matching/build/matching_cli