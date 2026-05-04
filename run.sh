#!/bin/bash
set -e

# auto-recover broken docker socket
if ! docker ps > /dev/null 2>&1; then
    echo "docker socket broken, recovering..."
    sudo pkill -9 dockerd 2>/dev/null || true
    sudo rm -f /var/run/docker.pid
    sudo rm -f /var/run/docker.sock
    sudo systemctl reset-failed docker 2>/dev/null || true
    sudo systemctl start docker
    sleep 3
fi

SHARD_ID=${1:-0}  # take first argument, default to 0

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

NUM_SHARDS=3
# ── create topics (only once, so check if they exist first) ───────────────────
for TOPIC in commands events; do
    docker exec $KAFKA kafka-topics --create \
        --bootstrap-server localhost:9092 \
        --topic $TOPIC \
        --partitions $NUM_SHARDS \
        --replication-factor 1 \
        --if-not-exists
    echo "topic $TOPIC ready"
done

# ── start engine ──────────────────────────────────────────────────────────────
echo "starting engine for shard $SHARD_ID..."
matching/build/matching_cli --shard_id $SHARD_ID