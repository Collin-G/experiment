#!/bin/bash
set -e

export NUM_SHARDS=3
export LOG_DIR="./logs"
mkdir -p "$LOG_DIR"

# Function to stop all background jobs on exit
cleanup() {
    echo "Stopping all services..."
    kill $(jobs -p) 2>/dev/null || true
    wait 2>/dev/null || true
    echo "All services stopped."
}
trap cleanup EXIT INT TERM

# 1. Start infrastructure (Kafka, Redis)
echo "Starting Kafka and Redis via docker-compose..."
docker-compose -f docker/docker-compose.yml up -d

# 2. Wait for Kafka to be ready
echo "Waiting for Kafka..."
until docker exec docker-kafka-1 kafka-topics --bootstrap-server localhost:9092 --list > /dev/null 2>&1; do
    sleep 2
done
echo "Kafka ready."

# 3. Create topics (idempotent)
for TOPIC in commands events; do
    docker exec docker-kafka-1 kafka-topics --create \
        --bootstrap-server localhost:9092 \
        --topic $TOPIC \
        --partitions $NUM_SHARDS \
        --replication-factor 1 \
        --if-not-exists
    echo "Topic $TOPIC created."
done

# 4. Start C++ engines (one per shard)
for i in $(seq 0 $((NUM_SHARDS-1))); do
    echo "Starting C++ engine for shard $i..."
    # Call run.sh without exec (it will run in background)
    bash run.sh "$i" > "$LOG_DIR/engine_$i.log" 2>&1 &
    # Wait a bit to avoid overwhelming the system
    sleep 0.2
done

# 5. Start Go shard workers (one per shard)
for i in $(seq 0 $((NUM_SHARDS-1))); do
    echo "Starting Go shard worker for shard $i..."
    (cd shard_worker && go run main.go "$i") > "$LOG_DIR/shard_worker_$i.log" 2>&1 &
    sleep 0.2
done

# 6. Start Go gateway
echo "Starting Go gateway..."
(cd gateway && go run main.go) > "$LOG_DIR/gateway.log" 2>&1 &

echo "All services started. Logs are in $LOG_DIR/"
echo "Press Ctrl+C to stop all services."
wait