#!/bin/bash
echo "Stopping all C++ engines..."
pkill -f "matching/build/matching_cli" 2>/dev/null || true
echo "Stopping all Go shard workers..."
pkill -f "shard_worker/main.go" 2>/dev/null || true
echo "Stopping Go gateway..."
pkill -f "gateway/main.go" 2>/dev/null || true
echo "Stopping Kafka and Redis containers..."
docker-compose -f docker/docker-compose.yml down 2>/dev/null || true
echo "All services stopped."