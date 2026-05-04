package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/confluentinc/confluent-kafka-go/v2/kafka"
	"github.com/gin-gonic/gin"
	"github.com/redis/go-redis/v9"
)

// ------------------------- Configuration -------------------------
const (
	kafkaBroker   = "localhost:9092"
	commandsTopic = "commands"
	eventsTopic   = "events"
	redisAddr     = "localhost:6379"
)

// ------------------------- Request/Response types -------------------------
type AddRiderReq struct {
	ExtID int     `json:"ext_id"`
	Bid   float64 `json:"bid"`
	Lat   float64 `json:"lat"`
	Lon   float64 `json:"lon"`
	Mode  string  `json:"mode"`
}

type AddDriverReq struct {
	ExtID int     `json:"ext_id"`
	Lat   float64 `json:"lat"`
	Lon   float64 `json:"lon"`
}

type DriverInterestReq struct {
	ExtDriverID int     `json:"ext_driver_id"`
	Dsn         int     `json:"dsn"`
	ExtRiderID  int     `json:"ext_rider_id"`
	Rsn         int     `json:"rsn"`
	Ask         float64 `json:"ask"`
}

type InstantMatchReq struct {
	ExtRiderID  int `json:"ext_rider_id"`
	Rsn         int `json:"rsn"`
	ExtDriverID int `json:"ext_driver_id"`
	Dsn         int `json:"dsn"`
}

type CancelReq struct {
	ExtID int `json:"ext_id"`
	Sn    int `json:"sn"`
}

// ------------------------- Kafka producer -------------------------
var kafkaProducer *kafka.Producer

func initKafkaProducer() error {
	p, err := kafka.NewProducer(&kafka.ConfigMap{
		"bootstrap.servers": kafkaBroker,
		"acks":              "all",
	})
	if err != nil {
		return err
	}
	kafkaProducer = p
	return nil
}

func produceCommand(topic string, partition int, value interface{}) error {
	data, err := json.Marshal(value)
	if err != nil {
		return err
	}
	return kafkaProducer.Produce(&kafka.Message{
		TopicPartition: kafka.TopicPartition{Topic: &topic, Partition: int32(partition)},
		Value:          data,
	}, nil)
}

// ------------------------- Redis helpers -------------------------
func storeRiderMapping(rdb *redis.Client, extID, shardID, seqNo int, lat, lon float64, mode string, bid float64) {
	key := fmt.Sprintf("rider:%d", extID)
	rdb.HSet(context.Background(), key,
		"shard", shardID,
		"seq_no", seqNo,
		"lat", lat,
		"lon", lon,
		"mode", mode,
		"bid", bid,
	)
	rdb.Expire(context.Background(), key, 24*time.Hour)
}

func getRiderMode(rdb *redis.Client, extID int) (string, error) {
	key := fmt.Sprintf("rider:%d", extID)
	mode, err := rdb.HGet(context.Background(), key, "mode").Result()
	if err != nil {
		return "", err
	}
	return mode, nil
}

func storeDriverMapping(rdb *redis.Client, extID, shardID, seqNo int, lat, lon float64) {
	key := fmt.Sprintf("driver:%d", extID)
	rdb.HSet(context.Background(), key,
		"shard", shardID,
		"seq_no", seqNo,
		"lat", lat,
		"lon", lon,
	)
	rdb.Expire(context.Background(), key, 24*time.Hour)
}

// ------------------------- HTTP handlers for gateway -------------------------
func handleAddRider(rdb *redis.Client, shardID int) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req AddRiderReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		storeRiderMapping(rdb, req.ExtID, shardID, 0, req.Lat, req.Lon, req.Mode, req.Bid)

		cmd := map[string]interface{}{
			"type":    "ADD_RIDER",
			"payload": req,
		}
		if err := produceCommand(commandsTopic, shardID, cmd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "kafka produce failed"})
			return
		}
		c.JSON(http.StatusAccepted, gin.H{"status": "accepted"})
	}
}

func handleAddDriver(rdb *redis.Client, shardID int) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req AddDriverReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		storeDriverMapping(rdb, req.ExtID, shardID, 0, req.Lat, req.Lon)

		cmd := map[string]interface{}{
			"type":    "ADD_DRIVER",
			"payload": req,
		}
		if err := produceCommand(commandsTopic, shardID, cmd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "kafka produce failed"})
			return
		}
		c.JSON(http.StatusAccepted, gin.H{"status": "accepted"})
	}
}

func handleDriverInterest(rdb *redis.Client, shardID int) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req DriverInterestReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		cmd := map[string]interface{}{
			"type":    "DRIVER_INTEREST",
			"payload": req,
		}
		if err := produceCommand(commandsTopic, shardID, cmd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "kafka produce failed"})
			return
		}
		c.JSON(http.StatusAccepted, gin.H{"status": "accepted"})
	}
}

func handleInstantMatch(rdb *redis.Client, shardID int) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req InstantMatchReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}

		// Validate rider mode
		mode, err := getRiderMode(rdb, req.ExtRiderID)
		if err != nil {
			c.JSON(http.StatusNotFound, gin.H{"error": "rider not found"})
			return
		}
		if mode != "instant" {
			c.JSON(http.StatusBadRequest, gin.H{"error": "instant match only allowed for riders with mode 'instant'"})
			return
		}

		cmd := map[string]interface{}{
			"type":    "INSTANT_MATCH",
			"payload": req,
		}
		if err := produceCommand(commandsTopic, shardID, cmd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "kafka produce failed"})
			return
		}
		c.JSON(http.StatusAccepted, gin.H{"status": "accepted"})
	}
}

func handleCancelRider(rdb *redis.Client, shardID int) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req CancelReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		cmd := map[string]interface{}{
			"type":    "CANCEL_RIDER",
			"payload": req,
		}
		if err := produceCommand(commandsTopic, shardID, cmd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "kafka produce failed"})
			return
		}
		c.JSON(http.StatusAccepted, gin.H{"status": "accepted"})
	}
}

func handleCancelDriver(rdb *redis.Client, shardID int) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req CancelReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		cmd := map[string]interface{}{
			"type":    "CANCEL_DRIVER",
			"payload": req,
		}
		if err := produceCommand(commandsTopic, shardID, cmd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "kafka produce failed"})
			return
		}
		c.JSON(http.StatusAccepted, gin.H{"status": "accepted"})
	}
}

// ------------------------- Kafka event consumer -------------------------
func startEventConsumer(rdb *redis.Client, shardID int) {
	consumer, err := kafka.NewConsumer(&kafka.ConfigMap{
		"bootstrap.servers": kafkaBroker,
		"group.id":          fmt.Sprintf("shard-worker-%d", shardID),
		"auto.offset.reset": "earliest",
	})
	if err != nil {
		log.Fatalf("Failed to create consumer: %v", err)
	}
	defer consumer.Close()

	topic := eventsTopic
	partition := shardID
	tp := kafka.TopicPartition{Topic: &topic, Partition: int32(partition)}
	if err := consumer.Assign([]kafka.TopicPartition{tp}); err != nil {
		log.Fatalf("Failed to assign partition: %v", err)
	}

	log.Printf("Event consumer started for shard %d, partition %d", shardID, partition)

	for {
		msg, err := consumer.ReadMessage(-1)
		if err != nil {
			log.Printf("Consumer error: %v", err)
			continue
		}

		var event map[string]interface{}
		if err := json.Unmarshal(msg.Value, &event); err != nil {
			log.Printf("Failed to parse event: %v", err)
			continue
		}

		typ, _ := event["type"].(string)
		switch typ {
		case "RIDER_ADDED":
			extID, _ := event["ext_id"].(float64)
			seqNo, _ := event["seq_no"].(float64)
			shard, _ := event["shard_id"].(float64)
			lat, _ := event["lat"].(float64)
			lon, _ := event["lon"].(float64)

			// Update only seq_no, lat, lon – preserve existing fields like mode
			key := fmt.Sprintf("rider:%d", int(extID))
			rdb.HSet(context.Background(), key,
				"seq_no", int(seqNo),
				"lat", lat,
				"lon", lon,
			)
			rdb.Expire(context.Background(), key, 24*time.Hour)
			// Add to geo set
			geoKey := fmt.Sprintf("riders:active:shard%d", int(shard))
			rdb.GeoAdd(context.Background(), geoKey, &redis.GeoLocation{
				Name:      fmt.Sprintf("%d", int(extID)),
				Longitude: lon,
				Latitude:  lat,
			})
			log.Printf("RIDER_ADDED: ext_id=%d shard=%d", int(extID), int(shard))

			// ---- NEW: publish notification for WebSocket ----
			payload, _ := json.Marshal(event)
			rdb.Publish(context.Background(), fmt.Sprintf("user:%d", int(extID)), payload)
			// -----------------------------------------------

		case "DRIVER_ADDED":
			extID, _ := event["ext_id"].(float64)
			seqNo, _ := event["seq_no"].(float64)
			shard, _ := event["shard_id"].(float64)
			lat, _ := event["lat"].(float64)
			lon, _ := event["lon"].(float64)
			storeDriverMapping(rdb, int(extID), int(shard), int(seqNo), lat, lon)
			geoKey := fmt.Sprintf("drivers:active:shard%d", int(shard))
			rdb.GeoAdd(context.Background(), geoKey, &redis.GeoLocation{
				Name:      fmt.Sprintf("%d", int(extID)),
				Longitude: lon,
				Latitude:  lat,
			})
			log.Printf("DRIVER_ADDED: ext_id=%d shard=%d", int(extID), int(shard))

			// ---- NEW: publish notification for WebSocket ----
			payload, _ := json.Marshal(event)
			rdb.Publish(context.Background(), fmt.Sprintf("user:%d", int(extID)), payload)
			// -----------------------------------------------

		case "MATCHED":
			riderID, _ := event["ext_rider_id"].(float64)
			driverID, _ := event["ext_driver_id"].(float64)
			shard, _ := event["shard_id"].(float64)
			price, _ := event["price"].(float64)

			// Remove from geo sets
			riderGeoKey := fmt.Sprintf("riders:active:shard%d", int(shard))
			driverGeoKey := fmt.Sprintf("drivers:active:shard%d", int(shard))
			rdb.ZRem(context.Background(), riderGeoKey, fmt.Sprintf("%d", int(riderID)))
			rdb.ZRem(context.Background(), driverGeoKey, fmt.Sprintf("%d", int(driverID)))
			// Delete mapping keys (optional)
			rdb.Del(context.Background(), fmt.Sprintf("rider:%d", int(riderID)))
			rdb.Del(context.Background(), fmt.Sprintf("driver:%d", int(driverID)))

			// Publish notification
			payload, _ := json.Marshal(event)
			rdb.Publish(context.Background(), fmt.Sprintf("user:%d", int(riderID)), payload)
			rdb.Publish(context.Background(), fmt.Sprintf("user:%d", int(driverID)), payload)
			log.Printf("MATCHED: rider=%d driver=%d price=%.2f shard=%d", int(riderID), int(driverID), price, int(shard))

		case "RIDER_CANCELED":
			extID, _ := event["ext_id"].(float64)
			shard, _ := event["shard_id"].(float64)
			geoKey := fmt.Sprintf("riders:active:shard%d", int(shard))
			rdb.ZRem(context.Background(), geoKey, fmt.Sprintf("%d", int(extID)))
			rdb.Del(context.Background(), fmt.Sprintf("rider:%d", int(extID)))

			payload, _ := json.Marshal(event)
			rdb.Publish(context.Background(), fmt.Sprintf("user:%d", int(extID)), payload)

			log.Printf("RIDER_CANCELED: ext_id=%d shard=%d", int(extID), int(shard))

		case "DRIVER_CANCELED":
			extID, _ := event["ext_id"].(float64)
			shard, _ := event["shard_id"].(float64)
			geoKey := fmt.Sprintf("drivers:active:shard%d", int(shard))
			rdb.ZRem(context.Background(), geoKey, fmt.Sprintf("%d", int(extID)))
			rdb.Del(context.Background(), fmt.Sprintf("driver:%d", int(extID)))

			payload, _ := json.Marshal(event)
			rdb.Publish(context.Background(), fmt.Sprintf("user:%d", int(extID)), payload)

			log.Printf("DRIVER_CANCELED: ext_id=%d shard=%d", int(extID), int(shard))

		}
	}
}

// ------------------------- Main -------------------------
func main() {
	if len(os.Args) < 2 {
		log.Fatal("Usage: go run main.go <shard_id>")
	}
	shardID, err := strconv.Atoi(os.Args[1])
	if err != nil {
		log.Fatalf("Invalid shard_id: %v", err)
	}
	log.Printf("Starting shard worker for shard %d", shardID)

	// Connect to Redis
	rdb := redis.NewClient(&redis.Options{Addr: redisAddr})
	if err := rdb.Ping(context.Background()).Err(); err != nil {
		log.Fatalf("Redis connection failed: %v", err)
	}

	// Init Kafka producer
	if err := initKafkaProducer(); err != nil {
		log.Fatalf("Failed to create Kafka producer: %v", err)
	}
	defer kafkaProducer.Close()

	// Start event consumer (in background)
	go startEventConsumer(rdb, shardID)

	// HTTP server for gateway requests
	port := 8080 + shardID
	router := gin.Default()
	router.POST("/rider/add", handleAddRider(rdb, shardID))
	router.POST("/driver/add", handleAddDriver(rdb, shardID))
	router.POST("/driver/interest", handleDriverInterest(rdb, shardID))
	router.POST("/instant-match", handleInstantMatch(rdb, shardID))
	router.POST("/cancel/rider", handleCancelRider(rdb, shardID))
	router.POST("/cancel/driver", handleCancelDriver(rdb, shardID))

	srv := &http.Server{
		Addr:    fmt.Sprintf(":%d", port),
		Handler: router,
	}
	go func() {
		log.Printf("Shard worker %d listening on port %d", shardID, port)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("HTTP server failed: %v", err)
		}
	}()

	// Graceful shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit
	log.Println("Shutting down shard worker...")
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	srv.Shutdown(ctx)
	kafkaProducer.Flush(1000)
}
