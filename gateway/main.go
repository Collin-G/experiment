package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sort"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/redis/go-redis/v9"
	"github.com/uber/h3-go/v4"
)

// ------------------------- Configuration -------------------------
const (
	h3Resolution    = 6
	numShards       = 3
	shardWorkerPort = 8080 // base port: worker 0 -> 8080, worker 1 -> 8081, ...
	redisAddr       = "localhost:6379"
)

// ------------------------- Request types -------------------------
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

// ------------------------- WebSocket manager -------------------------
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

var (
	wsClients   = make(map[int]*websocket.Conn)
	wsClientsMu sync.RWMutex
)

func registerWS(extID int, conn *websocket.Conn) {
	wsClientsMu.Lock()
	defer wsClientsMu.Unlock()
	wsClients[extID] = conn
	log.Printf("WebSocket registered for user %d", extID)
}

func unregisterWS(extID int) {
	wsClientsMu.Lock()
	defer wsClientsMu.Unlock()
	delete(wsClients, extID)
	log.Printf("WebSocket unregistered for user %d", extID)
}

func sendToUser(extID int, message []byte) {
	wsClientsMu.RLock()
	conn, ok := wsClients[extID]
	wsClientsMu.RUnlock()
	if !ok {
		log.Printf("No WebSocket for user %d", extID)
		return
	}
	log.Printf("Sending to user %d: %s", extID, message)
	if err := conn.WriteMessage(websocket.TextMessage, message); err != nil {
		log.Printf("Write error for user %d: %v", extID, err)
		unregisterWS(extID)
	}
}

// ------------------------- Redis Pub/Sub forwarder (using PSubscribe) -------------------------
func startRedisPubSubForwarder(rdb *redis.Client) {
	log.Println("Starting Redis Pub/Sub forwarder (PSubscribing to user:*)")
	pubsub := rdb.PSubscribe(context.Background(), "user:*")
	defer pubsub.Close()

	// Wait for subscription to be active
	if _, err := pubsub.Receive(context.Background()); err != nil {
		log.Printf("Subscription receive error: %v", err)
		return
	}

	ch := pubsub.Channel()
	for msg := range ch {
		log.Printf("Pub/Sub received on %s: %s", msg.Channel, msg.Payload)
		// channel format: "user:1234"
		parts := strings.Split(msg.Channel, ":")
		if len(parts) != 2 {
			log.Printf("Invalid channel format: %s", msg.Channel)
			continue
		}
		extID, err := strconv.Atoi(parts[1])
		if err != nil {
			log.Printf("Invalid user ID: %v", err)
			continue
		}
		sendToUser(extID, []byte(msg.Payload))
	}
}

// ------------------------- Helpers -------------------------
func computeShardID(lat, lon float64) int {
	cell, err := h3.LatLngToCell(h3.LatLng{Lat: lat, Lng: lon}, h3Resolution)
	if err != nil {
		log.Printf("H3 error: %v, fallback to shard 0", err)
		return 0
	}
	return int(cell) % numShards
}

func lookupShard(rdb *redis.Client, key string) (int, error) {
	shardStr, err := rdb.HGet(context.Background(), key, "shard").Result()
	if err != nil {
		return -1, err
	}
	return strconv.Atoi(shardStr)
}

func forwardToShardWorker(shardID int, path string, body []byte) (*http.Response, error) {
	url := fmt.Sprintf("http://localhost:%d%s", shardWorkerPort+shardID, path)
	log.Printf("Forwarding to %s", url)
	return http.Post(url, "application/json", bytes.NewBuffer(body))
}

// ------------------------- HTTP handlers -------------------------
func setupRouter(rdb *redis.Client) *gin.Engine {
	r := gin.Default()

	// WebSocket upgrade
	r.GET("/ws", func(c *gin.Context) {
		userIDStr := c.Query("user_id")
		extID, err := strconv.Atoi(userIDStr)
		if err != nil {
			c.String(http.StatusBadRequest, "invalid user_id")
			return
		}
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			log.Printf("WebSocket upgrade error for user %d: %v", extID, err)
			return
		}
		registerWS(extID, conn)
		defer unregisterWS(extID)

		// Keep connection alive until client closes
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				break
			}
		}
	})

	// Add rider (new participant)
	r.POST("/rider/add", func(c *gin.Context) {
		var req AddRiderReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		shardID := computeShardID(req.Lat, req.Lon)
		body, _ := json.Marshal(req)
		resp, err := forwardToShardWorker(shardID, "/rider/add", body)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "shard worker unreachable"})
			return
		}
		defer resp.Body.Close()
		c.Status(resp.StatusCode)
	})

	// Add driver
	r.POST("/driver/add", func(c *gin.Context) {
		var req AddDriverReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		shardID := computeShardID(req.Lat, req.Lon)
		body, _ := json.Marshal(req)
		resp, err := forwardToShardWorker(shardID, "/driver/add", body)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "shard worker unreachable"})
			return
		}
		defer resp.Body.Close()
		c.Status(resp.StatusCode)
	})

	// Driver interest (needs rider's shard)
	r.POST("/driver/interest", func(c *gin.Context) {
		var req DriverInterestReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		shardID, err := lookupShard(rdb, fmt.Sprintf("rider:%d", req.ExtRiderID))
		if err != nil {
			c.JSON(http.StatusNotFound, gin.H{"error": "rider not found"})
			return
		}
		body, _ := json.Marshal(req)
		resp, err := forwardToShardWorker(shardID, "/driver/interest", body)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "shard worker unreachable"})
			return
		}
		defer resp.Body.Close()
		c.Status(resp.StatusCode)
	})

	// Instant match (needs rider's shard)
	r.POST("/instant-match", func(c *gin.Context) {
		var req InstantMatchReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		shardID, err := lookupShard(rdb, fmt.Sprintf("rider:%d", req.ExtRiderID))
		if err != nil {
			c.JSON(http.StatusNotFound, gin.H{"error": "rider not found"})
			return
		}
		body, _ := json.Marshal(req)
		resp, err := forwardToShardWorker(shardID, "/instant-match", body)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "shard worker unreachable"})
			return
		}
		defer resp.Body.Close()
		c.Status(resp.StatusCode)
	})

	// Cancel rider
	r.POST("/cancel/rider", func(c *gin.Context) {
		var req CancelReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		shardID, err := lookupShard(rdb, fmt.Sprintf("rider:%d", req.ExtID))
		if err != nil {
			c.JSON(http.StatusNotFound, gin.H{"error": "rider not found"})
			return
		}
		body, _ := json.Marshal(req)
		resp, err := forwardToShardWorker(shardID, "/cancel/rider", body)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "shard worker unreachable"})
			return
		}
		defer resp.Body.Close()
		c.Status(resp.StatusCode)
	})

	// Cancel driver
	r.POST("/cancel/driver", func(c *gin.Context) {
		var req CancelReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		shardID, err := lookupShard(rdb, fmt.Sprintf("driver:%d", req.ExtID))
		if err != nil {
			c.JSON(http.StatusNotFound, gin.H{"error": "driver not found"})
			return
		}
		body, _ := json.Marshal(req)
		resp, err := forwardToShardWorker(shardID, "/cancel/driver", body)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "shard worker unreachable"})
			return
		}
		defer resp.Body.Close()
		c.Status(resp.StatusCode)
	})

	// GET /nearby-riders?lat=...&lon=...&radius=...&sort=distance|bid
	r.GET("/nearby-riders", func(c *gin.Context) {
		lat, err := strconv.ParseFloat(c.Query("lat"), 64)
		if err != nil {
			c.JSON(400, gin.H{"error": "invalid lat"})
			return
		}
		lon, err := strconv.ParseFloat(c.Query("lon"), 64)
		if err != nil {
			c.JSON(400, gin.H{"error": "invalid lon"})
			return
		}
		radius, err := strconv.ParseFloat(c.Query("radius"), 64)
		if err != nil {
			radius = 5.0
		}
		sortBy := c.DefaultQuery("sort", "distance")

		shardID := computeShardID(lat, lon)
		key := fmt.Sprintf("riders:active:shard%d", shardID)

		// Query Redis for riders within radius
		locations, err := rdb.GeoRadius(context.Background(), key, lon, lat, &redis.GeoRadiusQuery{
			Radius:    radius,
			Unit:      "km",
			WithDist:  true,
			WithCoord: false,
			Sort:      "ASC",
		}).Result()
		if err != nil {
			c.JSON(500, gin.H{"error": err.Error()})
			return
		}

		type RiderInfo struct {
			ExtID    int     `json:"ext_id"`
			Distance float64 `json:"distance_km"`
			Bid      float64 `json:"bid"`
			Mode     string  `json:"mode"`
			SeqNo    int     `json:"seq_no"`
		}
		var riders []RiderInfo
		for _, loc := range locations {
			extID, _ := strconv.Atoi(loc.Name)
			// Fetch rider details from Redis hash
			riderKey := fmt.Sprintf("rider:%d", extID)
			bidStr, _ := rdb.HGet(context.Background(), riderKey, "bid").Result()
			modeStr, _ := rdb.HGet(context.Background(), riderKey, "mode").Result()
			seqNoStr, _ := rdb.HGet(context.Background(), riderKey, "seq_no").Result()
			bid, _ := strconv.ParseFloat(bidStr, 64)
			seqNo, _ := strconv.Atoi(seqNoStr)

			riders = append(riders, RiderInfo{
				ExtID:    extID,
				Distance: loc.Dist,
				Bid:      bid,
				Mode:     modeStr,
				SeqNo:    seqNo,
			})
		}

		// Sort by bid (descending) or distance (ascending)
		if sortBy == "bid" {
			sort.Slice(riders, func(i, j int) bool { return riders[i].Bid > riders[j].Bid })
		} else {
			// already sorted by distance from GeoRadius
		}
		c.JSON(200, riders)
	})

	r.StaticFile("/driver", "driver_dashboard.html")

	return r
}

// ------------------------- Main -------------------------
func main() {
	rdb := redis.NewClient(&redis.Options{Addr: redisAddr})
	if err := rdb.Ping(context.Background()).Err(); err != nil {
		log.Fatalf("Redis connection failed: %v", err)
	}

	go startRedisPubSubForwarder(rdb)

	router := setupRouter(rdb)
	srv := &http.Server{
		Addr:    ":8000",
		Handler: router,
	}

	go func() {
		log.Println("Gateway listening on port 8000")
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("listen: %s", err)
		}
	}()

	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit
	log.Println("Shutting down gateway...")
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	srv.Shutdown(ctx)
}
