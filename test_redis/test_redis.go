package main

import (
    "context"
    "log"

    "github.com/redis/go-redis/v9"
)

func main() {
    rdb := redis.NewClient(&redis.Options{Addr: "localhost:6379"})

    ctx := context.Background()
    pubsub := rdb.Subscribe(ctx, "user:*")
    defer pubsub.Close()

    // Wait for subscription to be active
    _, err := pubsub.Receive(ctx)
    if err != nil {
        log.Fatal("Subscribe failed:", err)
    }
    log.Println("Subscribed to user:*")

    ch := pubsub.Channel()
    for msg := range ch {
        log.Printf("GOT: channel=%s payload=%s", msg.Channel, msg.Payload)
    }
}