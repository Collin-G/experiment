// main.go
package main

import (
    "fmt"
    "github.com/confluentinc/confluent-kafka-go/kafka"
)

func main() {
    c, err := kafka.NewConsumer(&kafka.ConfigMap{
        "bootstrap.servers": "localhost:9092",
        "group.id":          "shard-0-go-worker",
        "auto.offset.reset": "earliest",
    })
    if err != nil {
        panic(err)
    }
    defer c.Close()

    c.SubscribeTopics([]string{"shard-0-events"}, nil)

    fmt.Println("shard worker started, listening for events...")

    for {
        msg, err := c.ReadMessage(-1)
        if err != nil {
            fmt.Printf("consumer error: %v\n", err)
            continue
        }
        fmt.Printf("received: %s\n", string(msg.Value))
    }
}