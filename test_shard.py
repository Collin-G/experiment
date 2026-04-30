from kafka import KafkaProducer
import json

producer = KafkaProducer(
    bootstrap_servers='localhost:9092',
    value_serializer=lambda v: json.dumps(v).encode('utf-8')
)

# Rider
producer.send('commands', partition=0, value={
    "type": "ADD_RIDER",
    "payload": {
        "ext_id": 999,
        "bid": 99.9,
        "lat": 40.0,
        "lon": -75.0,
        "mode": "auction"
    }
})

# Driver (close enough to rider for match)
producer.send('commands', partition=0, value={
    "type": "ADD_DRIVER",
    "payload": {
        "ext_id": 2001,
        "lat": 40.001,
        "lon": -75.001
    }
})

# Driver interest (makes match happen immediately in your engine? No – interest just records, match occurs every 30 sec)
producer.send('commands', partition=0, value={
    "type": "DRIVER_INTEREST",
    "payload": {
        "ext_driver_id": 2001,
        "dsn": 0,
        "ext_rider_id": 999,
        "rsn": 0,
        "ask": 80.0
    }
})

producer.flush()
print("Rider, Driver, and Interest sent to partition 0!")