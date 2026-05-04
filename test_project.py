import time
import subprocess
import redis
import requests
import argparse
import random

GATEWAY_URL = "http://localhost:8000"
SHARD_BASE_PORT = 8080
REDIS_HOST = "localhost"
REDIS_PORT = 6379
NUM_SHARDS = 3

def get_seq_no(key, timeout=5):
    r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)
    start = time.time()
    # Short initial delay to let engine process
    time.sleep(0.3)
    while time.time() - start < timeout:
        seq = r.hget(key, "seq_no")
        if seq is not None:
            # If it's 0 but not the placeholder? We can't differentiate.
            # So accept 0 only after the initial delay.
            return int(seq)
        time.sleep(0.1)
    raise TimeoutError(f"Could not get seq_no for {key}")

def add_participant(shard, typ, ext_id, lat, lon, mode=None, bid=None):
    url = f"http://localhost:{SHARD_BASE_PORT + shard}/{typ}/add"
    payload = {"ext_id": ext_id, "lat": lat, "lon": lon}
    if typ == "rider":
        if mode is None:
            mode = "auction"
        payload["mode"] = mode
        payload["bid"] = bid or 100.0
    resp = requests.post(url, json=payload)
    if resp.status_code != 202:
        raise Exception(f"Add {typ} failed: {resp.text}")
    seq = get_seq_no(f"{typ}:{ext_id}")
    print(f"Added {typ} {ext_id} (shard {shard}) with seq_no {seq}")
    return seq

def driver_interest(shard, driver_id, dsn, rider_id, rsn, ask):
    url = f"http://localhost:{SHARD_BASE_PORT + shard}/driver/interest"
    payload = {"ext_driver_id": driver_id, "dsn": dsn,
               "ext_rider_id": rider_id, "rsn": rsn, "ask": ask}
    resp = requests.post(url, json=payload)
    if resp.status_code != 202:
        raise Exception(f"Interest failed: {resp.text}")
    print(f"Driver {driver_id} interested in rider {rider_id} (ask={ask})")

def instant_match(shard, rider_id, rsn, driver_id, dsn):
    url = f"http://localhost:{SHARD_BASE_PORT + shard}/instant-match"
    payload = {"ext_rider_id": rider_id, "rsn": rsn,
               "ext_driver_id": driver_id, "dsn": dsn}
    resp = requests.post(url, json=payload)
    if resp.status_code != 202:
        raise Exception(f"Instant match failed: {resp.text}")
    print(f"Instant match: rider {rider_id} <-> driver {driver_id}")

def random_coords():
    return random.uniform(-90, 90), random.uniform(-180, 180)

def unique_id(prefix, base=None):
    if base is None:
        base = int(time.time() * 1000) % 1000000
    return prefix * 1000000 + base + random.randint(0, 999)

def stress_test(num_pairs=5):
    print("\n=== STRESS TEST (instant match, riders in 'instant' mode) ===")
    for _ in range(num_pairs):
        shard = random.randint(0, NUM_SHARDS - 1)
        # Generate unique IDs based on current time (ms)
        base = int(time.time() * 1000) % 1000000
        rider_id = unique_id(1, base)
        driver_id = unique_id(2, base)
        lat, lon = random_coords()
        try:
            rsn = add_participant(shard, "rider", rider_id, lat, lon, mode="instant", bid=random.uniform(10,100))
            dsn = add_participant(shard, "driver", driver_id, lat+0.0001, lon+0.0001)
            instant_match(shard, rider_id, rsn, driver_id, dsn)
            time.sleep(0.2)
        except Exception as e:
            print(f"Error processing pair: {e}")
    print("Stress test finished.")

def auction_test(shard=0):
    print("\n=== AUCTION MATCH TEST (waits 30 seconds, riders in 'auction' mode) ===")
    # Use unique IDs for each run
    base = int(time.time() * 1000) % 1000000
    rider_id = unique_id(1, base)
    driver_id = unique_id(2, base)
    lat, lon = 40.7128, -74.0060
    ask = 45.0

    rsn = add_participant(shard, "rider", rider_id, lat, lon, mode="auction", bid=100.0)
    dsn = add_participant(shard, "driver", driver_id, lat+0.0001, lon+0.0001)
    driver_interest(shard, driver_id, dsn, rider_id, rsn, ask)

    print("Waiting 30 seconds for periodic auction...")
    time.sleep(30)

    r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)
    if not r.exists(f"rider:{rider_id}"):
        print(f"✅ Auction match succeeded: rider {rider_id} removed from Redis (matched).")
    else:
        print(f"❌ Auction match failed: rider {rider_id} still in Redis.")
        seq = r.hget(f"rider:{rider_id}", "seq_no")
        print(f"Rider seq_no is {seq}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stress", action="store_true")
    parser.add_argument("--auction", action="store_true")
    parser.add_argument("--shard", type=int, default=0)
    args = parser.parse_args()
    if args.stress:
        stress_test()
    if args.auction:
        auction_test(shard=args.shard)
    if not (args.stress or args.auction):
        parser.print_help()

if __name__ == "__main__":
    main()