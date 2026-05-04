import asyncio
import json
import random
import argparse
import requests
import websockets

GATEWAY_URL = "http://localhost:8000"
WS_URL = "ws://localhost:8000/ws"

async def add_rider_via_http(ext_id, lat, lon, mode, bid):
    url = f"{GATEWAY_URL}/rider/add"
    payload = {"ext_id": ext_id, "lat": lat, "lon": lon, "mode": mode, "bid": bid}
    resp = requests.post(url, json=payload)
    if resp.status_code != 202:
        raise Exception(f"Add rider failed: {resp.text}")

async def add_driver_via_http(ext_id, lat, lon):
    url = f"{GATEWAY_URL}/driver/add"
    payload = {"ext_id": ext_id, "lat": lat, "lon": lon}
    resp = requests.post(url, json=payload)
    if resp.status_code != 202:
        raise Exception(f"Add driver failed: {resp.text}")

async def instant_match_via_http(rider_id, rsn, driver_id, dsn):
    url = f"{GATEWAY_URL}/instant-match"
    payload = {"ext_rider_id": rider_id, "rsn": rsn, "ext_driver_id": driver_id, "dsn": dsn}
    resp = requests.post(url, json=payload)
    if resp.status_code != 202:
        raise Exception(f"Instant match failed: {resp.text}")

async def wait_for_event(websocket, expected_type, timeout=10):
    try:
        msg = await asyncio.wait_for(websocket.recv(), timeout=timeout)
        data = json.loads(msg)
        if data.get("type") == expected_type:
            return data
        else:
            raise Exception(f"Unexpected event: {data}")
    except asyncio.TimeoutError:
        raise Exception(f"Timeout waiting for {expected_type}")

def get_coords_for_shard(target_shard):
    # Precomputed coordinates for each shard (adjust according to your H3 config)
    coords_map = {
        0: (40.7128, -74.0060),
        1: (34.0522, -118.2437),
        2: (41.8781, -87.6298),
        3: (29.7604, -95.3698),
        4: (33.4484, -112.0740),
        5: (39.9526, -75.1652),
        6: (29.4241, -98.4936),
        7: (32.7157, -117.1611),
        8: (32.7767, -96.7970),
        9: (37.7749, -122.4194),
    }
    return coords_map.get(target_shard, (40.7128, -74.0060))

async def stress_test(num_pairs=3, force_shard=None):
    print("\n=== STRESS TEST (instant mode, WebSocket) ===")
    if force_shard is not None:
        print(f"Forcing all operations to shard {force_shard}")
    for i in range(num_pairs):
        if force_shard is not None:
            lat, lon = get_coords_for_shard(force_shard)
        else:
            lat = random.uniform(-90, 90)
            lon = random.uniform(-180, 180)
        rider_id = random.randint(100000, 199999)
        driver_id = random.randint(200000, 299999)
        print(f"\nPair {i+1}: rider {rider_id}, driver {driver_id} (coords {lat:.4f},{lon:.4f})")

        # Open WebSocket for rider and driver in parallel
        async with (
            websockets.connect(f"{WS_URL}?user_id={rider_id}") as rider_ws,
            websockets.connect(f"{WS_URL}?user_id={driver_id}") as driver_ws
        ):
            # Add rider (instant mode)
            await add_rider_via_http(rider_id, lat, lon, "instant", random.uniform(10,100))
            rider_event = await wait_for_event(rider_ws, "RIDER_ADDED", timeout=10)
            rsn = rider_event.get("seq_no")
            print(f"  Added rider {rider_id} with seq_no {rsn}")

            # Add driver
            await add_driver_via_http(driver_id, lat+0.0001, lon+0.0001)
            driver_event = await wait_for_event(driver_ws, "DRIVER_ADDED", timeout=10)
            dsn = driver_event.get("seq_no")
            print(f"  Added driver {driver_id} with seq_no {dsn}")

            # Instant match with correct seq_nos
            await instant_match_via_http(rider_id, rsn, driver_id, dsn)
            # Wait for MATCHED event on rider's WebSocket
            match_event = await wait_for_event(rider_ws, "MATCHED", timeout=10)
            if match_event.get("ext_rider_id") == rider_id and match_event.get("ext_driver_id") == driver_id:
                print(f"  ✅ Match confirmed: price {match_event.get('price')}")
            else:
                raise Exception("Match event does not match IDs")
        await asyncio.sleep(0.2)  # slight delay between pairs
    print("Stress test finished.")

async def auction_test(force_shard=None):
    print("\n=== AUCTION TEST (auction mode, WebSocket) ===")
    if force_shard is not None:
        lat, lon = get_coords_for_shard(force_shard)
    else:
        lat, lon = 40.7128, -74.0060
    rider_id = random.randint(90000, 99999)
    driver_id = random.randint(90000, 99999)
    ask = 45.0

    async with (
        websockets.connect(f"{WS_URL}?user_id={rider_id}") as rider_ws,
        websockets.connect(f"{WS_URL}?user_id={driver_id}") as driver_ws
    ):
        # Add rider (auction mode)
        await add_rider_via_http(rider_id, lat, lon, "auction", 100.0)
        rider_event = await wait_for_event(rider_ws, "RIDER_ADDED", timeout=10)
        rsn = rider_event.get("seq_no")
        print(f"Added rider {rider_id} with seq_no {rsn}")

        # Add driver
        await add_driver_via_http(driver_id, lat+0.0001, lon+0.0001)
        driver_event = await wait_for_event(driver_ws, "DRIVER_ADDED", timeout=10)
        dsn = driver_event.get("seq_no")
        print(f"Added driver {driver_id} with seq_no {dsn}")

        # Driver interest
        url = f"{GATEWAY_URL}/driver/interest"
        payload = {"ext_driver_id": driver_id, "dsn": dsn, "ext_rider_id": rider_id, "rsn": rsn, "ask": ask}
        resp = requests.post(url, json=payload)
        if resp.status_code != 202:
            raise Exception(f"Interest failed: {resp.text}")
        print(f"Driver {driver_id} interested in rider {rider_id} (ask={ask})")

        print("Waiting 30 seconds for periodic auction...")
        match_event = await wait_for_event(rider_ws, "MATCHED", timeout=35)
        if match_event.get("ext_rider_id") == rider_id and match_event.get("ext_driver_id") == driver_id:
            print(f"✅ Auction match succeeded: price {match_event.get('price')}")
        else:
            raise Exception("Match event does not match IDs")
    print("Auction test completed.")

async def cancel_test(force_shard=0):
    print("\n=== CANCEL TEST ===")
    lat, lon = get_coords_for_shard(force_shard)
    rider_id = random.randint(70000, 79999)
    driver_id = random.randint(80000, 89999)

    async with (
        websockets.connect(f"{WS_URL}?user_id={rider_id}") as rider_ws,
        websockets.connect(f"{WS_URL}?user_id={driver_id}") as driver_ws
    ):
        # Add rider
        await add_rider_via_http(rider_id, lat, lon, "auction", 100.0)
        rider_event = await wait_for_event(rider_ws, "RIDER_ADDED", timeout=10)
        rsn = rider_event.get("seq_no")
        print(f"Added rider {rider_id} with seq_no {rsn}")

        # Cancel rider
        url = f"{GATEWAY_URL}/cancel/rider"
        payload = {"ext_id": rider_id, "sn": rsn}
        resp = requests.post(url, json=payload)
        if resp.status_code != 202:
            raise Exception(f"Cancel rider failed: {resp.text}")
        cancel_event = await wait_for_event(rider_ws, "RIDER_CANCELED", timeout=5)
        print(f"  ✅ RIDER_CANCELED received: {cancel_event}")

        # Add driver
        await add_driver_via_http(driver_id, lat+0.0001, lon+0.0001)
        driver_event = await wait_for_event(driver_ws, "DRIVER_ADDED", timeout=10)
        dsn = driver_event.get("seq_no")
        print(f"Added driver {driver_id} with seq_no {dsn}")

        # Cancel driver
        url = f"{GATEWAY_URL}/cancel/driver"
        payload = {"ext_id": driver_id, "sn": dsn}
        resp = requests.post(url, json=payload)
        if resp.status_code != 202:
            raise Exception(f"Cancel driver failed: {resp.text}")
        cancel_event = await wait_for_event(driver_ws, "DRIVER_CANCELED", timeout=5)
        print(f"  ✅ DRIVER_CANCELED received: {cancel_event}")

    print("Cancel test finished.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stress", action="store_true")
    parser.add_argument("--auction", action="store_true")
    parser.add_argument("--cancel", action="store_true")
    parser.add_argument("--shard", type=int, default=None, help="Force all operations to a specific shard (0-9)")
    args = parser.parse_args()

    if args.stress:
        asyncio.run(stress_test(force_shard=args.shard))
    if args.auction:
        asyncio.run(auction_test(force_shard=args.shard))
    if args.cancel:
        asyncio.run(cancel_test(force_shard=args.shard if args.shard is not None else 0))
    if not (args.stress or args.auction or args.cancel):
        parser.print_help()

if __name__ == "__main__":
    main()