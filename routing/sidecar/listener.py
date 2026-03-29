#!/usr/bin/env python3
"""
Ontario 511 API Data Fetcher
Fetches data from three key endpoints:
- /event (traffic events, closures, incidents)
- /roadconditions (weather/road surface conditions)
- /constructionprojects (planned construction)
Respects API rate limiting (10 calls per 60 seconds).
"""

import requests
import json
import time
from datetime import datetime
from pathlib import Path

# Configuration
API_BASE = "https://511on.ca/api/v2/get"
OUTPUT_DIR = Path("511_data")
CALLS_PER_MINUTE = 10
DELAY_BETWEEN_CALLS = 60 / CALLS_PER_MINUTE  # 6 seconds between calls

# Create output directory
OUTPUT_DIR.mkdir(exist_ok=True)

# The three endpoints you requested
ENDPOINTS = [
    "event",
    "roadconditions",
    "constructionprojects"
]

def fetch_endpoint(endpoint_name):
    """Fetch data from a single endpoint and return JSON."""
    url = f"{API_BASE}/{endpoint_name}"
    params = {"format": "json"}
    
    print(f"🌐 Fetching {endpoint_name}...")
    
    try:
        response = requests.get(url, params=params, timeout=30)
        response.raise_for_status()
        data = response.json()
        
        # Count items if it's a list
        item_count = len(data) if isinstance(data, list) else "N/A"
        print(f"✅ Got {item_count} items")
        return data
        
    except requests.exceptions.RequestException as e:
        print(f"❌ Error fetching {endpoint_name}: {e}")
        return None
    except json.JSONDecodeError as e:
        print(f"❌ Invalid JSON from {endpoint_name}: {e}")
        return None

def save_data(endpoint_name, data):
    """Save data to timestamped and latest JSON files."""
    if data is None:
        return
    
    # Save with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = OUTPUT_DIR / f"{endpoint_name}_{timestamp}.json"
    
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"💾 Saved to {filename}")
    
    # Also save as latest (overwrites previous)
    latest_file = OUTPUT_DIR / f"{endpoint_name}_latest.json"
    with open(latest_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"💾 Updated {latest_file}")

def analyze_endpoint(endpoint_name, data):
    """Print basic stats about the fetched data."""
    if not data:
        return
    
    print(f"\n📊 {endpoint_name.upper()} Analysis:")
    
    if isinstance(data, list):
        print(f"   Total items: {len(data)}")
        
        if len(data) > 0:
            # Show sample fields from first item
            sample = data[0]
            fields = list(sample.keys())
            print(f"   Fields: {', '.join(fields[:8])}{'...' if len(fields) > 8 else ''}")
            
            # Check for useful location fields
            has_linkid = 'LinkId' in sample
            has_coords = 'Latitude' in sample and 'Longitude' in sample
            has_roadway = 'RoadwayName' in sample
            
            if has_linkid:
                with_linkid = sum(1 for e in data if e.get('LinkId'))
                print(f"   Items with LinkId: {with_linkid}/{len(data)}")
            
            if has_coords:
                with_coords = sum(1 for e in data if e.get('Latitude') and e.get('Longitude'))
                print(f"   Items with coordinates: {with_coords}/{len(data)}")
                
            if has_roadway:
                with_roadway = sum(1 for e in data if e.get('RoadwayName'))
                print(f"   Items with roadway name: {with_roadway}/{len(data)}")
    else:
        print(f"   Data type: {type(data)}")
        if isinstance(data, dict):
            print(f"   Keys: {list(data.keys())}")

def main():
    print("🚗 Ontario 511 API Fetcher (3 Endpoints)")
    print("=" * 50)
    print(f"Endpoints: {', '.join(ENDPOINTS)}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Rate limit: {CALLS_PER_MINUTE} calls/minute")
    print("=" * 50)
    
    for i, endpoint in enumerate(ENDPOINTS):
        print(f"\n[{i+1}/{len(ENDPOINTS)}] Processing: {endpoint}")
        
        data = fetch_endpoint(endpoint)
        save_data(endpoint, data)
        analyze_endpoint(endpoint, data)
        
        # Rate limiting - don't delay after last request
        if i < len(ENDPOINTS) - 1:
            wait_time = DELAY_BETWEEN_CALLS
            print(f"⏱️  Waiting {wait_time:.1f} seconds (rate limit)...")
            time.sleep(wait_time)
    
    print("\n✅ All done! Check the 511_data directory for JSON files.")
    print("\n📁 Files saved:")
    for endpoint in ENDPOINTS:
        latest = OUTPUT_DIR / f"{endpoint}_latest.json"
        if latest.exists():
            print(f"   - {latest}")

if __name__ == "__main__":
    main()