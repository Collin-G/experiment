import ctypes
import os
import threading

# ============================================================
# Configuration
# ============================================================

# Absolute path to the shared library
LIB_PATH = "/home/collin/programming/flux/routing/build/librouting.so"

# Absolute path to the OSM PBF file
OSM_PATH = "/home/collin/programming/flux/routing/files/Toronto.osm.pbf"

# ============================================================
# Load shared library
# ============================================================

if not os.path.isfile(LIB_PATH):
    raise FileNotFoundError(f"Shared library not found: {LIB_PATH}")

lib = ctypes.CDLL(LIB_PATH)

# ============================================================
# Declare C function signatures
# ============================================================

# Initialize the routing engine
lib.init_router.argtypes = [ctypes.c_char_p]
lib.init_router.restype = ctypes.c_bool

# Compute shortest path distance
lib.route_distance.argtypes = [
    ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double
]
lib.route_distance.restype = ctypes.c_double

# Update edge weight
lib.update_edge_by_coordinates.argtypes = [
    ctypes.c_double, ctypes.c_double, ctypes.c_double
]
lib.update_edge_by_coordinates.restype = None

# By id
lib.update_edge_by_id.argtypes = [
    ctypes.c_int, ctypes.c_double
]
lib.update_edge_by_id.restype = None


lib.update_edge_by_nodes.argtypes = [
    ctypes.c_int,  ctypes.c_int, ctypes.c_double
]
lib.update_edge_by_nodes.restype = None

# # Update edge weight
# lib.update_edge.argtypes = [
#     ctypes.c_double, ctypes.c_double, ctypes.c_double
# ]
# lib.update_edge.restype = None

# ============================================================
# One-time initialization guard (Python-side)
# ============================================================

_init_lock = threading.Lock()
_initialized = False

def init():
    """
    Initialize the routing engine. Safe to call multiple times.
    """
    global _initialized

    if _initialized:
        return

    with _init_lock:
        if _initialized:
            return

        ok = lib.init_router(OSM_PATH.encode("utf-8"))
        if not ok:
            raise RuntimeError("Failed to initialize routing engine")

        _initialized = True

# ============================================================
# Public API
# ============================================================

def route_distance(lat1, lon1, lat2, lon2):
    """
    Compute shortest-path distance (meters) between two lat/lon points.
    """
    if not _initialized:
        raise RuntimeError("Router not initialized. Call init() first.")

    dist = lib.route_distance(
        float(lat1), float(lon1),
        float(lat2), float(lon2)
    )

    return dist


def update_edge_by_coordinates(lat, lon, new_weight):
    """
    Update the weight of the edge closest to the given coordinates.
    """
    if not _initialized:
        raise RuntimeError("Router not initialized. Call init() first.")

    lib.update_edge_by_coordinates(
        float(lat), float(lon), float(new_weight)
    )


def update_edge_by_id(id, new_weight):
    """
    Update the weight of the edge closest to the given coordinates.
    """
    if not _initialized:
        raise RuntimeError("Router not initialized. Call init() first.")

    lib.update_edge_by_id(
        int(id), float(new_weight)
    )

def update_edge_by_nodes(frm, to, weight):
    if not _initialized:
        raise RuntimeError("Router not initialized. Call init() first.")

    lib.update_edge_by_nodes(
        int(frm), int(to),float(weight)
    )

# ============================================================
# Example usage (direct test)
# ============================================================

if __name__ == "__main__":
    init()

    # Example coordinates (Toronto City Hall → CN Tower)
    start_lat, start_lon = 43.69, -79.32
    end_lat, end_lon = 43.6845, -79.339

    # 1. Distance before updating edges
    dist_before = route_distance(start_lat, start_lon, end_lat, end_lon)
    print(f"Distance before update: {dist_before:.2f} meters")

    # 2. Simulate traffic / blockage by increasing edge weight
    accident_lat, accident_lon = (43.692+ 43.6896)/2, (-79.322 -79.3221)/2
    print(f"Updating edge near ({accident_lat}, {accident_lon})...")
    update_edge_by_coordinates(accident_lat, accident_lon,100)  # huge weight to simulate blockage

    # 3. Distance after updating edges
    dist_after = route_distance(start_lat, start_lon, end_lat, end_lon)
    print(f"Distance after update: {dist_after:.2f} meters")

    # 4. Optional: update back to normal
    print("Resetting edge weight...")
    update_edge_by_coordinates(accident_lat, accident_lon,1.0) 

    dist_reset = route_distance(start_lat, start_lon, end_lat, end_lon)
    print(f"Distance after reset: {dist_reset:.2f} meters")
