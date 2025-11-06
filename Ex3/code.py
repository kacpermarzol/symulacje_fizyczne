# Kacper Marzol

import vpython
import numpy as np
import time
import math

BOX_COUNT = 2000
WORLD_SIZE = 50.0
MAX_SPEED = 0.1
MIN_SIZE, MAX_SIZE = 1.0, 2.0

vpython.scene.width = 1000
vpython.scene.height = 750
vpython.scene.background = vpython.color.gray(0.15)
vpython.scene.title = "<b>Collision Comparison</b> — 1=Brute  2=SAP  3=BVH"

positions = np.random.uniform(-WORLD_SIZE/2, WORLD_SIZE/2, (BOX_COUNT, 3)).astype(np.float32)
velocities = np.random.uniform(-MAX_SPEED, MAX_SPEED, (BOX_COUNT, 3)).astype(np.float32)
sizes = np.random.uniform(MIN_SIZE, MAX_SIZE, (BOX_COUNT, 3)).astype(np.float32)  # non-uniform allowed

aabb_mins = np.empty_like(positions)
aabb_maxs = np.empty_like(positions)

boxes = []
for i in range(BOX_COUNT):
    p = positions[i]
    s = sizes[i]
    b = vpython.box(pos=vpython.vector(*p), size=vpython.vector(*s), color=vpython.color.green)
    boxes.append(b)

def update_aabbs():
    half = sizes * 0.5
    np.subtract(positions, half, out=aabb_mins)
    np.add(positions, half, out=aabb_maxs)

def aabb_intersect_raw(i, j):
    return (aabb_mins[i, 0] <= aabb_maxs[j, 0] and aabb_maxs[i, 0] >= aabb_mins[j, 0] and
            aabb_mins[i, 1] <= aabb_maxs[j, 1] and aabb_maxs[i, 1] >= aabb_mins[j, 1] and
            aabb_mins[i, 2] <= aabb_maxs[j, 2] and aabb_maxs[i, 2] >= aabb_mins[j, 2])

# Brute Force
def brute_force():
    pairs = []
    checks = 0
    for i in range(BOX_COUNT):
        for j in range(i+1, BOX_COUNT):
            checks += 1
            if aabb_intersect_raw(i, j):
                pairs.append((i, j))
    return pairs, checks

# Sweep & Prune
def sweep_and_prune():
    idx = np.argsort(aabb_mins[:, 0], kind='quicksort')
    pairs = []
    checks = 0
    for ii in range(BOX_COUNT):
        i = int(idx[ii])
        max_i_x = aabb_maxs[i, 0]
        for jj in range(ii+1, BOX_COUNT):
            j = int(idx[jj])
            if aabb_mins[j, 0] > max_i_x:
                break
            checks += 1
            if aabb_intersect_raw(i, j):
                pairs.append((i, j))
    return pairs, checks

# BVH
class BVHNode:
    def __init__(self):
        self.min = np.zeros(3)
        self.max = np.zeros(3)
        self.left = None
        self.right = None
        self.leaf_index = -1

def expand_bits(v):
    v = (v * 0x00010001) & 0xFF0000FF
    v = (v * 0x00000101) & 0x0F00F00F
    v = (v * 0x00000011) & 0xC30C30C3
    v = (v * 0x00000005) & 0x49249249
    return v

def morton3D(x, y, z):
    def norm(val):
        return (val + WORLD_SIZE / 2) / WORLD_SIZE

    x = max(0.0, min(1.0, norm(x)))
    y = max(0.0, min(1.0, norm(y)))
    z = max(0.0, min(1.0, norm(z)))

    x = int(min(1023, max(0, math.floor(x * 1023))))
    y = int(min(1023, max(0, math.floor(y * 1023))))
    z = int(min(1023, max(0, math.floor(z * 1023))))

    xx = expand_bits(x)
    yy = expand_bits(y)
    zz = expand_bits(z)

    return xx | (yy << 1) | (zz << 2)

def build_bvh_recursive(indices):
    if len(indices) == 0:
        return None

    morton_list = []
    for i in indices:
        center = positions[i]
        morton_code = morton3D(center[0], center[1], center[2])
        morton_list.append((i, morton_code))

    morton_list.sort(key=lambda x: x[1])
    sorted_indices = [x[0] for x in morton_list]

    def build_subtree(begin, end):
        node = BVHNode()
        if begin == end:
            i = sorted_indices[begin]
            node.leaf_index = i
            node.min[:] = aabb_mins[i]
            node.max[:] = aabb_maxs[i]
            return node

        mid = (begin + end) // 2
        node.left = build_subtree(begin, mid)
        node.right = build_subtree(mid + 1, end)

        node.min[:] = np.minimum(node.left.min, node.right.min)
        node.max[:] = np.maximum(node.left.max, node.right.max)
        return node
    return build_subtree(0, len(sorted_indices) - 1)

def query_bvh(node, idx, pairs_set, checks_ref):
    checks_ref[0] += 1
    if (aabb_mins[idx, 0] > node.max[0] or aabb_maxs[idx, 0] < node.min[0] or
        aabb_mins[idx, 1] > node.max[1] or aabb_maxs[idx, 1] < node.min[1] or
        aabb_mins[idx, 2] > node.max[2] or aabb_maxs[idx, 2] < node.min[2]):
        return

    if node.leaf_index >= 0:
        j = node.leaf_index
        if j != idx and aabb_intersect_raw(idx, j):
            pairs_set.add(tuple(sorted((idx, j))))
        return

    if node.left:
        query_bvh(node.left, idx, pairs_set, checks_ref)
    if node.right:
        query_bvh(node.right, idx, pairs_set, checks_ref)


def bvh_collisions(root):
    pairs_set = set()
    checks_ref = [0]
    for i in range(BOX_COUNT):
        query_bvh(root, i, pairs_set, checks_ref)
    return list(pairs_set), checks_ref[0]


# ______________________________________________



# Controls
method = 3             # 1=Brute,2=SAP,3=BVH
method_name = "BVH"
bvh_root = None

def key_input(evt):
    global method, method_name
    key = evt.key
    if key == '1':
        method, method_name = 1, "Brute Force"
    elif key == '2':
        method, method_name = 2, "Sweep & Prune"
    elif key == '3':
        method, method_name = 3, "BVH"
vpython.scene.bind('keydown', key_input)

status = vpython.label(pos=vpython.vector(0, WORLD_SIZE/2 + 4, 0), box=False, height=12, color=vpython.color.white)

frame_count = 0
last_time = time.perf_counter()
fps = 0.0
update_aabbs()
bvh_root = build_bvh_recursive(list(range(BOX_COUNT)))

while True:
    vpython.rate(60)
    frame_count += 1

    positions += velocities
    half_sizes = sizes * 0.5
    limits = WORLD_SIZE/2 - half_sizes

    for axis in range(3):
        over = positions[:, axis] > limits[:, axis]
        if np.any(over):
            positions[over, axis] = limits[over, axis]
            velocities[over, axis] *= -1
        under = positions[:, axis] < -limits[:, axis]
        if np.any(under):
            positions[under, axis] = -limits[under, axis]
            velocities[under, axis] *= -1

    for i, b in enumerate(boxes):
        b.pos = vpython.vector(*positions[i])
        b.size = vpython.vector(*sizes[i])

    update_aabbs()

    start = time.perf_counter()

    if method == 1:  # brute
        t0 = time.perf_counter()
        pairs, checks = brute_force()
        t1 = time.perf_counter()
        query_ms = (t1 - t0) * 1000.0

    elif method == 2:  # SAP
        t0 = time.perf_counter()
        pairs, checks = sweep_and_prune()
        t1 = time.perf_counter()
        query_ms = (t1 - t0) * 1000.0
    else:
        bvh_root = build_bvh_recursive(list(range(BOX_COUNT)))
        t_query_start = time.perf_counter()
        pairs, checks = bvh_collisions(bvh_root)
        t_query_end = time.perf_counter()
        query_ms = (t_query_end - t_query_start) * 1000.0

    elapsed = (time.perf_counter() - start) * 1000.0
    now = time.perf_counter()

    if now - last_time >= 1.0:
        fps = frame_count / (now - last_time)
        frame_count = 0
        last_time = now

    status.text = (f"Method: {method_name}\n"
                   f"FPS: {fps:5.1f}     Total: {elapsed:6.2f} ms\n"
                   f"query: {query_ms:6.2f} ms     Pairs: {len(pairs):6d}\n"
                   f"   Checks: {checks:8d}     Boxes: {BOX_COUNT}\n")

    for b in boxes:
        b.color = vpython.color.green
    for (i, j) in pairs:
        if i < BOX_COUNT:
            boxes[i].color = vpython.color.red
        if j < BOX_COUNT:
            boxes[j].color = vpython.color.red
