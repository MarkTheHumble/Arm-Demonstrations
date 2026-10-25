#!/usr/bin/env python3
"""
Find handwritten X and O marks and report an (x, y) for each one.
Works on an OAK camera (DepthAI v3) or on an image file.

How it works
------------
1. LOCATE  Classic image processing: threshold the ink, join broken strokes, and take
           every separate blob of a plausible size as a candidate mark.  This is where
           the x, y comes from (centre of the blob's box).
2. CLASSIFY  Each candidate is cropped, scaled and centred like an MNIST digit (20 px in
           a 28x28 frame) and a small CNN says "x" or "o".  The CNN is trained on the
           X and O letters from EMNIST - the MNIST-style dataset that includes letters.

Why not plain MNIST: MNIST only contains the digits 0-9.  It has no X at all (and its
"0" is only a rough stand-in for an O), so it cannot teach this.  EMNIST letters uses the
same 28x28 format and has both.

First run: if WEIGHTS_PATH does not exist, `main()` downloads EMNIST (roughly 0.5 GB, once)
and trains the CNN, which takes a few minutes on a CPU.  You can also call `train()` yourself.

Usage (edit SETTINGS first):
    import xo_detect
    xo_detect.main()                  # camera loop, or one image if IMAGE_PATH is set
    marks = xo_detect.detect(frame)   # one BGR numpy frame -> list of Mark

Coordinates are pixels in the frame: origin top-left, x to the right, y down.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import threading
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from colman_motion.arm_control import ArmControl
from colman_motion.scene_manager import SceneManager
from colman_motion.vacuum_control import VacuumControl

PRE_CAMERA_POSITION = {
    "shoulder_pan_joint": 0.5 * (np.pi / 180),
    "shoulder_lift_joint": -120.5 * (np.pi / 180),
    "elbow_joint": 120.5 * (np.pi / 180),
    "wrist_1_joint": -90.5 * (np.pi / 180),
    "wrist_2_joint": -90.5 * (np.pi / 180),
    "wrist_3_joint": 0.5 * (np.pi / 180),
}

CAMERA_POSITION = {
    "shoulder_pan_joint": 0 * (np.pi / 180),
    "shoulder_lift_joint": -120 * (np.pi / 180),
    "elbow_joint": 120 * (np.pi / 180),
    "wrist_1_joint": -90 * (np.pi / 180),
    "wrist_2_joint": -90 * (np.pi / 180),
    "wrist_3_joint": 0 * (np.pi / 180),
}

board_positions = [
    [(0.2248202570602908, 0.13978172353256214), (0.2248202570602908, 0.08428172353256214), (0.2248202570602908, 0.03478172353256214)],
    [(0.2783202570602908, 0.13978172353256214), (0.2783202570602908, 0.08428172353256214), (0.2783202570602908, 0.03478172353256214)],
    [(0.3318202570602908, 0.13978172353256214), (0.3318202570602908, 0.08428172353256214), (0.3318202570602908, 0.03478172353256214)]
]

def draw_x(row, col):

    global arm, vacuum, EE_DOWN, PRE_CAMERA_POSITION, CAMERA_POSITION, veryFast, verySlow, board_positions

    # first half
    x = board_positions[row][col][0] + 0.011
    y = board_positions[row][col][1] + 0.011
    z = 0.08507029111737157

    coordinates = [
        x, y, z
    ]
    arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)
    
    coordinates[0] -= 0.022
    coordinates[1] -= 0.022
    arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)


    # raise the marker to avoid smearing
    coordinates[2] += 0.03
    arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)

    # second half
    x = board_positions[row][col][0] + 0.011
    y = board_positions[row][col][1] - 0.011
    z = 0.08507029111737157

    coordinates = [
        x, y, z
    ]
    arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)
    
    coordinates[0] -= 0.022
    coordinates[1] += 0.022
    arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)



    # reset arm position
    arm.go_to_joint_pose(PRE_CAMERA_POSITION, veryFast)
    arm.go_to_joint_pose(CAMERA_POSITION, verySlow)

def draw_circle(row, col):

    global arm, vacuum, EE_DOWN, PRE_CAMERA_POSITION, CAMERA_POSITION, veryFast, verySlow, board_positions

    radius = 0.015
    for i in range(9):
        # calculate the angle for each point in radians
        # 9 iterations instead of 8 so it reaches the start a second time
        angle = (2 * math.pi * i) / 8
        
        # calculate x and y coordinates
        x = board_positions[row][col][0] + radius * math.cos(angle)
        y = board_positions[row][col][1] + radius * math.sin(angle)
        z = 0.08507029111737157

        coordinates = [
            x, y, z
        ]
        arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)

    # reset arm position
    arm.go_to_joint_pose(PRE_CAMERA_POSITION, veryFast)
    arm.go_to_joint_pose(CAMERA_POSITION, verySlow)


# claude generated vision model training
# =========================================================================== #
# SETTINGS - edit these by hand.  Nothing is read from the command line.
# =========================================================================== #

IMAGE_PATH = None          # None = read the OAK camera.  Or e.g. "page.png" to process one
                           # image (writes "page_xo.png" next to it).
JSON_PATH = None           # e.g. "marks.jsonl" appends every result as one JSON line; None = off

# Camera (only used when IMAGE_PATH is None)
CAMERA_SIZE = (1000, 1000) # (width, height); check your sensor's maximum
CAMERA_FPS = 10
PRINT_EVERY_S = 1.0        # how often the camera loop prints results to the console

# Model files / training
WEIGHTS_PATH = "xo_cnn.pt"  # created by train(); loaded by detect()
DATA_DIR = "emnist_data"    # where torchvision stores the EMNIST download
TRAIN_EPOCHS = 8

# Deciding what counts as a mark
MIN_CONF = 0.90            # ignore marks the CNN is less sure about (0 - 1)
INK_IS_DARK = True         # True = dark pen on light paper, False = light ink on dark
THRESH_BLOCK = 51          # adaptive-threshold window in px (odd; bigger than the pen stroke)
THRESH_C = 12              # how much darker than its surroundings a pixel must be to count as ink
JOIN_PX = 5                # close gaps up to about this many px so a two-stroke X is one mark
MIN_SIZE = 80              # px; a mark's longer side must be at least this ...
MAX_SIZE = 300             # ... and at most this
MAX_ASPECT = 2.5           # reject long thin blobs (ruled lines, edges)
TARGET_STROKE = 0.11       # pen thickness as a fraction of the mark size after scaling; thin
                           # pen strokes are thickened to this so they look like EMNIST's.
                           # 0 turns it off.
REMOVE_LINES_PX = 0        # 0 = off.  For lined paper or a drawn grid, set this to a little more
                           # than your biggest mark (e.g. 150): straight horizontal/vertical ink
                           # at least this long is erased before looking for marks.  Without it,
                           # a mark that touches a line merges with it and is rejected as too big.

# =========================================================================== #

CLASSES = ("x", "o")
_EMNIST_O, _EMNIST_X = 15, 24      # label numbers in EMNIST "letters" (a=1 ... z=26)


# --------------------------------------------------------------------------- #
# Model
# --------------------------------------------------------------------------- #


class XONet(nn.Module):
    def __init__(self):
        super().__init__()
        self.c1 = nn.Conv2d(1, 16, 3)
        self.c2 = nn.Conv2d(16, 32, 3)
        self.f1 = nn.Linear(32 * 5 * 5, 64)
        self.f2 = nn.Linear(64, 2)

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.c1(x)), 2)      # 28 -> 26 -> 13
        x = F.max_pool2d(F.relu(self.c2(x)), 2)      # 13 -> 11 -> 5
        x = F.dropout(F.relu(self.f1(x.flatten(1))), 0.3, self.training)
        return self.f2(x)


def load_emnist_xo():
    """EMNIST letters, only X and O.  Returns uint8 images (N,28,28) white-on-black and
    labels 0 = x, 1 = o, for train and test: (xs, ys, xt, yt)."""
    from torchvision.datasets import EMNIST     # already installed with ultralytics
    out = []
    for is_train in (True, False):
        try:
            ds = EMNIST(DATA_DIR, split="letters", train=is_train, download=True)
        except Exception as e:                  # pragma: no cover - needs network
            raise RuntimeError(
                f"Could not download EMNIST into {DATA_DIR!r}: {e}\n"
                "Download gzip.zip from https://www.nist.gov/itl/products-and-services/"
                "emnist-dataset and place it where torchvision expects it, or retry.") from e
        t = ds.targets
        keep = (t == _EMNIST_X) | (t == _EMNIST_O)
        imgs = ds.data[keep].transpose(1, 2).contiguous().numpy()   # EMNIST is stored transposed
        out += [imgs, (t[keep] == _EMNIST_O).long().numpy()]
    return tuple(out)


def augment(x: torch.Tensor) -> torch.Tensor:
    """Random rotation / scale / shift and pen thickness, on a (B,1,28,28) batch in 0..1."""
    b = x.shape[0]
    ang = (torch.rand(b) * 2 - 1) * math.radians(20)
    sc = 0.8 + 0.4 * torch.rand(b)
    t = (torch.rand(b, 2) * 2 - 1) * 0.12
    cos, sin = torch.cos(ang) / sc, torch.sin(ang) / sc
    theta = torch.stack([torch.stack([cos, -sin, t[:, 0]], 1),
                         torch.stack([sin, cos, t[:, 1]], 1)], 1)
    x = F.grid_sample(x, F.affine_grid(theta, list(x.shape), align_corners=False),
                      padding_mode="zeros", align_corners=False)
    r = torch.rand(b, 1, 1, 1)
    thick = F.max_pool2d(x, 3, 1, 1)
    thin = -F.max_pool2d(-x, 3, 1, 1)
    return torch.where(r < 0.35, thick, torch.where(r > 0.8, thin, x))


def train(epochs: int | None = None) -> None:
    epochs = TRAIN_EPOCHS if epochs is None else epochs
    torch.manual_seed(0)
    xs, ys, xt, yt = load_emnist_xo()
    save_preview(xs, ys)
    xs_t = torch.from_numpy(xs).float().div(255).unsqueeze(1)
    ys_t = torch.from_numpy(ys)
    xt_t = torch.from_numpy(xt).float().div(255).unsqueeze(1)
    yt_t = torch.from_numpy(yt)
    model = XONet()
    opt = torch.optim.Adam(model.parameters(), 1e-3)
    for epoch in range(epochs):
        model.train()
        perm = torch.randperm(len(xs_t))
        for i in range(0, len(perm), 128):
            idx = perm[i:i + 128]
            loss = F.cross_entropy(model(augment(xs_t[idx])), ys_t[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            acc = (model(xt_t).argmax(1) == yt_t).float().mean().item()
        print(f"epoch {epoch + 1}/{epochs}  last loss {loss.item():.3f}  test accuracy {acc * 100:.1f}%")
    torch.save({"state": model.state_dict(), "classes": CLASSES}, WEIGHTS_PATH)
    print(f"saved {WEIGHTS_PATH}")


def save_preview(xs: np.ndarray, ys: np.ndarray) -> None:
    """Writes xo_training_preview.png: top row X, bottom row O.  Eyeball that they are
    upright and look like letters (this catches a mis-oriented dataset)."""
    rows = [np.hstack(list(xs[ys == k][:12])) for k in (0, 1)]
    cv2.imwrite("xo_training_preview.png", cv2.resize(np.vstack(rows), None, fx=3, fy=3,
                                                      interpolation=cv2.INTER_NEAREST))


_MODEL = None


def get_model() -> XONet:
    global _MODEL
    if _MODEL is None:
        if not Path(WEIGHTS_PATH).exists():
            raise FileNotFoundError(f"{WEIGHTS_PATH} not found - call train() first (or main()).")
        m = XONet()
        m.load_state_dict(torch.load(WEIGHTS_PATH, map_location="cpu")["state"])
        _MODEL = m.eval()
    return _MODEL


# --------------------------------------------------------------------------- #
# Locate + classify
# --------------------------------------------------------------------------- #


@dataclass
class Mark:
    label: str                       # "x" or "o"
    x: float                         # centre of the mark's box, pixels
    y: float
    box: tuple[int, int, int, int]   # x0, y0, x1, y1
    conf: float                      # 0..1


def find_candidates(bgr: np.ndarray):
    """Returns [(box, mask)] where mask is the mark's own ink (255) cropped to its box."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if not INK_IS_DARK:
        gray = 255 - gray
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    ink = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY_INV, THRESH_BLOCK | 1, THRESH_C)
    if REMOVE_LINES_PX:
        # Hough finds straight lines at any tilt (a camera is never perfectly square to the page)
        segs = cv2.HoughLinesP(ink, 1, np.pi / 180, threshold=REMOVE_LINES_PX // 2,
                               minLineLength=REMOVE_LINES_PX, maxLineGap=8)
        if segs is not None:
            erase = np.zeros_like(ink)
            for x1, y1, x2, y2 in segs[:, 0]:
                cv2.line(erase, (int(x1), int(y1)), (int(x2), int(y2)), 255, 5)
            ink = ink & ~erase
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (JOIN_PX | 1, JOIN_PX | 1))
    joined = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, k)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(joined, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, _area = (int(v) for v in stats[i])
        long_, short_ = max(w, h), min(w, h)
        if long_ < MIN_SIZE or long_ > MAX_SIZE or long_ > MAX_ASPECT * short_:
            continue
        sl = (slice(y, y + h), slice(x, x + w))
        mask = (((labels[sl] == i) & (ink[sl] > 0)) * 255).astype(np.uint8)
        out.append(((x, y, x + w, y + h), mask))
    return out


def to_mnist(mask: np.ndarray) -> np.ndarray:
    """Tight 255-on-0 crop -> 28x28 the way MNIST/EMNIST frame a character:
    longest side 20 px, centred by centre of mass.  Thin strokes are thickened first."""
    h, w = mask.shape
    if TARGET_STROKE > 0:
        cnts, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        perimeter = sum(len(c) for c in cnts)
        if perimeter:
            stroke = 2.0 * float(np.count_nonzero(mask)) / perimeter      # ~pen width in px
            grow = TARGET_STROKE * max(h, w) - stroke
            if grow >= 1:
                k = 2 * int(round(grow / 2)) + 1
                mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    s = 20.0 / max(h, w)
    m = cv2.resize(mask, (max(1, round(w * s)), max(1, round(h * s))), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((28, 28), np.uint8)
    y0, x0 = (28 - m.shape[0]) // 2, (28 - m.shape[1]) // 2
    canvas[y0:y0 + m.shape[0], x0:x0 + m.shape[1]] = m
    mo = cv2.moments(canvas)
    if mo["m00"] > 0:
        shift = np.float32([[1, 0, 13.5 - mo["m10"] / mo["m00"]], [0, 1, 13.5 - mo["m01"] / mo["m00"]]])
        canvas = cv2.warpAffine(canvas, shift, (28, 28))
    return canvas


def detect(bgr: np.ndarray) -> list[Mark]:
    cands = find_candidates(bgr)
    if not cands:
        return []
    batch = torch.from_numpy(np.stack([to_mnist(m) for _, m in cands])).float().div(255).unsqueeze(1)
    with torch.no_grad():
        probs = F.softmax(get_model()(batch), dim=1)
    conf, cls = probs.max(1)
    marks = []
    for ((x0, y0, x1, y1), _), c, k in zip(cands, conf.tolist(), cls.tolist()):
        if c >= MIN_CONF:
            marks.append(Mark(CLASSES[k], (x0 + x1) / 2.0, (y0 + y1) / 2.0, (x0, y0, x1, y1), c))
    marks.sort(key=lambda m: (m.y, m.x))
    return marks


# --------------------------------------------------------------------------- #
# Output helpers + entry points
# --------------------------------------------------------------------------- #


def to_json(marks: list[Mark], w: int, h: int) -> dict:
    return {"time": time.time(), "frame_size": [w, h],
            "marks": [{"label": m.label, "x": round(m.x, 1), "y": round(m.y, 1),
                       "box": list(m.box), "conf": round(m.conf, 3)} for m in marks]}


def print_marks(marks: list[Mark]) -> None:
    print(f"--- {len(marks)} mark(s) ---")
    for m in marks:
        print(f"  {m.label.upper()}  ({m.x:.0f}, {m.y:.0f})  conf {m.conf:.2f}")


def draw(img: np.ndarray, marks: list[Mark]) -> np.ndarray:
    out = img.copy()
    for m in marks:
        colour = (0, 0, 255) if m.label == "x" else (255, 0, 0)
        x0, y0, x1, y1 = m.box
        cv2.rectangle(out, (x0, y0), (x1, y1), colour, 2)
        cv2.putText(out, f"{m.label.upper()} {m.conf:.2f}", (x0, max(12, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    thickness = 1    
    cv2.line(out, (384, 129), (384, 878), (0, 255, 0), thickness)
    cv2.line(out, (617, 880), (617, 129), (0, 255, 0), thickness)
    cv2.line(out, (132, 385), (870, 385), (0, 255, 0), thickness)
    cv2.line(out, (132, 623), (868, 623), (0, 255, 0), thickness)
    return out


def run_image() -> None:
    path = Path(IMAGE_PATH)
    img = cv2.imread(str(path))
    if img is None:
        raise SystemExit(f"could not read {path}")
    marks = detect(img)
    print_marks(marks)
    out_png = path.with_name(path.stem + "_xo.png")
    cv2.imwrite(str(out_png), draw(img, marks))
    print(f"annotated image -> {out_png}")
    if JSON_PATH:
        with open(JSON_PATH, "a") as f:
            f.write(json.dumps(to_json(marks, img.shape[1], img.shape[0])) + "\n")


def run_camera() -> None:
    import depthai as dai   # imported here so image mode works without a camera

    w, h = CAMERA_SIZE
    json_file = open(JSON_PATH, "a") if JSON_PATH else None
    last_print = 0.0
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        out = cam.requestOutput((w, h), fps=CAMERA_FPS)
        q = out.createOutputQueue(maxSize=1, blocking=False)
        pipeline.start()
        while pipeline.isRunning():
            msg = q.tryGet()
            if msg is None:
                if cv2.waitKey(5) == ord("q"):
                    break
                continue
            frame = msg.getCvFrame()
            marks = detect(frame)
            now = time.monotonic()
            if now - last_print >= PRINT_EVERY_S:
                last_print = now
                print_marks(marks)
            if json_file:
                json_file.write(json.dumps(to_json(marks, frame.shape[1], frame.shape[0])) + "\n")
                json_file.flush()
            cv2.imshow("X / O detector", draw(frame, marks))
            if cv2.waitKey(1) == ord("q"):
                break
        pipeline.stop()
    cv2.destroyAllWindows()
    if json_file:
        json_file.close()

# arm controls
APPROACH_OFFSET = 0.1
EE_OFFSET = 0.01
EE_DOWN = (1.0, 0.0, 0.0, 0.0)
rclpy.init()
ur = MoveItPy(node_name="moveit_py")
arm = ArmControl(ur)
scene = SceneManager(ur)
executor = MultiThreadedExecutor()
executor.add_node(arm)
executor.add_node(scene)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

# Setup collision scene and planners
scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))
veryFast = PlanRequestParameters(ur, "ompl")
veryFast.max_velocity_scaling_factor = 0.99
veryFast.max_acceleration_scaling_factor = 0.99
fast = PlanRequestParameters(ur, "ompl")
fast.max_velocity_scaling_factor = 0.8
fast.max_acceleration_scaling_factor = 0.8
slow = PlanRequestParameters(ur, "ompl")
slow.max_velocity_scaling_factor = 0.2
slow.max_acceleration_scaling_factor = 0.2
verySlow = PlanRequestParameters(ur, "ompl")
verySlow.max_velocity_scaling_factor = 0.01
verySlow.max_acceleration_scaling_factor = 0.01

# get in the position
arm.go_to_joint_pose(PRE_CAMERA_POSITION, veryFast)
arm.go_to_joint_pose(CAMERA_POSITION, verySlow)

def main() -> None:
    """Trains first if there are no weights yet, then runs image mode or the camera loop."""
    if not Path(WEIGHTS_PATH).exists():
        print(f"{WEIGHTS_PATH} not found - downloading EMNIST and training (one time only)...")
        train()
    run_image() if IMAGE_PATH else run_camera()


if __name__ == "__main__":
    main()
