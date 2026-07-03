# map4_smartpole_detection_filter

> **TL;DR** An edge-side ROS 2 node that drops perception objects lying outside the areas you care about (or inside "no-go" areas), and any object with a broken (non-finite) position — a geometric, map-based cleanup step in the LiDAR pipeline.

## Overview

The edge pipeline turns raw LiDAR points into tracked objects:

```
LiDAR → sanitizer → CenterPoint (3D detection) → multi-object tracker → [ this filter ] → downstream
```

A roadside pole "sees" everything in range, but you usually only care about a
specific slice of the world — the road and its shoulders, say — and definitely
**not** things happening inside a neighbouring building, a parking lot, or a
sensor blind-spot that produces phantom objects. Feeding those unwanted objects
downstream wastes bandwidth and pollutes the merged view built by the server.

This package solves that with a purely **geometric, region-based filter**. You
draw polygons on the map (in the `map` frame) marking areas to keep
("whitelist") and areas to reject ("blacklist"), and the node passes through
only the objects whose ground position falls in the right place. It does **not**
look at object class, confidence, or size — the decision is based solely on
*where the object is*.

> Despite the name "detection filter", the running node consumes the **tracker's**
> output, not raw CenterPoint detections — see [Notes](#notes).

## Provides

- **Node** `area_filter` (class `AreaFilter`) — the only executable in the package.
  Entry point: `area_filter = map4_smartpole_detection_filter.area_filter:main`.

There is no launch file inside this package; it is wired into the edge pipeline
by `map4_smartpole_launch` (see [Usage](#usage)).

## How it works

The node loads polygon definitions from a directory (the `config_path`
parameter), then filters every incoming object message. Each object is checked
by its 2D ground position (`x`, `y`) against the polygons, applying these rules
**in order**:

1. **Non-finite guard** — if the object's `x` or `y` is `NaN`/`Inf`, it is
   dropped. (Marked `FSR-1.1` in the code — a functional safety requirement so
   corrupt geometry never reaches downstream consumers.)
2. **Blacklist** — if the object's position is inside *any* `*.black.txt`
   polygon, it is dropped.
3. **Whitelist** — if at least one `*.white.txt` polygon is configured, the
   object is kept **only if** its position is inside one of them. If *no*
   whitelist polygons exist, every object that survived the blacklist passes
   through.

### Topics

All names below are the node's internal names; the edge launch remaps them (see [Usage](#usage)).

| Direction | Topic (internal) | Type | Notes |
|-----------|------------------|------|-------|
| Subscribe | `input/objects`  | `autoware_perception_msgs/TrackedObjects` | Objects to filter |
| Publish   | `output/objects` | `autoware_perception_msgs/TrackedObjects` | The kept subset |
| Publish   | `output/markers` | `visualization_msgs/MarkerArray` | Polygon outlines + labels for RViz (latched / `TRANSIENT_LOCAL`) |
| Publish   | `/diagnostics`   | `diagnostic_msgs/DiagnosticStatus` | Health (see below) |

### Polygon file format

The `config_path` directory is scanned for two file globs:

- `*.white.txt` → keep-areas (whitelist)
- `*.black.txt` → reject-areas (blacklist)

Each file is one polygon, one vertex per line, as comma-separated `x,y,z`
coordinates in the `map` frame (the `z` value is used only to place the RViz
text label; the containment test is 2D):

```
89523.1,42711.4,3.2
89540.7,42711.9,3.2
89541.2,42690.3,3.2
89522.6,42689.8,3.2
```

You can have many of each; every file becomes one polygon and gets a distinct
colour in the marker overlay.

### Diagnostics

The node runs a `diagnostic_updater` task (`area_filter`) that reports:
`ERROR` when `config_path` is missing (the filter is effectively inactive), when
no input has ever arrived, or when input has stalled (>5 s); `WARN` when
non-finite positions have been seen; otherwise `OK`. It also publishes running
counters (objects in / out / filtered / non-finite) as key-value pairs.

## Build

```bash
bash build.sh --packages-select map4_smartpole_detection_filter
```

## Usage

The node is launched as part of the edge pipeline by
`map4_smartpole_launch/launch/map4_smartpole_base.launch.xml`. It is **opt-in**
and controlled by an environment variable:

- `MAP4_SMARTPOLE_TRACKING_USE_AREA_FILTER=true` → the `area_filter` node runs,
  reading `tracked/raw/objects` and publishing `tracked/objects`.
- Default (`false`) → a `topic_tools relay` simply forwards
  `tracked/raw/objects` → `tracked/objects` unchanged (filter bypassed).

In the launch file the topics are remapped like so:

| Internal name    | Edge topic          |
|------------------|---------------------|
| `input/objects`  | `tracked/raw/objects` |
| `output/objects` | `tracked/objects`     |
| `output/markers` | `area_filter/markers` |

Run it standalone for testing:

```bash
ros2 run map4_smartpole_detection_filter area_filter \
  --ros-args -p config_path:=/path/to/polygons \
  -r input/objects:=/perception/lidar/tracked/raw/objects \
  -r output/objects:=/perception/lidar/tracked/objects
```

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `""` | Directory holding `*.white.txt` / `*.black.txt` polygon files. **If empty or missing, the node subscribes to nothing and filters nothing** (reported as `ERROR` in diagnostics). |
| `qos/sub` | `reliable` | Input QoS; set to `best_effort` for lossy/high-rate links. |
| `qos/pub` | `reliable` | Output QoS; `best_effort` supported. |
| `diagnostics.period_sec` | `1.0` | How often the diagnostics task publishes. |
| `diagnostics.hardware_id` | `detection_filter` | Hardware ID reported in diagnostics. |

Related environment variable (read by `map4_smartpole_launch`, not by this node):

| Env var | Default | Effect |
|---------|---------|--------|
| `MAP4_SMARTPOLE_TRACKING_USE_AREA_FILTER` | `false` | `true` runs this filter; otherwise a plain relay is used. |

> **Heads-up:** the base launch currently sets `config_path` to an empty string,
> so even when the filter is enabled it needs a real path supplied before it does
> anything. With no `config_path` the node is intentionally inert and flags itself
> as `ERROR` rather than silently passing everything.

## Notes

- **Package name vs. git repo name.** This package directory is
  `map4_smartpole_detection_filter`, but its upstream git repository is
  `map4_smartpole_detection_filterer` (note the extra `er`), as referenced in
  `anvil.repos.yaml`. The ROS package name is the shorter form.
- **It filters tracked objects, not raw detections.** The name suggests it sits
  between detection and tracking, but the code and launch wiring place it
  **after** the multi-object tracker (`tracked/raw/objects` → `tracked/objects`),
  and it works on `TrackedObjects`.
- **Area-only, no class filter (yet).** `schema/area_filter.schema.json`
  describes a richer JSON config with per-area `classes` lists, but the
  implemented loader ignores classes entirely — it only reads plain
  `x,y,z` vertex text files and filters by 2D position. Class-based filtering is
  not currently implemented.
- **Whitelist semantics.** With *no* whitelist files present, the filter is
  "blacklist-only" — everything not explicitly rejected passes. Add at least one
  whitelist polygon to switch to "reject everything except these areas".
