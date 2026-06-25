# FrameForge

A linter and diagnostician for robot frame trees, sensor conventions, and
transform consistency. It does **not** draw the TF tree (Foxglove and
`tf2_tools` already do that) — it answers *why the tree doesn't make physical or
mathematical sense, and which transform is most likely wrong.*

FrameForge is a single static Zig binary with **zero runtime dependencies** — no
ROS install required. It ingests TF dumps, CSV sensor logs, and a declarative
robot profile, and emits ranked, evidence-backed findings.

> General tool, real first target: the included
> [`profiles/barracuda.profile.yaml`](profiles/barracuda.profile.yaml) was
> verified against the Barracuda AUV's URDF, a captured TF dump, the estimator
> config, and git history. See the comments in that file for provenance.

## Build & run

Requires **Zig 0.16.0**.

```bash
zig build                 # produces zig-out/bin/frameforge
zig build test            # unit tests
zig build run -- help     # run via the build system
```

## Commands

```text
frameforge tf       <tree.gv>          Validate TF structure + profile agreement
frameforge gravity  <imu.csv>          Check gravity direction/magnitude (stationary IMU)
frameforge compare  <a.csv> <b.csv>    Median roll/pitch/yaw diff; names 90/180 deg bugs
frameforge validate --profile <f> [--tf F] [--imu F]   Run all available checks
frameforge profile  <file>             Load a profile and print a summary
```

Exit code is non-zero when any `FAIL` is emitted (suitable for CI).

### Try it on the bundled Barracuda data

```bash
zig build
P=profiles/barracuda.profile.yaml

# Real captured TF dump — reproduces the "base_link missing from the tree" finding
./zig-out/bin/frameforge tf examples/barracuda_tf.gv --profile $P

# Gravity from a stationary IMU log (good vs upside-down)
./zig-out/bin/frameforge gravity examples/imu_stationary_good.csv    --profile $P
./zig-out/bin/frameforge gravity examples/imu_stationary_flipped.csv --profile $P

# Two orientation streams that disagree by 180 deg in yaw
./zig-out/bin/frameforge compare examples/orientation_imu.csv examples/orientation_camera.csv
```

## The robot profile

Checks are not hard-coded to any robot. A robot declares its expected setup once
— world/base frames, conventions, gravity, sensor topic→frame map, expected
rates, the estimator contract, and which downstream nodes must consume the
estimator. Every check then validates measured data against that contract. Adding
a robot is "write a profile," not a code change.

## Architecture

```text
parsers (tfdump .gv · CSV)              src/parsers/
        |  normalize into
        v
frame model + ROBOT PROFILE             src/profile.zig
        |  each check = measured-vs-profile pass
        v
checks  ->  findings  ->  reporter      src/checks/, src/finding.zig
```

Each check is an independent pass that emits `Finding`s (id, severity, summary,
and ranked `Cause`s with how-to-confirm / how-to-fix). Adding a check never
touches a parser; adding a parser never touches a check.

## Status (M1)

Implemented: profile loader, `.gv` TF-dump and CSV parsers, and the structure,
gravity, and cross-sensor convention checks — validated end-to-end on real
Barracuda artifacts.

Planned (see [AGENTS.md](AGENTS.md)): rosbag2 `.db3`/`.mcap` + CDR ingest,
static-TF-vs-URDF diff, timestamp/rate analysis, consumer-contract scanning of
launch/param files, JSON output, and the GTSAM estimator contract checks.
