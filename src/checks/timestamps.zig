//! Timestamp/rate check: verify monotonic sensor stamps, gaps, and observed
//! rate against the robot profile's topic rate_hz contract.
const std = @import("std");
const Allocator = std.mem.Allocator;
const csv = @import("../parsers/csv.zig");
const Profile = @import("../profile.zig").Profile;
const Reporter = @import("../finding.zig").Reporter;

const Stats = struct {
    samples: usize,
    first: f64,
    last: f64,
    duration: f64,
    rate_hz: f64,
    min_dt: f64,
    max_dt: f64,
    mean_dt: f64,
    out_of_order: usize,
    gaps: usize,
};

const TimeCols = struct {
    sec: usize,
    nsec: ?usize = null,
};

pub fn run(gpa: Allocator, rep: *Reporter, tbl: csv.Table, profile: ?Profile, topic: ?[]const u8, time_column: ?[]const u8, fallback_type: ?[]const u8) !void {
    const time_cols = findTimeCols(tbl, time_column);

    if (time_cols == null) {
        try rep.emit(.{
            .id = "timestamps/no-time-column",
            .severity = .warn,
            .summary = "Could not find a timestamp column (timestamp/stamp/time/t/sec or header.stamp.sec/nanosec).",
            .detail = "Use --time <column-name> when the CSV uses a project-specific single timestamp header.",
        });
        return;
    }

    const expected = expectedRate(profile, topic, fallback_type);
    const stats = (try computeStats(tbl, time_cols.?, expected)) orelse {
        try rep.emit(.{
            .id = "timestamps/not-enough-samples",
            .severity = .warn,
            .summary = "Need at least two numeric timestamps to analyze rate/jitter.",
        });
        return;
    };

    const expected_txt = if (expected) |hz|
        try std.fmt.allocPrint(gpa, " expected {d:.2} Hz,", .{hz})
    else
        "";
    const detail = try std.fmt.allocPrint(
        gpa,
        "{d} samples over {d:.3}s,{s} observed {d:.2} Hz; dt min/mean/max = {d:.4}/{d:.4}/{d:.4}s.",
        .{ stats.samples, stats.duration, expected_txt, stats.rate_hz, stats.min_dt, stats.mean_dt, stats.max_dt },
    );

    if (stats.out_of_order > 0) {
        try rep.emit(.{
            .id = "timestamps/out-of-order",
            .severity = .fail,
            .summary = try std.fmt.allocPrint(gpa, "{d} timestamp step(s) go backwards.", .{stats.out_of_order}),
            .detail = detail,
            .causes = &.{
                .{ .text = "A driver is publishing stale messages after newer samples.", .confidence = "high", .confirm = "Sort the raw log by receive time and inspect the offending topic stamps.", .fix = "Drop stale samples or fix the driver's timestamp source." },
                .{ .text = "Simulation time and wall time are mixed in the same stream.", .confidence = "medium", .confirm = "Check use_sim_time and the timestamp units for this topic." },
            },
        });
    } else {
        try rep.emit(.{
            .id = "timestamps/monotonic",
            .severity = .pass,
            .summary = "Timestamps are monotonic.",
            .detail = detail,
        });
    }

    if (stats.gaps > 0) {
        try rep.emit(.{
            .id = "timestamps/gaps",
            .severity = .warn,
            .summary = try std.fmt.allocPrint(gpa, "{d} large timestamp gap(s) detected.", .{stats.gaps}),
            .detail = detail,
            .causes = &.{
                .{ .text = "Sensor or transport dropped samples under load.", .confidence = "medium", .confirm = "Compare with ROS topic statistics or driver logs for the same time window." },
                .{ .text = "The log combines separate capture segments.", .confidence = "low", .confirm = "Plot timestamp deltas and inspect capture boundaries." },
            },
        });
    } else {
        try rep.emit(.{
            .id = "timestamps/gaps",
            .severity = .pass,
            .summary = "No large timestamp gaps detected.",
            .detail = detail,
        });
    }

    if (expected) |hz| {
        const rel_err = @abs(stats.rate_hz - hz) / hz;
        if (rel_err > 0.20) {
            try rep.emit(.{
                .id = "timestamps/rate",
                .severity = .fail,
                .summary = try std.fmt.allocPrint(gpa, "Observed {d:.2} Hz, expected {d:.2} Hz from profile.", .{ stats.rate_hz, hz }),
                .detail = detail,
                .causes = &.{
                    .{ .text = "The topic is throttled, remapped to the wrong stream, or using the wrong driver config.", .confidence = "high", .confirm = "Run `ros2 topic hz` for the topic named in the profile." },
                    .{ .text = "Timestamp units are wrong (milliseconds or nanoseconds treated as seconds).", .confidence = "medium", .confirm = "Inspect the raw timestamp magnitude and adjacent deltas." },
                },
            });
        } else {
            try rep.emit(.{
                .id = "timestamps/rate",
                .severity = .pass,
                .summary = try std.fmt.allocPrint(gpa, "Observed {d:.2} Hz matches profile rate {d:.2} Hz.", .{ stats.rate_hz, hz }),
                .detail = detail,
            });
        }
    } else {
        const topic_txt = topic orelse "(no --topic supplied)";
        try rep.emit(.{
            .id = "timestamps/rate-profile",
            .severity = .info,
            .summary = try std.fmt.allocPrint(gpa, "No profile rate_hz found for topic {s}; skipped expected-rate comparison.", .{topic_txt}),
            .detail = detail,
        });
    }
}

fn expectedRate(profile: ?Profile, topic: ?[]const u8, fallback_type: ?[]const u8) ?f64 {
    const p = profile orelse return null;
    if (topic) |t| {
        if (p.sensorByTopic(t)) |s| return s.rate_hz;
        return null;
    }
    if (fallback_type) |kind| {
        if (p.firstSensorOfType(kind)) |s| return s.rate_hz;
    }
    return null;
}

fn findTimeCols(tbl: csv.Table, time_column: ?[]const u8) ?TimeCols {
    if (time_column) |name| {
        if (tbl.col(&.{name})) |idx| return .{ .sec = idx };
    }
    if (tbl.col(&.{ "header.stamp.sec", "stamp.sec", "time.sec" })) |sec| {
        if (tbl.col(&.{ "header.stamp.nanosec", "header.stamp.nsec", "stamp.nanosec", "stamp.nsec", "time.nanosec", "time.nsec" })) |nsec| {
            return .{ .sec = sec, .nsec = nsec };
        }
    }
    if (tbl.col(&.{ "timestamp", "header.stamp", "stamp", "time", "t", "sec" })) |idx| {
        return .{ .sec = idx };
    }
    return null;
}

fn rowTime(row: []const f64, cols: TimeCols) f64 {
    const sec = row[cols.sec];
    if (std.math.isNan(sec)) return std.math.nan(f64);
    if (cols.nsec) |nsec_col| {
        const nsec = row[nsec_col];
        if (std.math.isNan(nsec)) return std.math.nan(f64);
        return sec + nsec / 1_000_000_000.0;
    }
    return sec;
}

fn computeStats(tbl: csv.Table, time_cols: TimeCols, expected: ?f64) !?Stats {
    var samples: usize = 0;
    var first: f64 = 0;
    var last: f64 = 0;
    var min_dt: f64 = std.math.inf(f64);
    var max_dt: f64 = 0;
    var sum_dt: f64 = 0;
    var positive_steps: usize = 0;
    var out_of_order: usize = 0;
    var gaps: usize = 0;
    const gap_threshold = if (expected) |hz| 3.0 / hz else 1.0;

    for (tbl.rows) |row| {
        const t = rowTime(row, time_cols);
        if (std.math.isNan(t)) continue;
        if (samples == 0) {
            first = t;
        } else {
            const dt = t - last;
            if (dt < 0) {
                out_of_order += 1;
            } else if (dt > 0) {
                min_dt = @min(min_dt, dt);
                max_dt = @max(max_dt, dt);
                sum_dt += dt;
                positive_steps += 1;
                if (dt > gap_threshold) gaps += 1;
            }
        }
        last = t;
        samples += 1;
    }

    if (samples < 2 or positive_steps == 0) return null;
    const duration = last - first;
    if (duration <= 0) return null;
    return .{
        .samples = samples,
        .first = first,
        .last = last,
        .duration = duration,
        .rate_hz = @as(f64, @floatFromInt(samples - 1)) / duration,
        .min_dt = min_dt,
        .max_dt = max_dt,
        .mean_dt = sum_dt / @as(f64, @floatFromInt(positive_steps)),
        .out_of_order = out_of_order,
        .gaps = gaps,
    };
}

test "timestamp check accepts steady profile rate" {
    const txt =
        \\time,value
        \\0.0,1
        \\0.1,2
        \\0.2,3
        \\0.3,4
    ;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const tbl = try csv.parse(arena.allocator(), txt);
    const stats = (try computeStats(tbl, .{ .sec = tbl.col(&.{"time"}).? }, 10.0)).?;
    try std.testing.expectApproxEqAbs(@as(f64, 10.0), stats.rate_hz, 0.001);
    try std.testing.expect(stats.out_of_order == 0);
    try std.testing.expect(stats.gaps == 0);
}

test "timestamp check counts out of order and gaps" {
    const txt =
        \\stamp,value
        \\0.0,1
        \\0.1,2
        \\1.0,3
        \\0.9,4
    ;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const tbl = try csv.parse(arena.allocator(), txt);
    const stats = (try computeStats(tbl, .{ .sec = tbl.col(&.{"stamp"}).? }, 10.0)).?;
    try std.testing.expect(stats.out_of_order == 1);
    try std.testing.expect(stats.gaps == 1);
}

test "timestamp check combines ROS sec and nanosec stamp columns" {
    const txt =
        \\header.stamp.sec,header.stamp.nanosec,value
        \\10,0,1
        \\10,500000000,2
        \\11,0,3
    ;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const tbl = try csv.parse(arena.allocator(), txt);
    const cols = findTimeCols(tbl, null).?;
    const stats = (try computeStats(tbl, cols, 2.0)).?;
    try std.testing.expectApproxEqAbs(@as(f64, 2.0), stats.rate_hz, 0.001);
}
