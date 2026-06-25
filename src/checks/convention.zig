//! Cross-sensor orientation comparison: median per-axis difference between two
//! orientation logs, flagging differences near 90 or 180 degrees.
const std = @import("std");
const Allocator = std.mem.Allocator;
const csv = @import("../parsers/csv.zig");
const Reporter = @import("../finding.zig").Reporter;

const Rpy = struct { roll: f64, pitch: f64, yaw: f64 };

fn extractRpy(gpa: Allocator, tbl: csv.Table) !?[]Rpy {
    const cr = tbl.col(&.{ "roll" });
    const cp = tbl.col(&.{ "pitch" });
    const cyaw = tbl.col(&.{ "yaw" });
    if (cr != null and cp != null and cyaw != null) {
        var out: std.ArrayList(Rpy) = .empty;
        for (tbl.rows) |row| {
            try out.append(gpa, .{ .roll = row[cr.?], .pitch = row[cp.?], .yaw = row[cyaw.?] });
        }
        return try out.toOwnedSlice(gpa);
    }

    // Quaternion fallback.
    const qx = tbl.col(&.{ "orientation.x", "qx", "quat_x" });
    const qy = tbl.col(&.{ "orientation.y", "qy", "quat_y" });
    const qz = tbl.col(&.{ "orientation.z", "qz", "quat_z" });
    const qw = tbl.col(&.{ "orientation.w", "qw", "quat_w" });
    if (qx != null and qy != null and qz != null and qw != null) {
        var out: std.ArrayList(Rpy) = .empty;
        for (tbl.rows) |row| {
            try out.append(gpa, quatToRpy(row[qx.?], row[qy.?], row[qz.?], row[qw.?]));
        }
        return try out.toOwnedSlice(gpa);
    }
    return null;
}

fn quatToRpy(x: f64, y: f64, z: f64, w: f64) Rpy {
    // ZYX (yaw-pitch-roll) convention.
    const sinr_cosp = 2.0 * (w * x + y * z);
    const cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const roll = std.math.atan2(sinr_cosp, cosr_cosp);

    const sinp = 2.0 * (w * y - z * x);
    const half_pi: f64 = std.math.pi / 2.0;
    const pitch = if (@abs(sinp) >= 1.0)
        std.math.copysign(half_pi, sinp)
    else
        std.math.asin(sinp);

    const siny_cosp = 2.0 * (w * z + x * y);
    const cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const yaw = std.math.atan2(siny_cosp, cosy_cosp);
    return .{ .roll = roll, .pitch = pitch, .yaw = yaw };
}

fn median(gpa: Allocator, vals: []const f64) !f64 {
    if (vals.len == 0) return std.math.nan(f64);
    const copy = try gpa.dupe(f64, vals);
    std.mem.sort(f64, copy, {}, std.sort.asc(f64));
    return copy[copy.len / 2];
}

const deg = 180.0 / std.math.pi;

fn classifyAndEmit(gpa: Allocator, rep: *Reporter, axis: []const u8, med_deg: f64) !void {
    const a = @abs(med_deg);
    const near = struct {
        fn f(v: f64, target: f64) bool {
            return @abs(v - target) < 15.0;
        }
    }.f;

    if (near(a, 180.0)) {
        try rep.emit(.{
            .id = "convention/flip-180",
            .severity = .fail,
            .summary = try std.fmt.allocPrint(gpa, "{s} differs by {d:.1} deg (~180): a frame flip.", .{ axis, med_deg }),
            .causes = &.{
                .{ .text = "One sensor mounted upside down relative to the other.", .confidence = "high" },
                .{ .text = "ENU vs NED or FLU vs FRD mismatch between the two streams.", .confidence = "high", .confirm = "Check each driver's declared convention." },
            },
        });
    } else if (near(a, 90.0)) {
        try rep.emit(.{
            .id = "convention/rotate-90",
            .severity = .fail,
            .summary = try std.fmt.allocPrint(gpa, "{s} differs by {d:.1} deg (~90): an axis swap.", .{ axis, med_deg }),
            .causes = &.{
                .{ .text = "X/Y axes swapped between the two frames (e.g. RFU vs FLU).", .confidence = "high", .fix = "Add a 90 deg yaw in the static transform or fix the driver mapping." },
            },
        });
    } else if (a < 5.0) {
        try rep.emit(.{
            .id = "convention/agree",
            .severity = .pass,
            .summary = try std.fmt.allocPrint(gpa, "{s} agree (median diff {d:.1} deg).", .{ axis, med_deg }),
        });
    } else {
        try rep.emit(.{
            .id = "convention/offset",
            .severity = .warn,
            .summary = try std.fmt.allocPrint(gpa, "{s} differ by {d:.1} deg (not a clean 90/180).", .{ axis, med_deg }),
            .detail = "A non-axis-aligned offset suggests a real mounting angle, timing skew, or scale issue rather than a convention bug.",
        });
    }
}

pub fn compare(gpa: Allocator, rep: *Reporter, a: csv.Table, b: csv.Table) !void {
    const ra = (try extractRpy(gpa, a)) orelse {
        try rep.emit(.{ .id = "convention/no-orientation", .severity = .warn, .summary = "First log has no roll/pitch/yaw or quaternion columns." });
        return;
    };
    const rb = (try extractRpy(gpa, b)) orelse {
        try rep.emit(.{ .id = "convention/no-orientation", .severity = .warn, .summary = "Second log has no roll/pitch/yaw or quaternion columns." });
        return;
    };

    const n = @min(ra.len, rb.len);
    if (n == 0) {
        try rep.emit(.{ .id = "convention/empty", .severity = .warn, .summary = "One of the logs has no rows." });
        return;
    }

    var dr = try gpa.alloc(f64, n);
    var dp = try gpa.alloc(f64, n);
    var dy = try gpa.alloc(f64, n);
    for (0..n) |i| {
        dr[i] = wrapDeg((ra[i].roll - rb[i].roll) * deg);
        dp[i] = wrapDeg((ra[i].pitch - rb[i].pitch) * deg);
        dy[i] = wrapDeg((ra[i].yaw - rb[i].yaw) * deg);
    }

    try classifyAndEmit(gpa, rep, "Roll", try median(gpa, dr));
    try classifyAndEmit(gpa, rep, "Pitch", try median(gpa, dp));
    try classifyAndEmit(gpa, rep, "Yaw", try median(gpa, dy));
}

fn wrapDeg(d: f64) f64 {
    var x = d;
    while (x > 180.0) x -= 360.0;
    while (x < -180.0) x += 360.0;
    return x;
}
