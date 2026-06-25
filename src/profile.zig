//! Robot profile loader.
//!
//! This is not a general YAML parser — it understands exactly the constructs the
//! profile schema uses (top-level scalars, a few nested mappings, and lists of
//! mappings). Keeping it focused makes it robust and dependency-free, which is
//! the whole point of the single-binary design.
const std = @import("std");
const Allocator = std.mem.Allocator;

pub const Sensor = struct {
    topic: []const u8 = "",
    type: []const u8 = "",
    frame: []const u8 = "",
    convention: []const u8 = "",
    rate_hz: ?f64 = null,
};

pub const StaticTf = struct {
    parent: []const u8 = "",
    child: []const u8 = "",
};

pub const Consumer = struct {
    node: []const u8 = "",
    must_subscribe: []const u8 = "",
};

pub const Profile = struct {
    robot: []const u8 = "",
    world_frame: []const u8 = "",
    base_frame: []const u8 = "",

    conv_world: []const u8 = "",
    conv_body: []const u8 = "",
    gravity_magnitude: ?f64 = null,
    gravity_tol: f64 = 0.2,

    est_pose_topic: []const u8 = "",
    est_odom_topic: []const u8 = "",
    est_output_frame: []const u8 = "",
    est_tf_parent: []const u8 = "",
    est_tf_child: []const u8 = "",

    sensors: []Sensor = &.{},
    static_tfs: []StaticTf = &.{},
    consumers: []Consumer = &.{},

    pub fn sensorByFrame(self: Profile, frame: []const u8) ?Sensor {
        for (self.sensors) |s| {
            if (std.mem.eql(u8, s.frame, frame)) return s;
        }
        return null;
    }
};

const Section = enum { none, conventions, estimator, sensors, static_transforms, consumers };

fn indentOf(line: []const u8) usize {
    var n: usize = 0;
    while (n < line.len and line[n] == ' ') n += 1;
    return n;
}

/// Strip a trailing `# comment`. A '#' only starts a comment at line start or
/// when preceded by whitespace, so it never eats a '#' embedded in a value.
fn stripComment(line: []const u8) []const u8 {
    if (line.len > 0 and line[0] == '#') return "";
    var i: usize = 0;
    while (i < line.len) : (i += 1) {
        if (line[i] == '#' and i > 0 and (line[i - 1] == ' ' or line[i - 1] == '\t')) {
            return line[0..i];
        }
    }
    return line;
}

fn unquote(s: []const u8) []const u8 {
    const t = std.mem.trim(u8, s, " \t\r");
    if (t.len >= 2 and ((t[0] == '"' and t[t.len - 1] == '"') or (t[0] == '\'' and t[t.len - 1] == '\''))) {
        return t[1 .. t.len - 1];
    }
    return t;
}

/// Split `key: value` on the first colon. Returns null if there is no colon.
fn splitKv(line: []const u8) ?struct { key: []const u8, val: []const u8 } {
    const idx = std.mem.indexOfScalar(u8, line, ':') orelse return null;
    return .{
        .key = std.mem.trim(u8, line[0..idx], " \t\r-"),
        .val = unquote(line[idx + 1 ..]),
    };
}

pub fn parse(gpa: Allocator, text: []const u8) !Profile {
    var p: Profile = .{};
    var sensors: std.ArrayList(Sensor) = .empty;
    var tfs: std.ArrayList(StaticTf) = .empty;
    var consumers: std.ArrayList(Consumer) = .empty;

    var section: Section = .none;
    var lines = std.mem.splitScalar(u8, text, '\n');
    while (lines.next()) |raw| {
        const noc = stripComment(raw);
        const trimmed = std.mem.trim(u8, noc, " \t\r");
        if (trimmed.len == 0) continue;
        const indent = indentOf(noc);

        // A top-level key (indent 0) re-selects the active section.
        if (indent == 0) {
            section = .none;
            if (splitKv(trimmed)) |kv| {
                if (std.mem.eql(u8, kv.key, "conventions")) {
                    section = .conventions;
                } else if (std.mem.eql(u8, kv.key, "estimator")) {
                    section = .estimator;
                } else if (std.mem.eql(u8, kv.key, "sensors")) {
                    section = .sensors;
                } else if (std.mem.eql(u8, kv.key, "static_transforms")) {
                    section = .static_transforms;
                } else if (std.mem.eql(u8, kv.key, "consumers")) {
                    section = .consumers;
                } else if (std.mem.eql(u8, kv.key, "robot")) {
                    p.robot = try gpa.dupe(u8, kv.val);
                } else if (std.mem.eql(u8, kv.key, "world_frame")) {
                    p.world_frame = try gpa.dupe(u8, kv.val);
                } else if (std.mem.eql(u8, kv.key, "base_frame")) {
                    p.base_frame = try gpa.dupe(u8, kv.val);
                }
            }
            continue;
        }

        const is_item = std.mem.startsWith(u8, trimmed, "- ");
        const kv = splitKv(trimmed) orelse continue;
        const key = kv.key;
        const val = try gpa.dupe(u8, kv.val);

        switch (section) {
            .conventions => {
                if (std.mem.eql(u8, key, "world")) p.conv_world = val;
                if (std.mem.eql(u8, key, "body")) p.conv_body = val;
                if (std.mem.eql(u8, key, "gravity_magnitude")) p.gravity_magnitude = parseF(val);
                if (std.mem.eql(u8, key, "gravity_tol")) {
                    if (parseF(val)) |f| p.gravity_tol = f;
                }
            },
            .estimator => {
                if (std.mem.eql(u8, key, "pose_topic")) p.est_pose_topic = val;
                if (std.mem.eql(u8, key, "odom_topic")) p.est_odom_topic = val;
                if (std.mem.eql(u8, key, "output_frame")) p.est_output_frame = val;
                if (std.mem.eql(u8, key, "publishes_tf")) {
                    if (std.mem.indexOf(u8, val, "->")) |arrow| {
                        p.est_tf_parent = std.mem.trim(u8, val[0..arrow], " \t");
                        p.est_tf_child = std.mem.trim(u8, val[arrow + 2 ..], " \t");
                    }
                }
            },
            .sensors => {
                if (is_item) try sensors.append(gpa, .{});
                if (sensors.items.len == 0) continue;
                var cur = &sensors.items[sensors.items.len - 1];
                if (std.mem.eql(u8, key, "topic")) cur.topic = val;
                if (std.mem.eql(u8, key, "type")) cur.type = val;
                if (std.mem.eql(u8, key, "frame")) cur.frame = val;
                if (std.mem.eql(u8, key, "convention")) cur.convention = val;
                if (std.mem.eql(u8, key, "rate_hz")) cur.rate_hz = parseF(val);
            },
            .static_transforms => {
                if (is_item) try tfs.append(gpa, .{});
                if (tfs.items.len == 0) continue;
                var cur = &tfs.items[tfs.items.len - 1];
                if (std.mem.eql(u8, key, "parent")) cur.parent = val;
                if (std.mem.eql(u8, key, "child")) cur.child = val;
            },
            .consumers => {
                if (is_item) try consumers.append(gpa, .{});
                if (consumers.items.len == 0) continue;
                var cur = &consumers.items[consumers.items.len - 1];
                if (std.mem.eql(u8, key, "node")) cur.node = val;
                if (std.mem.eql(u8, key, "must_subscribe")) cur.must_subscribe = val;
            },
            .none => {},
        }
    }

    p.sensors = try sensors.toOwnedSlice(gpa);
    p.static_tfs = try tfs.toOwnedSlice(gpa);
    p.consumers = try consumers.toOwnedSlice(gpa);
    return p;
}

fn parseF(s: []const u8) ?f64 {
    return std.fmt.parseFloat(f64, std.mem.trim(u8, s, " \t\r")) catch null;
}

test "parse minimal profile" {
    const txt =
        \\robot: barracuda
        \\world_frame: odom
        \\base_frame: barracuda/base_link
        \\conventions:
        \\  world: ENU
        \\  gravity_magnitude: 9.56
        \\estimator:
        \\  publishes_tf: odom -> barracuda/base_link
        \\sensors:
        \\  - topic: /imu
        \\    frame: barracuda/imu_link
        \\    convention: FLU
        \\    rate_hz: 40
    ;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const p = try parse(arena.allocator(), txt);
    try std.testing.expectEqualStrings("odom", p.world_frame);
    try std.testing.expect(p.gravity_magnitude.? == 9.56);
    try std.testing.expectEqualStrings("barracuda/base_link", p.est_tf_child);
    try std.testing.expect(p.sensors.len == 1);
    try std.testing.expectEqualStrings("FLU", p.sensors[0].convention);
}
