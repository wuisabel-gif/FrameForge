//! Parser for `view_frames` / `tf2_tools` graphviz (.gv) TF dumps.
//!
//! These files describe the live transform tree as `"parent" -> "child"[label=...]`
//! edges, where the label carries the broadcaster and average publish rate. That
//! is exactly the structural + timing evidence the TF checks need, with no ROS
//! install required.
const std = @import("std");
const Allocator = std.mem.Allocator;

pub const Edge = struct {
    parent: []const u8,
    child: []const u8,
    rate_hz: ?f64 = null,
};

pub const Tree = struct {
    edges: []Edge,

    pub fn nodeCount(self: Tree, gpa: Allocator) !usize {
        var set = std.StringHashMap(void).init(gpa);
        defer set.deinit();
        for (self.edges) |e| {
            try set.put(e.parent, {});
            try set.put(e.child, {});
        }
        return set.count();
    }
};

/// Extract the contents of the first two double-quoted spans in `line`.
fn twoQuoted(line: []const u8) ?struct { a: []const u8, b: []const u8, rest: []const u8 } {
    const a0 = std.mem.indexOfScalar(u8, line, '"') orelse return null;
    const a1 = std.mem.indexOfScalarPos(u8, line, a0 + 1, '"') orelse return null;
    const b0 = std.mem.indexOfScalarPos(u8, line, a1 + 1, '"') orelse return null;
    const b1 = std.mem.indexOfScalarPos(u8, line, b0 + 1, '"') orelse return null;
    return .{ .a = line[a0 + 1 .. a1], .b = line[b0 + 1 .. b1], .rest = line[b1 + 1 ..] };
}

fn parseRate(rest: []const u8) ?f64 {
    const marker = "Average rate:";
    const i = std.mem.indexOf(u8, rest, marker) orelse return null;
    const s = std.mem.trimStart(u8, rest[i + marker.len ..], " \t");
    var end: usize = 0;
    while (end < s.len and (std.ascii.isDigit(s[end]) or s[end] == '.')) end += 1;
    if (end == 0) return null;
    return std.fmt.parseFloat(f64, s[0..end]) catch null;
}

pub fn parse(gpa: Allocator, text: []const u8) !Tree {
    var edges: std.ArrayList(Edge) = .empty;
    var lines = std.mem.splitScalar(u8, text, '\n');
    while (lines.next()) |raw| {
        const line = std.mem.trim(u8, raw, " \t\r");
        // Only real edges: must contain "->" and start with a quoted node name.
        if (std.mem.indexOf(u8, line, "->") == null) continue;
        if (line.len == 0 or line[0] != '"') continue;
        const q = twoQuoted(line) orelse continue;
        try edges.append(gpa, .{
            .parent = try gpa.dupe(u8, q.a),
            .child = try gpa.dupe(u8, q.b),
            .rate_hz = parseRate(q.rest),
        });
    }
    return .{ .edges = try edges.toOwnedSlice(gpa) };
}

test "parse barracuda-style edges" {
    const txt =
        \\digraph G {
        \\"map" -> "odom"[label=" Broadcaster: default_authority\nAverage rate: 10.349\n"];
        \\"odom" -> "barracuda_camera_link"[label=" Average rate: 10.349\n"];
        \\edge [style=invis];
        \\"Recorded at time: 1.0"[ shape=plaintext ] ;
    ;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const tree = try parse(arena.allocator(), txt);
    try std.testing.expect(tree.edges.len == 2);
    try std.testing.expectEqualStrings("map", tree.edges[0].parent);
    try std.testing.expectEqualStrings("odom", tree.edges[0].child);
    try std.testing.expect(tree.edges[0].rate_hz.? > 10.0);
}
