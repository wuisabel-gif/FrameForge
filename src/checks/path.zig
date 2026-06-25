//! path / why-not-linked: find a path between two frames, or report the two
//! disconnected components, their roots, and a likely bridging transform.
//! Connectivity is undirected because tf2 can invert a transform on lookup.
const std = @import("std");
const Allocator = std.mem.Allocator;
const tfdump = @import("../parsers/tfdump.zig");
const Profile = @import("../profile.zig").Profile;
const Reporter = @import("../finding.zig").Reporter;
const Cause = @import("../finding.zig").Cause;

pub const Status = enum {
    connected,
    missing_source,
    missing_target,
    missing_both,
    disconnected,
};

pub const SuggestionSource = enum { none, estimator, static_tf };

pub const Analysis = struct {
    status: Status,
    /// Frame walk from source to target (inclusive), only for `.connected`.
    path: [][]const u8 = &.{},
    /// Component roots, only for `.disconnected`.
    source_root: []const u8 = "",
    target_root: []const u8 = "",
    /// Suggested bridging transform, only for `.disconnected`.
    suggested_parent: []const u8 = "",
    suggested_child: []const u8 = "",
    suggestion_source: SuggestionSource = .none,
};

const NodeSet = std.StringHashMap(void);
const Adjacency = std.StringHashMap(std.ArrayList([]const u8));

fn nodeSet(gpa: Allocator, tree: tfdump.Tree) !NodeSet {
    var set = NodeSet.init(gpa);
    for (tree.edges) |e| {
        try set.put(e.parent, {});
        try set.put(e.child, {});
    }
    return set;
}

/// child -> parent (first parent wins if a frame has several).
fn parentMap(gpa: Allocator, tree: tfdump.Tree) !std.StringHashMap([]const u8) {
    var m = std.StringHashMap([]const u8).init(gpa);
    for (tree.edges) |e| {
        if (!m.contains(e.child)) try m.put(e.child, e.parent);
    }
    return m;
}

fn undirectedAdjacency(gpa: Allocator, tree: tfdump.Tree) !Adjacency {
    var adj = Adjacency.init(gpa);
    for (tree.edges) |e| {
        try addNeighbor(gpa, &adj, e.parent, e.child);
        try addNeighbor(gpa, &adj, e.child, e.parent);
    }
    return adj;
}

fn addNeighbor(gpa: Allocator, adj: *Adjacency, a: []const u8, b: []const u8) !void {
    const gop = try adj.getOrPut(a);
    if (!gop.found_existing) gop.value_ptr.* = .empty;
    try gop.value_ptr.append(gpa, b);
}

/// Set of frames reachable from `start` over the undirected tree.
fn reachable(gpa: Allocator, adj: Adjacency, start: []const u8) !NodeSet {
    var seen = NodeSet.init(gpa);
    var queue: std.ArrayList([]const u8) = .empty;
    try queue.append(gpa, start);
    try seen.put(start, {});
    var head: usize = 0;
    while (head < queue.items.len) : (head += 1) {
        const cur = queue.items[head];
        if (adj.get(cur)) |nbrs| {
            for (nbrs.items) |nb| {
                if (!seen.contains(nb)) {
                    try seen.put(nb, {});
                    try queue.append(gpa, nb);
                }
            }
        }
    }
    return seen;
}

/// Shortest undirected frame walk from `source` to `target`, or null if none.
fn findPath(gpa: Allocator, adj: Adjacency, source: []const u8, target: []const u8) !?[][]const u8 {
    var pred = std.StringHashMap([]const u8).init(gpa);
    var seen = NodeSet.init(gpa);
    var queue: std.ArrayList([]const u8) = .empty;
    try queue.append(gpa, source);
    try seen.put(source, {});
    var head: usize = 0;
    var found = std.mem.eql(u8, source, target);
    while (head < queue.items.len and !found) : (head += 1) {
        const cur = queue.items[head];
        if (adj.get(cur)) |nbrs| {
            for (nbrs.items) |nb| {
                if (!seen.contains(nb)) {
                    try seen.put(nb, {});
                    try pred.put(nb, cur);
                    if (std.mem.eql(u8, nb, target)) {
                        found = true;
                        break;
                    }
                    try queue.append(gpa, nb);
                }
            }
        }
    }
    if (!found) return null;

    // Walk predecessors target -> source, then reverse.
    var rev: std.ArrayList([]const u8) = .empty;
    var node = target;
    try rev.append(gpa, node);
    while (!std.mem.eql(u8, node, source)) {
        node = pred.get(node).?;
        try rev.append(gpa, node);
    }
    const path = try gpa.alloc([]const u8, rev.items.len);
    for (rev.items, 0..) |n, i| path[path.len - 1 - i] = n;
    return path;
}

/// Walk parent links up from `frame` to the component's TF root (the first
/// ancestor with no parent). Cycle-safe.
fn componentRoot(gpa: Allocator, pmap: std.StringHashMap([]const u8), frame: []const u8) ![]const u8 {
    var visited = NodeSet.init(gpa);
    var cur = frame;
    while (pmap.get(cur)) |p| {
        if (visited.contains(cur)) break; // cycle guard
        try visited.put(cur, {});
        cur = p;
    }
    return cur;
}

pub fn analyze(
    gpa: Allocator,
    tree: tfdump.Tree,
    source: []const u8,
    target: []const u8,
    profile: ?Profile,
) !Analysis {
    var nodes = try nodeSet(gpa, tree);
    const have_src = nodes.contains(source);
    const have_tgt = nodes.contains(target);

    if (!have_src and !have_tgt) return .{ .status = .missing_both };
    if (!have_src) return .{ .status = .missing_source };
    if (!have_tgt) return .{ .status = .missing_target };

    const adj = try undirectedAdjacency(gpa, tree);

    if (try findPath(gpa, adj, source, target)) |path| {
        return .{ .status = .connected, .path = path };
    }

    // Disconnected: characterize both components and suggest a bridge.
    const pmap = try parentMap(gpa, tree);
    const src_root = try componentRoot(gpa, pmap, source);
    const tgt_root = try componentRoot(gpa, pmap, target);
    const comp_s = try reachable(gpa, adj, source);
    const comp_t = try reachable(gpa, adj, target);

    var result: Analysis = .{
        .status = .disconnected,
        .source_root = src_root,
        .target_root = tgt_root,
        .suggested_parent = src_root,
        .suggested_child = tgt_root,
        .suggestion_source = .none,
    };

    if (profile) |p| {
        // Prefer the estimator edge if it bridges the two components.
        if (p.est_tf_parent.len > 0 and p.est_tf_child.len > 0 and
            bridges(comp_s, comp_t, p.est_tf_parent, p.est_tf_child))
        {
            result.suggested_parent = p.est_tf_parent;
            result.suggested_child = p.est_tf_child;
            result.suggestion_source = .estimator;
        } else {
            // Otherwise look for a profile static transform that bridges them.
            for (p.static_tfs) |t| {
                if (t.parent.len > 0 and t.child.len > 0 and bridges(comp_s, comp_t, t.parent, t.child)) {
                    result.suggested_parent = t.parent;
                    result.suggested_child = t.child;
                    result.suggestion_source = .static_tf;
                    break;
                }
            }
        }
    }
    return result;
}

/// True if edge (a,b) has one endpoint in each component (in either direction).
fn bridges(comp_s: NodeSet, comp_t: NodeSet, a: []const u8, b: []const u8) bool {
    return (comp_s.contains(a) and comp_t.contains(b)) or
        (comp_t.contains(a) and comp_s.contains(b));
}

pub fn run(
    gpa: Allocator,
    rep: *Reporter,
    tree: tfdump.Tree,
    source: []const u8,
    target: []const u8,
    profile: ?Profile,
) !void {
    const a = try analyze(gpa, tree, source, target, profile);
    switch (a.status) {
        .connected => {
            const walk = try joinArrow(gpa, a.path);
            try rep.emit(.{
                .id = "path/connected",
                .severity = .pass,
                .summary = try std.fmt.allocPrint(gpa, "{s} and {s} are connected ({d} hop(s)).", .{ source, target, a.path.len - 1 }),
                .detail = try std.fmt.allocPrint(gpa, "Path: {s}", .{walk}),
            });
        },
        .missing_source => try emitMissing(gpa, rep, source, target, source),
        .missing_target => try emitMissing(gpa, rep, source, target, target),
        .missing_both => {
            try rep.emit(.{
                .id = "path/missing-both",
                .severity = .fail,
                .summary = try std.fmt.allocPrint(gpa, "Neither {s} nor {s} exists in the TF tree.", .{ source, target }),
                .detail = "Both frames are absent — check for typos and whether the publishers (robot_state_publisher, the estimator) are running.",
            });
        },
        .disconnected => {
            const cause = try disconnectCauses(gpa, a, profile);
            try rep.emit(.{
                .id = "path/disconnected",
                .severity = .fail,
                .summary = try std.fmt.allocPrint(gpa, "No path from {s} to {s}.", .{ source, target }),
                .detail = try std.fmt.allocPrint(
                    gpa,
                    "{s} exists in component rooted at {s}.\n{s} exists in component rooted at {s}.\nLikely missing transform: {s} -> {s}.",
                    .{ source, a.source_root, target, a.target_root, a.suggested_parent, a.suggested_child },
                ),
                .causes = cause,
            });
        },
    }
}

fn emitMissing(gpa: Allocator, rep: *Reporter, source: []const u8, target: []const u8, missing: []const u8) !void {
    try rep.emit(.{
        .id = "path/missing-frame",
        .severity = .fail,
        .summary = try std.fmt.allocPrint(gpa, "Frame '{s}' does not exist in the TF tree.", .{missing}),
        .detail = try std.fmt.allocPrint(gpa, "Cannot look up {s} -> {s} because '{s}' is never published.", .{ source, target, missing }),
        .causes = &.{
            .{ .text = "Frame name typo (check exact spelling and namespace).", .confidence = "high", .confirm = "Compare against the URDF link names and the live frame list." },
            .{ .text = "The node that should publish this frame is not running.", .confidence = "high", .confirm = "ros2 node list; check robot_state_publisher / the estimator." },
        },
    });
}

fn disconnectCauses(gpa: Allocator, a: Analysis, profile: ?Profile) ![]const Cause {
    const robot = if (profile) |p| p.robot else "";
    switch (a.suggestion_source) {
        .estimator => {
            const est = if (robot.len > 0)
                try std.fmt.allocPrint(gpa, "{s}_estimation has not initialized yet, or its TF broadcaster is not running.", .{robot})
            else
                "The estimator has not initialized yet, or its TF broadcaster is not running.";
            var causes = try gpa.alloc(Cause, 2);
            causes[0] = .{
                .text = est,
                .confidence = "high",
                .confirm = "Check the estimator reports a valid key index, and that it broadcasts the TF.",
                .fix = try std.fmt.allocPrint(gpa, "Ensure the estimator publishes {s} -> {s} after init.", .{ a.suggested_parent, a.suggested_child }),
            };
            causes[1] = .{
                .text = "robot_state_publisher not running, so the static chain to the body frame is absent.",
                .confidence = "medium",
                .confirm = "ros2 node list | grep robot_state_publisher",
            };
            return causes;
        },
        .static_tf => {
            var causes = try gpa.alloc(Cause, 2);
            causes[0] = .{
                .text = try std.fmt.allocPrint(gpa, "The static transform {s} -> {s} (declared in the profile) is not being published.", .{ a.suggested_parent, a.suggested_child }),
                .confidence = "high",
                .confirm = "Check /tf_static and the launch file for this static_transform_publisher.",
                .fix = "Add the missing static_transform_publisher (or URDF joint).",
            };
            causes[1] = .{
                .text = "A launch file failed to start, dropping a whole subtree.",
                .confidence = "low",
            };
            return causes;
        },
        .none => {
            var causes = try gpa.alloc(Cause, 1);
            causes[0] = .{
                .text = try std.fmt.allocPrint(gpa, "No transform links the two components. A bridge {s} -> {s} is missing; the profile did not name a likely source.", .{ a.suggested_parent, a.suggested_child }),
                .confidence = "medium",
                .confirm = "Identify which node should publish a transform between these subtrees.",
                .fix = "Publish a transform connecting the two roots, or add it to the profile.",
            };
            return causes;
        },
    }
}

fn joinArrow(gpa: Allocator, items: [][]const u8) ![]const u8 {
    var buf: std.ArrayList(u8) = .empty;
    for (items, 0..) |s, i| {
        if (i > 0) try buf.appendSlice(gpa, " -> ");
        try buf.appendSlice(gpa, s);
    }
    return buf.toOwnedSlice(gpa);
}

const testing = std.testing;

fn makeTree(edges: []const tfdump.Edge) tfdump.Tree {
    return .{ .edges = @constCast(edges) };
}

test "connected path exists" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const tree = makeTree(&.{
        .{ .parent = "odom", .child = "barracuda/base_link" },
        .{ .parent = "barracuda/base_link", .child = "barracuda/imu_link" },
    });
    const r = try analyze(a, tree, "odom", "barracuda/imu_link", null);
    try testing.expect(r.status == .connected);
    try testing.expect(r.path.len == 3);
    try testing.expectEqualStrings("odom", r.path[0]);
    try testing.expectEqualStrings("barracuda/imu_link", r.path[2]);
}

test "missing source frame" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const tree = makeTree(&.{.{ .parent = "odom", .child = "barracuda/base_link" }});
    const r = try analyze(a, tree, "nope", "barracuda/base_link", null);
    try testing.expect(r.status == .missing_source);
}

test "missing target frame" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const tree = makeTree(&.{.{ .parent = "odom", .child = "barracuda/base_link" }});
    const r = try analyze(a, tree, "odom", "barracuda/dvl_link", null);
    try testing.expect(r.status == .missing_target);
}

test "disconnected components" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    // odom -> camera   and   base_link -> imu  are two separate trees.
    const tree = makeTree(&.{
        .{ .parent = "odom", .child = "barracuda/camera_link" },
        .{ .parent = "barracuda/base_link", .child = "barracuda/imu_link" },
    });
    const r = try analyze(a, tree, "odom", "barracuda/base_link", null);
    try testing.expect(r.status == .disconnected);
    try testing.expectEqualStrings("odom", r.source_root);
    try testing.expectEqualStrings("barracuda/base_link", r.target_root);
    // No profile -> generic suggestion of root -> root.
    try testing.expect(r.suggestion_source == .none);
    try testing.expectEqualStrings("odom", r.suggested_parent);
    try testing.expectEqualStrings("barracuda/base_link", r.suggested_child);
}

test "missing estimator edge from profile bridges the components" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const tree = makeTree(&.{
        .{ .parent = "odom", .child = "barracuda/camera_link" },
        .{ .parent = "barracuda/base_link", .child = "barracuda/imu_link" },
    });
    const profile: Profile = .{
        .robot = "barracuda",
        .world_frame = "odom",
        .base_frame = "barracuda/base_link",
        .est_tf_parent = "odom",
        .est_tf_child = "barracuda/base_link",
    };
    const r = try analyze(a, tree, "odom", "barracuda/base_link", profile);
    try testing.expect(r.status == .disconnected);
    try testing.expect(r.suggestion_source == .estimator);
    try testing.expectEqualStrings("odom", r.suggested_parent);
    try testing.expectEqualStrings("barracuda/base_link", r.suggested_child);
}
