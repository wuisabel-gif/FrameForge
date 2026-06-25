//! TF tree structure check: roots, cycles, multiple parents, and agreement with
//! the profile's expected world/base frames and estimator TF edge.
const std = @import("std");
const Allocator = std.mem.Allocator;
const tfdump = @import("../parsers/tfdump.zig");
const Profile = @import("../profile.zig").Profile;
const Reporter = @import("../finding.zig").Reporter;
const Cause = @import("../finding.zig").Cause;

pub fn run(gpa: Allocator, rep: *Reporter, tree: tfdump.Tree, profile: ?Profile) !void {
    // children-per-node and parent-per-node maps.
    var parents = std.StringHashMap(std.ArrayList([]const u8)).init(gpa);
    var is_child = std.StringHashMap(void).init(gpa);
    var nodes = std.StringHashMap(void).init(gpa);

    for (tree.edges) |e| {
        try nodes.put(e.parent, {});
        try nodes.put(e.child, {});
        try is_child.put(e.child, {});
        const gop = try parents.getOrPut(e.child);
        if (!gop.found_existing) gop.value_ptr.* = .empty;
        try gop.value_ptr.append(gpa, e.parent);
    }

    // Roots = nodes that are never a child.
    var roots: std.ArrayList([]const u8) = .empty;
    var it = nodes.keyIterator();
    while (it.next()) |k| {
        if (!is_child.contains(k.*)) try roots.append(gpa, k.*);
    }

    // --- Root count ---
    if (roots.items.len == 0) {
        try rep.emit(.{
            .id = "structure/no-root",
            .severity = .fail,
            .summary = "TF tree has no root (every frame is also a child) — there is a cycle.",
        });
    } else if (roots.items.len > 1) {
        const list = try join(gpa, roots.items);
        try rep.emit(.{
            .id = "structure/multi-root",
            .severity = .fail,
            .summary = try std.fmt.allocPrint(gpa, "TF tree is a forest, not a tree: {d} roots ({s}).", .{ roots.items.len, list }),
            .detail = "A connected robot TF tree must have exactly one root.",
            .causes = &.{
                .{ .text = "A required static transform between subtrees is not being published.", .confidence = "high", .confirm = "ros2 run tf2_tools view_frames; look for the disconnected island.", .fix = "Add the missing static_transform_publisher (or URDF joint)." },
                .{ .text = "A frame name typo split one logical frame into two.", .confidence = "low", .confirm = "Diff the root names against the URDF link names." },
            },
        });
    } else {
        try rep.emit(.{
            .id = "structure/single-root",
            .severity = .pass,
            .summary = try std.fmt.allocPrint(gpa, "Single TF root: {s}", .{roots.items[0]}),
        });
    }

    // --- Multiple parents ---
    var pit = parents.iterator();
    while (pit.next()) |entry| {
        if (entry.value_ptr.items.len > 1) {
            const list = try join(gpa, entry.value_ptr.items);
            try rep.emit(.{
                .id = "structure/multi-parent",
                .severity = .fail,
                .summary = try std.fmt.allocPrint(gpa, "Frame '{s}' has {d} parents ({s}).", .{ entry.key_ptr.*, entry.value_ptr.items.len, list }),
                .detail = "Each frame must have exactly one parent in a TF tree.",
                .causes = &.{
                    .{ .text = "Two nodes are both broadcasting this transform (duplicate publisher).", .confidence = "high", .confirm = "Check the Broadcaster column in view_frames for this edge." },
                    .{ .text = "A launch file starts the same static_transform_publisher twice.", .confidence = "medium" },
                },
            });
        }
    }

    // --- Cycle detection (DFS) ---
    if (try hasCycle(gpa, tree.edges, nodes)) {
        try rep.emit(.{
            .id = "structure/cycle",
            .severity = .fail,
            .summary = "TF tree contains a cycle.",
            .detail = "Transforms form a loop; lookups will be ambiguous or fail.",
        });
    }

    // --- Profile agreement ---
    if (profile) |p| {
        if (p.world_frame.len > 0 and !nodes.contains(p.world_frame)) {
            try rep.emit(.{
                .id = "structure/world-frame-absent",
                .severity = .warn,
                .summary = try std.fmt.allocPrint(gpa, "Profile world_frame '{s}' is not present in the TF dump.", .{p.world_frame}),
                .detail = if (roots.items.len == 1)
                    try std.fmt.allocPrint(gpa, "The live root is '{s}' instead.", .{roots.items[0]})
                else
                    "",
                .causes = &.{
                    .{ .text = "The dump was captured from a replay/nav session with a different frame layout.", .confidence = "medium", .confirm = "Re-capture view_frames on the live estimator stack." },
                },
            });
        }
        if (p.base_frame.len > 0 and !nodes.contains(p.base_frame)) {
            try rep.emit(.{
                .id = "structure/base-frame-absent",
                .severity = .fail,
                .summary = try std.fmt.allocPrint(gpa, "Profile base_frame '{s}' is missing from the TF tree.", .{p.base_frame}),
                .detail = "Downstream control/nav cannot transform into the robot body.",
                .causes = &.{
                    .{ .text = "robot_state_publisher was not running, so URDF frames (base_link, imu_link, ...) were never published.", .confidence = "high", .confirm = "ros2 node list | grep robot_state_publisher", .fix = "Launch robot_state_publisher with the URDF so base_link and sensor frames exist." },
                    .{ .text = "The estimator publishes odom -> <sensor> directly instead of odom -> base_link.", .confidence = "medium", .confirm = "Inspect the estimator's child_frame_id." },
                },
            });
        }
        // Estimator TF edge present?
        if (p.est_tf_parent.len > 0 and p.est_tf_child.len > 0) {
            var found = false;
            for (tree.edges) |e| {
                if (std.mem.eql(u8, e.parent, p.est_tf_parent) and std.mem.eql(u8, e.child, p.est_tf_child)) found = true;
            }
            if (!found) {
                try rep.emit(.{
                    .id = "structure/estimator-edge-missing",
                    .severity = .warn,
                    .summary = try std.fmt.allocPrint(gpa, "Declared estimator TF edge '{s} -> {s}' not found in the dump.", .{ p.est_tf_parent, p.est_tf_child }),
                    .causes = &.{
                        .{ .text = "Estimator had not initialized when the dump was captured (TF-only-after-init).", .confidence = "medium", .confirm = "Capture after the estimator reports a valid key index." },
                    },
                });
            }
        }
    }
}

fn join(gpa: Allocator, items: [][]const u8) ![]const u8 {
    var buf: std.ArrayList(u8) = .empty;
    for (items, 0..) |s, i| {
        if (i > 0) try buf.appendSlice(gpa, ", ");
        try buf.appendSlice(gpa, s);
    }
    return buf.toOwnedSlice(gpa);
}

fn hasCycle(gpa: Allocator, edges: []tfdump.Edge, nodes: std.StringHashMap(void)) !bool {
    var children = std.StringHashMap(std.ArrayList([]const u8)).init(gpa);
    for (edges) |e| {
        const gop = try children.getOrPut(e.parent);
        if (!gop.found_existing) gop.value_ptr.* = .empty;
        try gop.value_ptr.append(gpa, e.child);
    }
    // 0 = unvisited, 1 = on stack, 2 = done
    var state = std.StringHashMap(u8).init(gpa);
    var it = nodes.keyIterator();
    while (it.next()) |k| {
        if ((state.get(k.*) orelse 0) == 0) {
            if (try dfs(gpa, k.*, &children, &state)) return true;
        }
    }
    return false;
}

fn dfs(
    gpa: Allocator,
    node: []const u8,
    children: *std.StringHashMap(std.ArrayList([]const u8)),
    state: *std.StringHashMap(u8),
) !bool {
    try state.put(node, 1);
    if (children.get(node)) |kids| {
        for (kids.items) |c| {
            const s = state.get(c) orelse 0;
            if (s == 1) return true;
            if (s == 0 and try dfs(gpa, c, children, state)) return true;
        }
    }
    try state.put(node, 2);
    return false;
}
