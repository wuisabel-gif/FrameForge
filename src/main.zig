//! FrameForge — a linter and diagnostician for robot frame trees, sensor
//! conventions, and transform consistency. It does not draw the TF tree; it
//! explains why the tree does not make physical or mathematical sense and ranks
//! the likely causes. Checks are generic and driven by a declarative robot
//! profile (see profiles/barracuda.profile.yaml).
const std = @import("std");
const Io = std.Io;

const finding = @import("finding.zig");
const profile_mod = @import("profile.zig");
const tfdump = @import("parsers/tfdump.zig");
const csv = @import("parsers/csv.zig");
const structure = @import("checks/structure.zig");
const gravity = @import("checks/gravity.zig");
const convention = @import("checks/convention.zig");

const Profile = profile_mod.Profile;

const usage =
    \\FrameForge — diagnose robot frame trees, conventions, and transforms.
    \\
    \\USAGE:
    \\  frameforge <command> [args] [--profile <file>] [--no-color]
    \\
    \\COMMANDS:
    \\  tf       <tree.gv>            Validate TF tree structure (roots, cycles,
    \\                               multi-parent) and agreement with the profile.
    \\  gravity  <imu.csv>           Check gravity direction/magnitude from a
    \\                               stationary IMU accel log.
    \\  compare  <a.csv> <b.csv>     Median roll/pitch/yaw difference between two
    \\                               orientation logs; names 90/180 deg bugs.
    \\  validate --profile <file>    Run all checks for which inputs are provided:
    \\           [--tf F] [--imu F]
    \\  profile  <file>              Load a profile and print a summary (sanity).
    \\  help                         Show this help.
    \\
    \\EXAMPLES:
    \\  frameforge tf examples/barracuda_tf.gv --profile profiles/barracuda.profile.yaml
    \\  frameforge validate --profile profiles/barracuda.profile.yaml --tf examples/barracuda_tf.gv
    \\
;

const Args = struct {
    cmd: []const u8 = "",
    positionals: [][]const u8,
    profile_path: ?[]const u8 = null,
    tf_path: ?[]const u8 = null,
    imu_path: ?[]const u8 = null,
    no_color: bool = false,
};

fn parseArgs(gpa: std.mem.Allocator, raw: std.process.Args) !Args {
    var it = std.process.Args.Iterator.init(raw);
    _ = it.next(); // skip argv[0]
    var positionals: std.ArrayList([]const u8) = .empty;
    var a: Args = .{ .positionals = &.{} };
    var first = true;
    while (it.next()) |arg| {
        if (std.mem.eql(u8, arg, "--profile")) {
            a.profile_path = if (it.next()) |v| try gpa.dupe(u8, v) else null;
        } else if (std.mem.eql(u8, arg, "--tf")) {
            a.tf_path = if (it.next()) |v| try gpa.dupe(u8, v) else null;
        } else if (std.mem.eql(u8, arg, "--imu")) {
            a.imu_path = if (it.next()) |v| try gpa.dupe(u8, v) else null;
        } else if (std.mem.eql(u8, arg, "--no-color")) {
            a.no_color = true;
        } else if (first) {
            a.cmd = try gpa.dupe(u8, arg);
            first = false;
        } else {
            try positionals.append(gpa, try gpa.dupe(u8, arg));
        }
    }
    a.positionals = try positionals.toOwnedSlice(gpa);
    return a;
}

fn readFile(io: Io, gpa: std.mem.Allocator, path: []const u8) ![]u8 {
    return Io.Dir.cwd().readFileAlloc(io, path, gpa, .unlimited);
}

fn loadProfile(io: Io, gpa: std.mem.Allocator, path: ?[]const u8, rep: *finding.Reporter) !?Profile {
    const pp = path orelse return null;
    const text = readFile(io, gpa, pp) catch |e| {
        try rep.emit(.{
            .id = "profile/load",
            .severity = .warn,
            .summary = try std.fmt.allocPrint(gpa, "Could not read profile '{s}' ({s}); running profile-free.", .{ pp, @errorName(e) }),
        });
        return null;
    };
    return try profile_mod.parse(gpa, text);
}

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    const gpa = init.arena.allocator();

    var out_buf: [8192]u8 = undefined;
    var fw = Io.File.stdout().writer(io, &out_buf);
    const w = &fw.interface;

    const args = try parseArgs(gpa, init.minimal.args);
    var rep = finding.Reporter.init(w, !args.no_color);

    if (args.cmd.len == 0 or std.mem.eql(u8, args.cmd, "help") or std.mem.eql(u8, args.cmd, "--help")) {
        try w.writeAll(usage);
        try w.flush();
        return 0;
    }

    var code: u8 = 0;

    if (std.mem.eql(u8, args.cmd, "tf")) {
        const path = if (args.positionals.len > 0) args.positionals[0] else args.tf_path;
        if (path == null) {
            try w.writeAll("error: `tf` needs a .gv file.\n");
            try w.flush();
            return 2;
        }
        const prof = try loadProfile(io, gpa, args.profile_path, &rep);
        const tree = try tfdump.parse(gpa, try readFile(io, gpa, path.?));
        try structure.run(gpa, &rep, tree, prof);
        code = try rep.finish(false);
    } else if (std.mem.eql(u8, args.cmd, "gravity")) {
        const path = if (args.positionals.len > 0) args.positionals[0] else args.imu_path;
        if (path == null) {
            try w.writeAll("error: `gravity` needs an IMU .csv file.\n");
            try w.flush();
            return 2;
        }
        const prof = try loadProfile(io, gpa, args.profile_path, &rep);
        const tbl = try csv.parse(gpa, try readFile(io, gpa, path.?));
        try gravity.run(gpa, &rep, tbl, prof);
        code = try rep.finish(false);
    } else if (std.mem.eql(u8, args.cmd, "compare")) {
        if (args.positionals.len < 2) {
            try w.writeAll("error: `compare` needs two .csv files.\n");
            try w.flush();
            return 2;
        }
        const ta = try csv.parse(gpa, try readFile(io, gpa, args.positionals[0]));
        const tb = try csv.parse(gpa, try readFile(io, gpa, args.positionals[1]));
        try convention.compare(gpa, &rep, ta, tb);
        code = try rep.finish(false);
    } else if (std.mem.eql(u8, args.cmd, "validate")) {
        const prof = try loadProfile(io, gpa, args.profile_path, &rep);
        var ran = false;
        if (args.tf_path) |tp| {
            const tree = try tfdump.parse(gpa, try readFile(io, gpa, tp));
            try structure.run(gpa, &rep, tree, prof);
            ran = true;
        }
        if (args.imu_path) |ip| {
            const tbl = try csv.parse(gpa, try readFile(io, gpa, ip));
            try gravity.run(gpa, &rep, tbl, prof);
            ran = true;
        }
        if (!ran) {
            try w.writeAll("error: `validate` needs at least one of --tf or --imu.\n");
            try w.flush();
            return 2;
        }
        code = try rep.finish(false);
    } else if (std.mem.eql(u8, args.cmd, "profile")) {
        if (args.positionals.len < 1) {
            try w.writeAll("error: `profile` needs a profile file.\n");
            try w.flush();
            return 2;
        }
        const p = try profile_mod.parse(gpa, try readFile(io, gpa, args.positionals[0]));
        try printProfile(w, p);
        code = 0;
    } else {
        try w.print("error: unknown command '{s}'.\n\n", .{args.cmd});
        try w.writeAll(usage);
        code = 2;
    }

    try w.flush();
    return code;
}

fn printProfile(w: *Io.Writer, p: Profile) !void {
    try w.print("robot:       {s}\n", .{p.robot});
    try w.print("world_frame: {s}\n", .{p.world_frame});
    try w.print("base_frame:  {s}\n", .{p.base_frame});
    try w.print("conventions: world={s} body={s}", .{ p.conv_world, p.conv_body });
    if (p.gravity_magnitude) |g| try w.print(" gravity={d:.3}", .{g});
    try w.writeAll("\n");
    try w.print("estimator:   {s} | {s} (frame {s})\n", .{ p.est_pose_topic, p.est_odom_topic, p.est_output_frame });
    if (p.est_tf_parent.len > 0) try w.print("  TF edge:   {s} -> {s}\n", .{ p.est_tf_parent, p.est_tf_child });
    try w.print("sensors ({d}):\n", .{p.sensors.len});
    for (p.sensors) |s| {
        try w.print("  - {s}  frame={s}", .{ s.topic, s.frame });
        if (s.convention.len > 0) try w.print("  conv={s}", .{s.convention});
        if (s.rate_hz) |r| try w.print("  {d:.0} Hz", .{r});
        try w.writeAll("\n");
    }
    try w.print("static_transforms ({d}), consumers ({d})\n", .{ p.static_tfs.len, p.consumers.len });
}
