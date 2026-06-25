//! Gravity check: verify specific-force direction and magnitude from a
//! stationary IMU accel log. At rest, ENU/FLU expects a strong +Z.
const std = @import("std");
const Allocator = std.mem.Allocator;
const csv = @import("../parsers/csv.zig");
const Profile = @import("../profile.zig").Profile;
const Reporter = @import("../finding.zig").Reporter;

pub fn run(gpa: Allocator, rep: *Reporter, tbl: csv.Table, profile: ?Profile) !void {
    const cx = tbl.col(&.{ "linear_acceleration.x", "accel_x", "acc_x", "ax" });
    const cy = tbl.col(&.{ "linear_acceleration.y", "accel_y", "acc_y", "ay" });
    const cz = tbl.col(&.{ "linear_acceleration.z", "accel_z", "acc_z", "az" });

    if (cx == null or cy == null or cz == null) {
        try rep.emit(.{
            .id = "gravity/no-accel-columns",
            .severity = .warn,
            .summary = "Could not find accelerometer columns (ax/ay/az or linear_acceleration.*).",
        });
        return;
    }

    var sx: f64 = 0;
    var sy: f64 = 0;
    var sz: f64 = 0;
    var n: usize = 0;
    for (tbl.rows) |row| {
        const x = row[cx.?];
        const y = row[cy.?];
        const z = row[cz.?];
        if (std.math.isNan(x) or std.math.isNan(y) or std.math.isNan(z)) continue;
        sx += x;
        sy += y;
        sz += z;
        n += 1;
    }
    if (n == 0) {
        try rep.emit(.{
            .id = "gravity/no-samples",
            .severity = .warn,
            .summary = "No numeric accelerometer samples found.",
        });
        return;
    }

    const mx = sx / @as(f64, @floatFromInt(n));
    const my = sy / @as(f64, @floatFromInt(n));
    const mz = sz / @as(f64, @floatFromInt(n));
    const mag = @sqrt(mx * mx + my * my + mz * mz);

    // Dominant axis.
    const ax = @abs(mx);
    const ay = @abs(my);
    const az = @abs(mz);
    var dom: u8 = 'z';
    var dom_val = mz;
    if (ax >= ay and ax >= az) {
        dom = 'x';
        dom_val = mx;
    } else if (ay >= ax and ay >= az) {
        dom = 'y';
        dom_val = my;
    }

    const detail = try std.fmt.allocPrint(gpa, "mean specific force = ({d:.3}, {d:.3}, {d:.3}) m/s^2, |a| = {d:.3} over {d} samples.", .{ mx, my, mz, mag, n });

    // --- Direction ---
    if (dom != 'z') {
        try rep.emit(.{
            .id = "gravity/wrong-axis",
            .severity = .fail,
            .summary = try std.fmt.allocPrint(gpa, "Gravity dominant on body {c}-axis, expected Z (up).", .{dom}),
            .detail = detail,
            .causes = &.{
                .{ .text = "IMU is physically mounted on its side (90 deg rotation).", .confidence = "high", .confirm = "Compare to the imu_link orientation in the URDF.", .fix = "Correct the imu_joint rpy or the driver axis mapping." },
                .{ .text = "Body convention mismatch (e.g. FRD vs FLU swaps which axis is up).", .confidence = "medium" },
            },
        });
    } else if (dom_val < 0) {
        try rep.emit(.{
            .id = "gravity/inverted",
            .severity = .fail,
            .summary = "Gravity points DOWN the body Z-axis (negative). Expected +Z for ENU/FLU at rest.",
            .detail = detail,
            .causes = &.{
                .{ .text = "IMU mounted upside down.", .confidence = "high", .confirm = "Check imu_joint rpy in the URDF (a roll of pi flips Z).", .fix = "Add/correct the 180 deg roll, or fix the driver." },
                .{ .text = "Accelerometer sign / NED-vs-ENU convention inverted in the driver.", .confidence = "medium" },
            },
        });
    } else {
        try rep.emit(.{
            .id = "gravity/direction",
            .severity = .pass,
            .summary = "Gravity points up the body Z-axis (+Z), consistent with ENU/FLU at rest.",
            .detail = detail,
        });
    }

    // --- Magnitude vs profile ---
    if (profile) |p| {
        if (p.gravity_magnitude) |g| {
            const err = @abs(mag - g);
            if (err > p.gravity_tol) {
                try rep.emit(.{
                    .id = "gravity/magnitude",
                    .severity = .warn,
                    .summary = try std.fmt.allocPrint(gpa, "|a| = {d:.3} differs from profile gravity {d:.3} by {d:.3} (tol {d:.3}).", .{ mag, g, err, p.gravity_tol }),
                    .causes = &.{
                        .{ .text = "Accelerometer scale/bias not calibrated.", .confidence = "medium", .confirm = "Run a full IMU calibration; compare to local g.", .fix = "Update imu.gravity_mps2 / accel bias after calibration." },
                        .{ .text = "Robot was not actually stationary during capture.", .confidence = "low" },
                    },
                });
            } else {
                try rep.emit(.{
                    .id = "gravity/magnitude",
                    .severity = .pass,
                    .summary = try std.fmt.allocPrint(gpa, "|a| = {d:.3} within {d:.3} of profile gravity {d:.3}.", .{ mag, p.gravity_tol, g }),
                });
            }
        }
    }
}
