//! CSV reader for sensor logs. Columns are matched by fuzzy header name.
const std = @import("std");
const Allocator = std.mem.Allocator;

pub const Table = struct {
    headers: [][]const u8,
    rows: [][]f64, // rows[r][c]; non-numeric cells become NaN
    gpa: Allocator,

    /// Find a column whose header matches any of `candidates` (case-insensitive,
    /// substring). Returns the column index or null.
    pub fn col(self: Table, candidates: []const []const u8) ?usize {
        for (self.headers, 0..) |h, i| {
            for (candidates) |c| {
                if (containsIgnoreCase(h, c)) return i;
            }
        }
        return null;
    }

    pub fn rowCount(self: Table) usize {
        return self.rows.len;
    }
};

fn containsIgnoreCase(haystack: []const u8, needle: []const u8) bool {
    if (needle.len > haystack.len) return false;
    var i: usize = 0;
    while (i + needle.len <= haystack.len) : (i += 1) {
        if (std.ascii.eqlIgnoreCase(haystack[i .. i + needle.len], needle)) return true;
    }
    return false;
}

fn splitFields(gpa: Allocator, line: []const u8) ![][]const u8 {
    var out: std.ArrayList([]const u8) = .empty;
    var it = std.mem.splitScalar(u8, line, ',');
    while (it.next()) |f| try out.append(gpa, std.mem.trim(u8, f, " \t\r\""));
    return out.toOwnedSlice(gpa);
}

pub fn parse(gpa: Allocator, text: []const u8) !Table {
    var lines = std.mem.splitScalar(u8, text, '\n');
    var headers: [][]const u8 = &.{};
    var rows: std.ArrayList([]f64) = .empty;
    var have_header = false;
    var ncol: usize = 0;

    while (lines.next()) |raw| {
        const line = std.mem.trim(u8, raw, " \t\r");
        if (line.len == 0) continue;
        if (!have_header) {
            headers = try splitFields(gpa, line);
            ncol = headers.len;
            have_header = true;
            continue;
        }
        const fields = try splitFields(gpa, line);
        const vals = try gpa.alloc(f64, ncol);
        for (vals, 0..) |*v, c| {
            v.* = if (c < fields.len)
                (std.fmt.parseFloat(f64, fields[c]) catch std.math.nan(f64))
            else
                std.math.nan(f64);
        }
        try rows.append(gpa, vals);
    }

    return .{
        .headers = headers,
        .rows = try rows.toOwnedSlice(gpa),
        .gpa = gpa,
    };
}

test "parse csv with fuzzy columns" {
    const txt =
        \\t,linear_acceleration.x,linear_acceleration.y,linear_acceleration.z
        \\0.0,0.1,0.0,9.55
        \\0.1,0.0,0.1,9.57
    ;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const tbl = try parse(arena.allocator(), txt);
    try std.testing.expect(tbl.rowCount() == 2);
    const zc = tbl.col(&.{ "acceleration.z", "az", "accel_z" }).?;
    try std.testing.expect(tbl.rows[0][zc] > 9.5);
}
