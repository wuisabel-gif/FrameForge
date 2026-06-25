//! Finding types and the human-readable reporter.
//!
//! Every check emits Findings rather than printing directly, so the diagnostic
//! voice (severity + ranked, evidence-backed causes) is consistent and the same
//! data can later be rendered as JSON for CI.
const std = @import("std");
const Writer = std.Io.Writer;

pub const Severity = enum {
    fail,
    warn,
    info,
    pass,

    pub fn tag(self: Severity) []const u8 {
        return switch (self) {
            .fail => "FAIL",
            .warn => "WARN",
            .info => "INFO",
            .pass => "PASS",
        };
    }

    pub fn color(self: Severity) []const u8 {
        return switch (self) {
            .fail => "\x1b[1;31m", // bold red
            .warn => "\x1b[1;33m", // bold yellow
            .info => "\x1b[1;36m", // bold cyan
            .pass => "\x1b[1;32m", // bold green
        };
    }
};

/// One ranked, evidence-backed hypothesis for why a finding occurred.
pub const Cause = struct {
    text: []const u8,
    confidence: []const u8 = "medium", // high | medium | low
    confirm: []const u8 = "", // how to confirm
    fix: []const u8 = "", // how to fix
};

pub const Finding = struct {
    id: []const u8,
    severity: Severity,
    summary: []const u8,
    detail: []const u8 = "",
    causes: []const Cause = &.{},
};

/// Accumulates findings and renders them. Owns no output buffer; the caller
/// flushes the underlying file writer.
pub const Reporter = struct {
    w: *Writer,
    use_color: bool,
    n_fail: usize = 0,
    n_warn: usize = 0,
    n_pass: usize = 0,
    n_info: usize = 0,

    pub fn init(w: *Writer, use_color: bool) Reporter {
        return .{ .w = w, .use_color = use_color };
    }

    fn paint(self: *Reporter, code: []const u8) !void {
        if (self.use_color) try self.w.writeAll(code);
    }

    fn reset(self: *Reporter) !void {
        if (self.use_color) try self.w.writeAll("\x1b[0m");
    }

    pub fn emit(self: *Reporter, f: Finding) !void {
        switch (f.severity) {
            .fail => self.n_fail += 1,
            .warn => self.n_warn += 1,
            .pass => self.n_pass += 1,
            .info => self.n_info += 1,
        }

        try self.w.writeAll("\n");
        try self.paint(f.severity.color());
        try self.w.print("{s}", .{f.severity.tag()});
        try self.reset();
        try self.w.print("  [{s}]  {s}\n", .{ f.id, f.summary });

        if (f.detail.len > 0) {
            var it = std.mem.splitScalar(u8, f.detail, '\n');
            while (it.next()) |line| try self.w.print("      {s}\n", .{line});
        }

        if (f.causes.len > 0) {
            try self.w.writeAll("      Ranked causes:\n");
            for (f.causes, 1..) |c, i| {
                try self.w.print("       {d}. ({s}) {s}\n", .{ i, c.confidence, c.text });
                if (c.confirm.len > 0) try self.w.print("          confirm: {s}\n", .{c.confirm});
                if (c.fix.len > 0) try self.w.print("          fix:     {s}\n", .{c.fix});
            }
        }
    }

    /// Print the trailing summary line and return the process exit code
    /// (non-zero when any FAIL was emitted).
    pub fn finish(self: *Reporter, fail_on_warn: bool) !u8 {
        try self.w.writeAll("\n");
        try self.w.writeAll("-------------------------------------------------------------\n");
        if (self.n_fail == 0 and (!fail_on_warn or self.n_warn == 0)) {
            try self.paint(Severity.pass.color());
            try self.w.writeAll("PASS");
            try self.reset();
        } else {
            try self.paint(Severity.fail.color());
            try self.w.writeAll("FAIL");
            try self.reset();
        }
        try self.w.print(
            "  {d} pass, {d} info, {d} warn, {d} fail\n",
            .{ self.n_pass, self.n_info, self.n_warn, self.n_fail },
        );
        if (self.n_fail > 0) return 1;
        if (fail_on_warn and self.n_warn > 0) return 2;
        return 0;
    }
};
