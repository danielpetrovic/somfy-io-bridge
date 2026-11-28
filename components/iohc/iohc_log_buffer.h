/*
   Portions of this file are ported from laberning/home_io_control
   (https://github.com/laberning/home_io_control), MIT License,
   Copyright (c) 2026-present Lars Berning and other contributors.
   See LICENSES/MIT-home_io_control.txt.
 */

#pragma once

// Minimal stand-in for upstream's log_buffer.h (which backs an on-device web
// UI log view we're not vendoring for Phase 0/1). iohcRadio.cpp calls
// addLogMessage() unconditionally on every receive - route it to ESPHome's
// own logger instead of maintaining a separate buffer.
//
// The frame log hold of laberning/home_io_control (log_frame.h,
// FrameLogBuffer): printing a log line takes milliseconds, and a motor drops
// the bridge's answer to its challenge when it comes that much later. Every
// caller that transmits and then waits for the reply it provoked holds the
// frame log from before the transmit until its wait has ended. While held, a
// frame-log line is kept instead of printed; the outermost release, or a
// flush, prints the kept lines in order, and a note says how many did not
// fit. Holds nest. The lines come from the radio's own tasks here, so the
// buffer is locked.

#include <Arduino.h>
#include <cstdint>
#include <vector>

void addLogMessage(const String &msg);
std::vector<String> getLogMessages();

enum class FrameLogLevel : uint8_t { WARN, INFO, DEBUG, VERBOSE };

// Prints one frame-log line, or keeps it while the frame log is held. `tag`
// must outlive the hold (a logging tag constant).
void frame_log_line(FrameLogLevel level, const char *tag, const char *text);
void frame_log_hold();
void frame_log_release();
// Prints the kept lines now and keeps holding.
void frame_log_flush();
