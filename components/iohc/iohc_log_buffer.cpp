#include "iohc_log_buffer.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdio>

static const char *const TAG = "iohc.upstream";

namespace {

// Lines one hold keeps (FRAME_LOG_HELD_LINES): that component keeps 12, one
// line per frame; this bridge writes two per frame (the packet decode and the
// radio's own line), so twice as many.
constexpr uint8_t FRAME_LOG_HELD_LINES = 24;
// Characters one kept line holds, terminator included (HELD_LOG_LINE_SIZE).
constexpr size_t HELD_LOG_LINE_SIZE = 320;

struct HeldLine {
  FrameLogLevel level;
  const char *tag;
  char text[HELD_LOG_LINE_SIZE];
};

HeldLine held_lines[FRAME_LOG_HELD_LINES];
uint8_t held_count = 0;
uint8_t dropped_count = 0;
uint8_t hold_depth = 0;

SemaphoreHandle_t lock_handle() {
  static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
  return mutex;
}

void print_line(FrameLogLevel level, const char *tag, const char *text) {
  switch (level) {
    case FrameLogLevel::WARN:
      ESP_LOGW(tag, "%s", text);
      break;
    case FrameLogLevel::INFO:
      ESP_LOGI(tag, "%s", text);
      break;
    case FrameLogLevel::DEBUG:
      ESP_LOGD(tag, "%s", text);
      break;
    case FrameLogLevel::VERBOSE:
      ESP_LOGV(tag, "%s", text);
      break;
  }
}

// Prints the kept lines in order, one at a time outside the lock, so a
// radio task that writes meanwhile never waits for a print. Only the main
// loop flushes.
void flush_lines() {
  uint8_t i = 0;
  for (;;) {
    HeldLine line;
    xSemaphoreTake(lock_handle(), portMAX_DELAY);
    if (i >= held_count) {
      held_count = 0;
      const uint8_t dropped = dropped_count;
      dropped_count = 0;
      xSemaphoreGive(lock_handle());
      if (dropped > 0)
        ESP_LOGW(TAG, "%u frame-log lines not printed (hold buffer full)", dropped);
      return;
    }
    line = held_lines[i++];
    xSemaphoreGive(lock_handle());
    print_line(line.level, line.tag, line.text);
  }
}

}  // namespace

void frame_log_line(FrameLogLevel level, const char *tag, const char *text) {
  xSemaphoreTake(lock_handle(), portMAX_DELAY);
  if (hold_depth == 0) {
    xSemaphoreGive(lock_handle());
    print_line(level, tag, text);
    return;
  }
  if (held_count >= FRAME_LOG_HELD_LINES) {
    if (dropped_count < UINT8_MAX)
      dropped_count++;
  } else {
    HeldLine &line = held_lines[held_count++];
    line.level = level;
    line.tag = tag;
    snprintf(line.text, sizeof(line.text), "%s", text);
  }
  xSemaphoreGive(lock_handle());
}

void frame_log_hold() {
  xSemaphoreTake(lock_handle(), portMAX_DELAY);
  if (hold_depth < UINT8_MAX)
    hold_depth++;
  xSemaphoreGive(lock_handle());
}

void frame_log_release() {
  xSemaphoreTake(lock_handle(), portMAX_DELAY);
  if (hold_depth > 0)
    hold_depth--;
  const bool outermost = hold_depth == 0;
  xSemaphoreGive(lock_handle());
  if (outermost)
    flush_lines();
}

void frame_log_flush() { flush_lines(); }

void addLogMessage(const String &msg) { frame_log_line(FrameLogLevel::DEBUG, TAG, msg.c_str()); }

std::vector<String> getLogMessages() { return {}; }
