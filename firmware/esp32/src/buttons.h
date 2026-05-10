// =============================================================================
// buttons.h — Debounced 3-button handler with long-press detection.
//
// Each Button wraps one INPUT_PULLUP GPIO. update() should be called every
// loop iteration (typically once per ms). The class only emits BTN_PRESSED
// once per physical press, BTN_RELEASED once per release, and BTN_LONGPRESS
// once when the user holds past BUTTON_LONGPRESS_MS.
// =============================================================================
#pragma once

#include <Arduino.h>
#include <stdint.h>

class Button {
public:
    /// `id` is one of the ButtonId enum values from config.h; it's only
    /// stored so handlers know which button fired.
    void begin(uint8_t id, int pin);

    /// Poll the GPIO and update internal state.
    /// `on_event(id, state)` is called at most once per call, where state
    /// is one of BTN_PRESSED / BTN_RELEASED / BTN_LONGPRESS.
    void update(void (*on_event)(uint8_t id, uint8_t state));

private:
    uint8_t  id_;
    int      pin_;
    bool     stable_pressed_;
    bool     last_raw_pressed_;
    uint32_t last_change_ms_;
    uint32_t press_start_ms_;
    bool     longpress_fired_;
};
