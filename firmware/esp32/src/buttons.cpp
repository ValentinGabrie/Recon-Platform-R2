// =============================================================================
// buttons.cpp — debounce + long-press state machine.
//
// Standard "wait for raw input to stay stable for BUTTON_DEBOUNCE_MS before
// believing it" pattern.  Long-press fires once after BUTTON_LONGPRESS_MS
// of continuous press; we then suppress further long-press events until
// the next release.
// =============================================================================
#include "buttons.h"

#include "config.h"

void Button::begin(uint8_t id, int pin)
{
    id_  = id;
    pin_ = pin;
    pinMode(pin_, INPUT_PULLUP);

    stable_pressed_   = false;
    last_raw_pressed_ = false;
    last_change_ms_   = 0;
    press_start_ms_   = 0;
    longpress_fired_  = false;
}

void Button::update(void (*on_event)(uint8_t id, uint8_t state))
{
    const uint32_t now = millis();
    const bool raw_pressed = (digitalRead(pin_) == LOW);  // active-low

    if (raw_pressed != last_raw_pressed_) {
        last_raw_pressed_ = raw_pressed;
        last_change_ms_   = now;
        return;
    }

    if (raw_pressed != stable_pressed_ &&
        (now - last_change_ms_) >= BUTTON_DEBOUNCE_MS)
    {
        stable_pressed_ = raw_pressed;
        if (stable_pressed_) {
            press_start_ms_  = now;
            longpress_fired_ = false;
            on_event(id_, BTN_PRESSED);
        } else {
            on_event(id_, BTN_RELEASED);
        }
        return;
    }

    if (stable_pressed_ && !longpress_fired_ &&
        (now - press_start_ms_) >= BUTTON_LONGPRESS_MS)
    {
        longpress_fired_ = true;
        on_event(id_, BTN_LONGPRESS);
    }
}
