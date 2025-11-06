/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

#include "pico/stdlib.h"

// HID Consumer Control usage codes (from USB HID Usage Tables spec)
#ifndef HID_USAGE_CONSUMER_VOLUME_INCREMENT
#define HID_USAGE_CONSUMER_VOLUME_INCREMENT  0x00E9
#endif
#ifndef HID_USAGE_CONSUMER_VOLUME_DECREMENT
#define HID_USAGE_CONSUMER_VOLUME_DECREMENT  0x00EA
#endif

#define LEFT_BTN   2
#define RIGHT_BTN  16
#define MIDDLE_BTN 28

#define X1 1
#define X2 0
#define Y1 18
#define Y2 17

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

int8_t delta_x = 0;
int8_t delta_y = 0;

// Button state variables
static uint8_t button_state = 0;  // Bitmask for button states (bit 0: left, bit 1: right, bit 2: middle)

// Previous states for quadrature encoder
static uint8_t prev_x_state = 0;
static uint8_t prev_y_state = 0;

// Debouncing timestamps
static uint32_t last_x_interrupt = 0;
static uint32_t last_y_interrupt = 0;
const uint32_t DEBOUNCE_US = 5; // 5us debounce time

// Acceleration variables
static int8_t last_x_direction = 0;  // -1, 0, or 1
static int8_t last_y_direction = 0;  // -1, 0, or 1
static int8_t last_diagonal_direction = 0;  // 0=none, 1=up-right, 2=up-left, 3=down-right, 4=down-left
static float accel_counter = 1.0f;  // Single acceleration counter for all movement
static float x_accel_counter = 1.0f;  // Start at 1.0 for base speed
static float y_accel_counter = 1.0f;  // Start at 1.0 for base speed
const float MAX_ACCEL_COUNTER = 20.0f;   // Maximum acceleration level
const float ACCEL_INCREMENT = 0.0125f;    // How much to increase per movement
const float ACCEL_DECAY_RATE = 0.95f;   // Decay factor when no movement

void hid_task(void);

// Interrupt callback for quadrature encoder
void irq_callback(uint gpio, uint32_t events);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  // Initialize LED for status
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  gpio_put(25, true);  // Turn on LED to show device is running

  // Initialize button pins
  gpio_init(LEFT_BTN);
  gpio_set_dir(LEFT_BTN, GPIO_IN);
  gpio_pull_up(LEFT_BTN);

  gpio_init(RIGHT_BTN);
  gpio_set_dir(RIGHT_BTN, GPIO_IN);
  gpio_pull_up(RIGHT_BTN);

  gpio_init(MIDDLE_BTN);
  gpio_set_dir(MIDDLE_BTN, GPIO_IN);
  gpio_pull_up(MIDDLE_BTN);

  // Initialize quadrature encoder pins with interrupts
  gpio_init(X1);
  gpio_set_dir(X1, GPIO_IN);
  gpio_pull_down(X1);
  gpio_set_irq_enabled(X1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_init(X2);
  gpio_set_dir(X2, GPIO_IN);
  gpio_pull_down(X2);
  gpio_set_irq_enabled(X2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_init(Y1);
  gpio_set_dir(Y1, GPIO_IN);
  gpio_pull_down(Y1);
  gpio_set_irq_enabled(Y1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_init(Y2);
  gpio_set_dir(Y2, GPIO_IN);
  gpio_pull_down(Y2);
  gpio_set_irq_enabled(Y2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  // Set up interrupt handler
  gpio_set_irq_callback(irq_callback);
  irq_set_enabled(IO_IRQ_BANK0, true);

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    hid_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

void hid_task(void)
{
  // Send reports every 10ms to avoid flooding USB
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if (board_millis() - start_ms >= interval_ms)
  {
    start_ms += interval_ms;

    // Read button states (active low due to pull-ups)
    static bool prev_left_btn = true;   // true = gpio high = button not pressed
    static bool prev_right_btn = true;  // true = gpio high = button not pressed
    static bool volume_key_sent = false;
    bool left_btn = gpio_get(LEFT_BTN);
    bool right_btn = gpio_get(RIGHT_BTN);
    bool middle_btn = !gpio_get(MIDDLE_BTN);  // Still use middle button as mouse button

    // Handle volume control with left and right buttons
    uint16_t consumer_key = 0;
    bool button_pressed = false;
    
    // Detect button press (transition from high to low, since active low)
    if (!left_btn && prev_left_btn) {
      // Left button pressed - volume down
      consumer_key = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
      button_pressed = true;
    }
    else if (!right_btn && prev_right_btn) {
      // Right button pressed - volume up
      consumer_key = HID_USAGE_CONSUMER_VOLUME_INCREMENT;
      button_pressed = true;
    }
    // Detect button release (both buttons are now high/not pressed)
    else if (left_btn && right_btn && volume_key_sent) {
      consumer_key = 0;
      button_pressed = false;
    }

    prev_left_btn = left_btn;
    prev_right_btn = right_btn;

    // Send consumer control report when key is pressed or released
    if (tud_hid_n_ready(ITF_NUM_CONSUMER))
    {
      if (button_pressed) {
        // Send key press
        tud_hid_n_report(ITF_NUM_CONSUMER, 0, &consumer_key, sizeof(consumer_key));
        volume_key_sent = true;
      }
      else if (!button_pressed && volume_key_sent) {
        // Send key release by sending zero-value consumer_key
        tud_hid_n_report(ITF_NUM_CONSUMER, 0, &consumer_key, sizeof(consumer_key));
        volume_key_sent = false;
      }
    }

    // Apply acceleration to scroll deltas (use smaller values for scroll)
    int8_t scroll_x = 0;
    int8_t scroll_y = 0;

    if (delta_x != 0) {
      // Apply acceleration multiplier for horizontal scroll
      scroll_x = (int8_t)(delta_x * accel_counter);
      // Clamp to smaller range for scroll (typical scroll values are smaller)
      if (scroll_x > 10) scroll_x = 10;
      if (scroll_x < -10) scroll_x = -10;
    }

    if (delta_y != 0) {
      // Apply acceleration multiplier for vertical scroll
      scroll_y = (int8_t)(delta_y * accel_counter);
      // Clamp to smaller range for scroll
      if (scroll_y > 10) scroll_y = 10;
      if (scroll_y < -10) scroll_y = -10;
    }

    // Handle acceleration decay when no movement
    if (delta_x == 0 && delta_y == 0) {
      accel_counter = accel_counter * ACCEL_DECAY_RATE;
      if (accel_counter < 1.0f) accel_counter = 1.0f;
    }

    // mouse interface - send reports with accelerated scroll data
    if (tud_hid_n_ready(ITF_NUM_MOUSE))
    {
      uint8_t const report_id = 0;
      uint8_t buttons = 0;
      if (middle_btn) buttons |= (1 << 2); // Middle button
      // Use scroll_x for vertical scroll (wheel), scroll_y for horizontal scroll (pan)
      tud_hid_n_mouse_report(ITF_NUM_MOUSE, report_id, buttons, 0, 0, -scroll_y, scroll_x);

      // Reset deltas after sending
      delta_x = 0;
      delta_y = 0;
    }
  }
}

// Invoked when received SET_PROTOCOL request
// protocol is either HID_PROTOCOL_BOOT (0) or HID_PROTOCOL_REPORT (1)
void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol)
{
  (void)instance;
  (void)protocol;

  // nothing to do since we use the same compatible boot report for both Boot and Report mode.
  // TOOD set a indicator for user
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
  (void)instance;
  (void)report;
  (void)len;

  // nothing to do
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
  (void)report_id;
}

// Interrupt callback for quadrature encoder
void irq_callback(uint gpio, uint32_t events)
{
  uint32_t now = time_us_32();

  if (gpio == X1 || gpio == X2) {
    // Software debouncing for X axis
    if (now - last_x_interrupt < DEBOUNCE_US) return;
    last_x_interrupt = now;

    // Read current X state
    uint8_t x1 = gpio_get(X1);
    uint8_t x2 = gpio_get(X2);
    uint8_t current_x_state = (x1 << 1) | x2;

    // Improved quadrature decoding for X axis
    if (current_x_state != prev_x_state) {
      // Gray code transitions: 00->01->11->10->00 (clockwise = left)
      //                      00->10->11->01->00 (counter-clockwise = right)
      if ((prev_x_state == 0 && current_x_state == 1) ||
          (prev_x_state == 1 && current_x_state == 3) ||
          (prev_x_state == 3 && current_x_state == 2) ||
          (prev_x_state == 2 && current_x_state == 0)) {
        delta_x -= 1;  // Left (swapped)
        last_x_direction = -1;
      }
      else if ((prev_x_state == 0 && current_x_state == 2) ||
               (prev_x_state == 2 && current_x_state == 3) ||
               (prev_x_state == 3 && current_x_state == 1) ||
               (prev_x_state == 1 && current_x_state == 0)) {
        delta_x += 1;  // Right (swapped)
        last_x_direction = 1;
      }
      prev_x_state = current_x_state;
    }
  }
  else if (gpio == Y1 || gpio == Y2) {
    // Software debouncing for Y axis
    if (now - last_y_interrupt < DEBOUNCE_US) return;
    last_y_interrupt = now;

    // Read current Y state
    uint8_t y1 = gpio_get(Y1);
    uint8_t y2 = gpio_get(Y2);
    uint8_t current_y_state = (y1 << 1) | y2;

    // Improved quadrature decoding for Y axis
    if (current_y_state != prev_y_state) {
      if ((prev_y_state == 0 && current_y_state == 1) ||
          (prev_y_state == 1 && current_y_state == 3) ||
          (prev_y_state == 3 && current_y_state == 2) ||
          (prev_y_state == 2 && current_y_state == 0)) {
        delta_y += 1;  // Down
        last_y_direction = 1;
      }
      else if ((prev_y_state == 0 && current_y_state == 2) ||
               (prev_y_state == 2 && current_y_state == 3) ||
               (prev_y_state == 3 && current_y_state == 1) ||
               (prev_y_state == 1 && current_y_state == 0)) {
        delta_y -= 1;  // Up
        last_y_direction = -1;
      }
      prev_y_state = current_y_state;
    }
  }

  // Handle acceleration for any movement
  if (delta_x != 0 || delta_y != 0) {
    // Increase acceleration for any movement
    accel_counter = (accel_counter < MAX_ACCEL_COUNTER) ? accel_counter + ACCEL_INCREMENT : MAX_ACCEL_COUNTER;
  }
  // Note: Decay logic moved to hid_task() to work when mouse is stopped
}
