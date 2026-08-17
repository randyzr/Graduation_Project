#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>

class Elecrow7InchHMI : public lgfx::LGFX_Device {
 public:
  lgfx::Bus_RGB rgbBus;
  lgfx::Panel_RGB panel;
  lgfx::Light_PWM backlight;

  Elecrow7InchHMI() {
    {
      auto cfg = panel.config();
      cfg.memory_width = 800;
      cfg.memory_height = 480;
      cfg.panel_width = 800;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      panel.config(cfg);
    }

    {
      auto cfg = panel.config_detail();
      cfg.use_psram = true;
      panel.config_detail(cfg);
    }

    {
      auto cfg = rgbBus.config();
      cfg.panel = &panel;

      cfg.pin_d0 = GPIO_NUM_15;
      cfg.pin_d1 = GPIO_NUM_7;
      cfg.pin_d2 = GPIO_NUM_6;
      cfg.pin_d3 = GPIO_NUM_5;
      cfg.pin_d4 = GPIO_NUM_4;
      cfg.pin_d5 = GPIO_NUM_9;
      cfg.pin_d6 = GPIO_NUM_46;
      cfg.pin_d7 = GPIO_NUM_3;
      cfg.pin_d8 = GPIO_NUM_8;
      cfg.pin_d9 = GPIO_NUM_16;
      cfg.pin_d10 = GPIO_NUM_1;
      cfg.pin_d11 = GPIO_NUM_14;
      cfg.pin_d12 = GPIO_NUM_21;
      cfg.pin_d13 = GPIO_NUM_47;
      cfg.pin_d14 = GPIO_NUM_48;
      cfg.pin_d15 = GPIO_NUM_45;

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync = GPIO_NUM_40;
      cfg.pin_hsync = GPIO_NUM_39;
      cfg.pin_pclk = GPIO_NUM_0;
      cfg.freq_write = 15000000;

      cfg.hsync_polarity = 0;
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch = 40;
      cfg.vsync_polarity = 0;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch = 13;
      cfg.pclk_active_neg = 1;
      cfg.de_idle_high = 0;
      cfg.pclk_idle_high = 0;

      rgbBus.config(cfg);
      panel.setBus(&rgbBus);
    }

    {
      auto cfg = backlight.config();
      cfg.pin_bl = GPIO_NUM_2;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      backlight.config(cfg);
      panel.light(&backlight);
    }

    setPanel(&panel);
  }
};
