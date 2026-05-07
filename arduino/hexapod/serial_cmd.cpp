#include <Arduino.h>
#include "serial_cmd.h"
#include "gait.h"
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// serial_cmd.cpp  —  Serial command handler
//
// F() macro wraps string literals so they are stored in Flash (PROGMEM) and
// not copied into SRAM at startup — saves ~1 B per character on the Uno.
// ─────────────────────────────────────────────────────────────────────────────

// Speed limits: 0.001 = ~17-minute cycle, 0.1 = ~0.17-second cycle at 60 Hz
constexpr float SPEED_MIN  = 0.001f;
constexpr float SPEED_MAX  = 0.100f;
constexpr float SPEED_STEP = 0.002f;

static const char* gait_name(uint8_t pattern) {
    switch (pattern) {
        case GaitPattern::TRIPOD: return "TRIPOD";
        case GaitPattern::WAVE:   return "WAVE";
        case GaitPattern::RIPPLE: return "RIPPLE";
        default:                  return "UNKNOWN";
    }
}

void serial_cmd_poll(GaitConfig* cfg, Leg legs[], uint8_t num_legs) {
    while (Serial.available()) {
        char c = (char)Serial.read();

        switch (c) {
            // ── Gait on/off ───────────────────────────────────────────────────
            case 'S':
                cfg->running = true;
                Serial.println(F("START"));
                break;

            case 'X':
                cfg->running = false;
                Serial.println(F("STOP"));
                break;

            // ── Direction ─────────────────────────────────────────────────────
            case 'F':
                cfg->step_length = STEP_LENGTH;
                Serial.println(F("DIR: forward"));
                break;

            case 'B':
                cfg->step_length = -STEP_LENGTH;
                Serial.println(F("DIR: backward"));
                break;

            // ── Gait patterns ─────────────────────────────────────────────────
            case 'T':
                gait_set_pattern(legs, num_legs, cfg, GaitPattern::TRIPOD);
                Serial.println(F("GAIT: TRIPOD"));
                break;

            case 'W':
                gait_set_pattern(legs, num_legs, cfg, GaitPattern::WAVE);
                Serial.println(F("GAIT: WAVE"));
                break;

            case 'R':
                gait_set_pattern(legs, num_legs, cfg, GaitPattern::RIPPLE);
                Serial.println(F("GAIT: RIPPLE"));
                break;

            // ── Speed control ─────────────────────────────────────────────────
            case '+':
                cfg->speed += SPEED_STEP;
                if (cfg->speed > SPEED_MAX) cfg->speed = SPEED_MAX;
                Serial.print(F("SPEED: "));
                Serial.println(cfg->speed, 4);
                break;

            case '-':
                cfg->speed -= SPEED_STEP;
                if (cfg->speed < SPEED_MIN) cfg->speed = SPEED_MIN;
                Serial.print(F("SPEED: "));
                Serial.println(cfg->speed, 4);
                break;

            // ── Status query ──────────────────────────────────────────────────
            case '?':
                Serial.print(F("pattern="));
                Serial.print(gait_name(cfg->pattern));
                Serial.print(F("  speed="));
                Serial.print(cfg->speed, 4);
                Serial.print(F("  dir="));
                Serial.print(cfg->step_length >= 0 ? F("fwd") : F("bwd"));
                Serial.print(F("  running="));
                Serial.println(cfg->running ? F("yes") : F("no"));
                break;

            // ignore whitespace / newlines from terminal
            case '\r':
            case '\n':
            case ' ':
                break;

            default:
                Serial.print(F("? unknown command: "));
                Serial.println(c);
                break;
        }
    }
}
