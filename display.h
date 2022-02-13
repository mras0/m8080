#ifndef DISPLAY_H
#define DISPLAY_H

#include <memory>
#include <stdint.h>
#include <vector>

// These match virtual keyboard codes
constexpr uint8_t event_quit  = 0x1b; // Esc
constexpr uint8_t event_left  = 0x25;
constexpr uint8_t event_up    = 0x26;
constexpr uint8_t event_right = 0x27;
constexpr uint8_t event_down  = 0x28;

constexpr uint8_t event_keyup_mask = 0x80;

class display {
public:
    explicit display(int w, int h, int xscale=1, int yscale=1);
    ~display();

    bool show(const uint32_t* data);

    std::vector<uint8_t> events();

private:
    class impl;
    std::unique_ptr<impl> impl_;
};

#endif
