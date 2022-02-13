#include <iostream>
#include <signal.h>
#include "ioutil.h"
#include "m8080.h"
#include "cpm.h"

void disassemble_file(const char* filename, uint16_t load_offset)
{
    m8080_mem_bus mem_bus { 65536 };
    const auto data = read_file(filename);
    assert(data.size() < 65536 - load_offset);
    memcpy(&mem_bus.mem()[load_offset], data.data(), data.size());
    for (uint16_t pc = load_offset; pc < load_offset + data.size();) {
        pc = disasm_one(std::cout, mem_bus, pc);
    }
}

void run_cpm_tests()
{
    run_cpm_test("../misc/cputest/8080PRE.COM");
    run_cpm_test("../misc/cputest/TST8080.COM");
    run_cpm_test("../misc/cputest/8080EXM.COM");
    run_cpm_test("../misc/cputest/CPUTEST.COM");
}


bool ctrl_c_pressed;

void ctrl_c_handler(int)
{
    ctrl_c_pressed = true;
}

void install_signal_handler(void)
{
    ctrl_c_pressed = false;
    signal(SIGINT, &ctrl_c_handler);
}

// https://computerarcheology.com/Arcade/SpaceInvaders/Hardware.html
//<chip type="cpu" tag="maincpu" name="Intel 8080" clock="1996800"/>
//<display tag="screen" type="raster" rotate="270" width="260" height="224" refresh="59.541985" pixclock="4992000" htotal="320" hbend="0" hbstart="260" vtotal="262" vbend="0" vbstart="224"/>

class invaders final : public m8080_bus {
public:
    explicit invaders(const char* romfile)
        : cpu_ { *this }
        , rom_ { read_file(romfile) }
        , ram_(0x400)
        , video_ram_(7 * 1024)
    {
        assert(rom_.size() == ram_base);
        install_signal_handler();
    }

    uint8_t peek8(uint16_t addr) override
    {
        if (addr < ram_base)
            return rom_[addr];

        if (addr >= video_ram_base && addr < video_ram_end)
            return video_ram_[addr - video_ram_base];

        // TODO: Check if this is actually used
        if (addr >= video_ram_end) {
            throw std::runtime_error { "TODO: peek8(" + hexstring(addr) + ") PC " + hexstring(cpu_.state().pc) };
        }

        return ram_[addr & (ram_size - 1)];
    }

    void poke8(uint16_t addr, uint8_t val)
    {
        if (addr < ram_base)
            throw std::runtime_error { "Write to rom address 0" + hexstring(addr) + "H value 0" + hexstring(val) + "H PC " + hexstring(cpu_.state().pc) };

        if (addr >= video_ram_base && addr < video_ram_end) {
            video_ram_[addr - video_ram_base] = val;
            return;
        }

        // TODO: Check if this is actually used
        if (addr >= video_ram_end) {
            //throw std::runtime_error { "TODO: poke8(" + hexstring(addr) + ", " + hexstring(val) + ") PC " + hexstring(cpu_.state().pc) };
            std::cerr << "Write to RAM mirror addr=" << hex(addr) << " val=" << hex(val) << " pc = " << hex(cpu_.state().pc) << "\n";
            return;
        }

        ram_[addr & (ram_size - 1)] = val;
    }

    uint8_t input(uint8_t port)
    {
        switch (port) {
        case 0:
            return 0b00001110;
        case 1:
        case 2:
            return 0;
        case 3:
            return static_cast<uint8_t>((shift_register_ << shift_amount_) >> 8);
        }

        throw std::runtime_error { "TODO: Input from port 0" + hexstring(port) + "H PC " + hexstring(cpu_.state().pc) };
    }

    int interrupt() override
    {
        return interrupt_inst_;
    }

    void inte(bool new_state) override
    {
        if (new_state)
            interrupt_inst_ = -1;
    }

    void output(uint8_t port, uint8_t value)
    {
        switch (port) {
        case 2: // shift amount (3 bits)
            assert(shift_amount_ >> 3 == 0);
            shift_amount_ = value & 7;
            //std::cout << "Shift amount = 0" << hex(value) << "H\n";
            return;
        case 4: // shift value
            shift_register_ = value << 8 | shift_register_ >> 8;
            //std::cout << "Shift value = 0" << hex(value) << "H register = 0" << hex(shift_register_) << "H\n";
            return;
        case 3: // Sound bits
        case 5: // Sound bits
            return;
        case 6: // Watchdog
            return;
        }
        std::cout << "PC=" << hex(cpu_.state().pc) << "\n";
        throw std::runtime_error { "TODO: Output to port 0" + hexstring(port) + "H value 0" + hexstring(value) + "H PC " + hexstring(cpu_.state().pc) };
    }

    void run()
    {
        install_signal_handler();
        debug_prompt();

        uint8_t rem_pixel_cycle = 0;
        while (!quit_) {
            try {
                const auto cpu_cycles = cpu_.step();
                static_assert(5 * cpu_frequency_hz == 2 * pixel_frequency_hz);
                auto pixel_clocks = cpu_cycles * 5 / 2;
                rem_pixel_cycle += cpu_cycles % 2;
                assert(rem_pixel_cycle <= 2);
                if (rem_pixel_cycle == 2) {
                    rem_pixel_cycle = 0;
                    ++pixel_clocks;
                }
                assert(pixel_clocks < 256);
                update_video(static_cast<uint8_t>(pixel_clocks));

                if (auto it = std::find(break_points_.begin(), break_points_.end(), cpu_.state().pc); it != break_points_.end()) {
                    std::cout << "Break point " << it - break_points_.begin() << " hit\n";
                    single_stepping_ = true;
                }
                if (cpu_.state().pc == step_to_) {
                    single_stepping_ = true;
                    step_to_ = -1;
                }

                if (ctrl_c_pressed) {
                    debug_prompt();
                    install_signal_handler();
                } else if (single_stepping_) {
                    debug_prompt();
                }
            } catch (const std::exception& e) {
                std::cout << e.what() << "\n";
                debug_prompt();
            }
        }
    }

private:
    m8080 cpu_;
    const std::vector<uint8_t> rom_;
    std::vector<uint8_t> ram_; // 1K RAM
    std::vector<uint8_t> video_ram_; // 7K Video RAM
    bool quit_ = false;
    
    // Debugger
    bool single_stepping_ = false;
    std::vector<uint16_t> break_points_;
    int step_to_ = -1;

    // Shift register
    uint8_t shift_amount_ = 0;
    uint16_t shift_register_ = 0;

    // Video
    uint16_t vpos_ = 0;
    uint16_t hpos_ = 0;
    int interrupt_inst_ = -1;

    // Memory map
    // 0000-1FFF 8K ROM
    // 2000-23FF 1K RAM
    // 2400-3FFF 7K Video RAM
    // 4000-FFFF RAM mirror
    static constexpr uint16_t ram_size = 0x0400;
    static constexpr uint16_t ram_base = 0x2000;
    static constexpr uint16_t video_ram_size = 0x1c00;
    static constexpr uint16_t video_ram_base = 0x2400;
    static constexpr uint16_t video_ram_end = video_ram_base + video_ram_size;
    static_assert(video_ram_end == 0x4000);


    static constexpr uint32_t cpu_frequency_hz = 1996800;
    static constexpr uint32_t pixel_frequency_hz = 4992000;    

    // Screen dimensions
    static constexpr uint16_t htotal = 320;
    static constexpr uint16_t vtotal = 262;
    // Visual area
    static constexpr uint16_t width = 260;
    static constexpr uint16_t height = 224;
    //<display tag="screen" type="raster" rotate="270" width="260" height="224" refresh="59.541985" pixclock="4992000" htotal="320" hbend="0" hbstart="260" vtotal="262" vbend="0" vbstart="224"/>

    void debug_prompt();

    void update_video(uint8_t pixel_clocks);
};

void invaders::debug_prompt()
{
    single_stepping_ = false;
    cpu_.trace(nullptr);

    uint16_t disasm_pos = cpu_.state().pc;
    uint16_t hexdump_pos = 0;

    std::cout << "VPOS=" << hex(vpos_) << " HPOS=" << hex(hpos_) << " INT: " << (interrupt_inst_ == -1 ? 0 : interrupt_inst_ == 0xcf ? 1 : 2) << "\n";
    std::cout << cpu_.state() << "\n";
    disasm_one(std::cerr, *this, cpu_.state().pc);

    for (std::string line;;) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line))
            return;
        if (line.empty())
            continue;
        if (line == "q") {
            quit_ = true;
            return;
        } else if (line[0] == 'g') {
            if (line.length() > 1 && line[1] == 't')
                cpu_.trace(&std::cout);
            return;
        } else if (line == "t") {
            single_stepping_ = true;
            return;
        } else if (line == "z") {
            const auto pc = cpu_.state().pc;
            step_to_ = pc + instruction_length(peek8(pc));
            if (line.length() > 1 && line[1] == 't')
                cpu_.trace(&std::cout);
            return;
        } else if (line == "fd") {
            std::cout << "Cleared " << break_points_.size() << " breakpoints\n";
            break_points_.clear();
        } else if (line[0] == 'f') {
            if (std::optional<uint16_t> num {}; line.length() > 2 && line[1] == ' ' && (num = read_number(&line[2]))) {
                std::cout << "Adding breakpoint " << break_points_.size() << " 0" << hex(*num) << "H\n";
                break_points_.push_back(*num);
            } else {
                std::cout << "Invalid or missing arguments\n";
            }
        } else if (line[0] == 'd') {
            if (std::optional<uint16_t> num {}; line.length() > 2 && line[1] == ' ' && (num = read_number(&line[2]))) {
                disasm_pos = *num;
            }
            for (int i = 0; i < 10; ++i)
                disasm_pos = disasm_one(std::cout, *this, disasm_pos);
        } else if (line[0] == 'm') {
            if (std::optional<uint16_t> num {}; line.length() > 2 && line[1] == ' ' && (num = read_number(&line[2]))) {
                hexdump_pos = *num;
            }
            for (int i = 0; i < 10; ++i) {
                uint8_t temp[16];
                for (int j = 0; j < 16; ++j)
                    temp[j] = peek8(static_cast<uint16_t>(hexdump_pos + j));
                hexdump(std::cout, temp, sizeof(temp), hexdump_pos);
                hexdump_pos += 16;
            }
        } else {
            std::cout << "Unknown command \"" << line << "\"\n";
        }
    }
}

void invaders::update_video(uint8_t pixel_clocks)
{
    hpos_ += pixel_clocks;
    if (hpos_ >= htotal) {
        hpos_ -= htotal;
        assert(hpos_ < htotal);
        if (++vpos_ == vtotal) {
            vpos_ = 0;
        }
        // 0xc7 | (64V << 4) | (!64V << 3)
        if (vpos_ == 96)
            interrupt_inst_ = 0xCF;
        else if (vpos_ == 224)
            interrupt_inst_ = 0xD7;
    }
}

void run_invaders(const char* rom_filename)
{
    invaders inv { rom_filename };
    inv.run();
}

int main()
{
    try {
        //run_cpm_tests();
        const char* const rom_filename = "../misc/invaders/invaders.rom";
        run_invaders(rom_filename);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
