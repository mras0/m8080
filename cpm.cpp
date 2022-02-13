#include "cpm.h"
#include "m8080.h"
#include "ioutil.h"
#include <iostream>
#include <chrono>

namespace {

std::string output_buffer;
bool soft_reset = false;

void output_flush()
{
    if (output_buffer.empty())
        return;
    while (!output_buffer.empty() && output_buffer.back() == '\r')
        output_buffer.pop_back();
    std::cout << ">>> " << output_buffer << "\n";
    output_buffer.clear();
}

void output_char(char ch)
{
    if (ch == '\n') {
        output_flush();
        return;
    }
    if (ch < ' ' || ch > 0x7f) {
        if (ch != '\t' && ch != '\r') {
            output_buffer += "\\x" + hexstring(ch, 2);
            return;
        }
    }
    output_buffer += ch;
}

void dos_call(m8080& m)
{
    auto& state = m.state();
    switch (state.c()) {
    case 0: // System reset
        throw std::runtime_error { "System reset" };
    case 2: // Console output
        output_char(state.e());
        break;
    case 9: // Print string
    {
        uint16_t addr = state.de();
        for (unsigned i = 0; i < 65536 && m.peek8(addr) != '$'; ++i, ++addr)
            output_char(m.peek8(addr));
    } break;
    default:
        std::cerr << "TODO: DOSCALL " << static_cast<int>(state.c()) << "\n";
        std::cerr << state << "\n";
        throw std::runtime_error { "TODO: Handle DOSCALL " + std::to_string(state.c()) };
    }
}

} // unnamed namespace

void run_cpm_test(const char* filename)
{
    constexpr uint16_t DOS_ADDR = 0xE400;
    constexpr uint16_t BIOS_ADDR = 0xF200;

    const auto data = read_file(filename);

    soft_reset = false;

    m8080_mem_bus mem { 65536 };
    m8080 m { mem };
    auto& state = m.state();

    std::cout << "--------------------------------------------\n";
    std::cout << "Testing " << filename << "\n";
    std::cout << "--------------------------------------------\n";

    m.add_trap(DOS_ADDR, [&m]() { dos_call(m); });
    m.add_trap(BIOS_ADDR, [&]() { soft_reset = true; });

    constexpr uint8_t HLT_INS = 0b01110110;
    constexpr uint8_t RET_INS = 0b11001001;
    constexpr uint8_t JMP_INS = 0b11000011;

    assert(data.size() + 0x100 < (1 << 16) - 0x100);
    m.poke8(0x0000, JMP_INS);
    m.poke8(0x0001, BIOS_ADDR & 0xff);
    m.poke8(0x0002, BIOS_ADDR >> 8);
    m.poke8(0x0003, 0); // IO byte
    m.poke8(0x0004, 0); // Disk byte
    m.poke8(0x0005, JMP_INS); // JMP
    m.poke8(0x0006, DOS_ADDR & 0xff);
    m.poke8(0x0007, DOS_ADDR >> 8);
    // Restart vectors
    m.poke8(0x0008, HLT_INS);
    m.poke8(0x0010, HLT_INS);
    m.poke8(0x0018, HLT_INS);
    m.poke8(0x0020, HLT_INS);
    m.poke8(0x0028, HLT_INS);
    m.poke8(0x0030, HLT_INS);
    m.poke8(0x0038, HLT_INS);
    m.write_mem(0x0100, data.data(), static_cast<uint16_t>(data.size()));
    memset(&mem.mem()[DOS_ADDR], HLT_INS, 65536 - DOS_ADDR);
    state.pc = 0x100;
    m.poke8(DOS_ADDR, RET_INS);
    m.poke8(BIOS_ADDR, RET_INS);

    const auto start_time = std::chrono::steady_clock::now();
    auto report = [&]() {
        output_flush();
        const auto secs = std::chrono::duration<double> { std::chrono::steady_clock::now() - start_time }.count();
        std::cout << state.instruction_count << " instructions executed in " << secs << " seconds (" << (state.instruction_count / 1e6) / secs << " MIPS)\n";
        std::cout << state.cycle_count << " clock cycles (" << state.cycle_count / 2e6 << " seconds at 2MHz) " << (state.cycle_count / 2e6)*100/secs << "% compared to real time.\n";
    };
    //m.trace(&std::cout);
    try {
        while (!soft_reset) {
            m.step();
        }
    } catch (...) {
        report();
        throw;
    }
    report();
    std::cout << "Exiting after soft reset\n";
}
