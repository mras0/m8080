#include <iostream>
#include "ioutil.h"
#include "m8080.h"
#include "cpm.h"

void disassemble_file(const char* filename)
{
    m8080_mem_bus mem_bus { 65536 };
    const auto data = read_file(filename);
    assert(data.size() < 65536 - 0x100);
    memcpy(&mem_bus.mem()[0x100], data.data(), data.size());
    for (uint16_t pc = 0x100; pc < 0x100 + data.size();) {
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


int main()
{
    try {
        run_cpm_tests();
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
