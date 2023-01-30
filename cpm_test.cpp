#include <iostream>
#include "m8080.h"
#include "cpm.h"

int main()
{
    try {
        run_cpm_test("../misc/cputest/TST8080.COM", true);
        run_cpm_test("../misc/cputest/8080PRE.COM");
        run_cpm_test("../misc/cputest/8080EXM.COM");
        run_cpm_test("../misc/cputest/CPUTEST.COM");
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}

