#ifndef M8080_H_INCLUDED
#define M8080_H_INCLUDED

#include <stdint.h>
#include <functional>
#include <memory>
#include <vector>
#include <cassert>

struct m8080_state {
    uint8_t regs[8];
    uint16_t sp;
    uint16_t pc;
    bool inte; // interrupts enabled
    uint64_t instruction_count;
    uint64_t cycle_count;

    // Note: Must match regnnum

    uint8_t a() const
    {
        return regs[7];
    }
    uint8_t b() const
    {
        return regs[0];
    }
    uint8_t c() const
    {
        return regs[1];
    }
    uint8_t d() const
    {
        return regs[2];
    }
    uint8_t e() const
    {
        return regs[3];
    }
    uint8_t h() const
    {
        return regs[4];
    }
    uint8_t l() const
    {
        return regs[5];
    }
    uint16_t de() const
    {
        return d() << 8 | e();
    }
};

bool operator==(const m8080_state& l, const m8080_state& r);
bool operator!=(const m8080_state& l, const m8080_state& r);
std::ostream& operator<<(std::ostream& os, const m8080_state& state);

class m8080_bus {
public:
    // Memory interface
    virtual uint8_t peek8(uint16_t addr) = 0;
    virtual void poke8(uint16_t addr, uint8_t val) = 0;

    // I/O interface
    virtual uint8_t input(uint8_t port) = 0;
    virtual void output(uint8_t port, uint8_t value) = 0;

    virtual int interrupt() = 0;
    virtual void inte(bool) = 0;
};

class m8080_mem_bus final : public m8080_bus {
public:
    explicit m8080_mem_bus(size_t mem_size)
        : mem_(mem_size)
    {
        assert(mem_size <= 65536);
    }

    uint8_t peek8(uint16_t addr) override
    {
        assert(addr < mem_.size());
        return addr < mem_.size() ? mem_[addr] : 0xFF;
    }

    void poke8(uint16_t addr, uint8_t val) override
    {
        assert(addr < mem_.size());
        if (addr < mem_.size())
            mem_[addr] = val;
    }

    std::vector<uint8_t>& mem()
    {
        return mem_;
    }

    uint8_t input(uint8_t) override
    {
        assert(0);
        return 0xFF;
    }

    void output(uint8_t, uint8_t) override
    {
        assert(0);
    }

    int interrupt() override
    {
        return -1;
    }

    void inte(bool) override
    {
    }

private:
    std::vector<uint8_t> mem_;
};

class m8080 {
public:
    explicit m8080(m8080_bus& bus);
    ~m8080();

    m8080_state& state();

    uint8_t step();

    using trap_func = std::function<void(void)>;
    void add_trap(uint16_t addr, const trap_func& tf);

    uint8_t peek8(uint16_t addr);
    void poke8(uint16_t addr, uint8_t val);
    void write_mem(uint16_t addr, const void* src, uint16_t len);

    void trace(std::ostream* os);

private:
    class impl;
    std::unique_ptr<impl> impl_;
};

uint16_t disasm_one(std::ostream& os, m8080_bus& bus, uint16_t pc);

uint8_t instruction_length(uint8_t instruction);

#endif
