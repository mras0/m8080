#include <iostream>
#include <sstream>
#include <functional>
#include <map>
#include <chrono>
#include <assert.h>
#include "ioutil.h"

// Register pairs:
// B: B and C (0, 1)
// D: D and E (2, 3)
// H: H and L (4, 5)
// PSW: flags and A (6, 7)

// PC: 16bits
// SP: 16bits
// I/O: 256 

// Instructions up to 3 bytes in length

// Push: SP[-1] = high, SP[-2] = low, SP -= 2

// rp:
//   00 = BC
//   01 = DE
//   10 = HL
//   11 = SP/FA (SP for all except PUSH/POP)

// ope:
//   000 = ADD
//   001 = ADC
//   010 = SUB
//   011 = SBB
//   100 = ANA
//   101 = XRA
//   110 = ORA
//   111 = CMP

// rotate op:
// 00=RLC
// 01=RRC
// 10=RAL
// 11=RAR


// Flags: Carry, Aux carry (carry out on bit3), Sign, Zero, Parity
// SZ0A0P1C

constexpr uint8_t sf_bit = 7;
constexpr uint8_t zf_bit = 6;
constexpr uint8_t az_bit = 5; // Always zero
constexpr uint8_t af_bit = 4;
constexpr uint8_t pf_bit = 2;
constexpr uint8_t ao_bit = 1; // Always one
constexpr uint8_t cf_bit = 0;

constexpr uint8_t sf_mask = 1 << sf_bit;
constexpr uint8_t zf_mask = 1 << zf_bit;
constexpr uint8_t az_mask = 1 << az_bit;
constexpr uint8_t af_mask = 1 << af_bit;
constexpr uint8_t pf_mask = 1 << pf_bit;
constexpr uint8_t ao_mask = 1 << ao_bit;
constexpr uint8_t cf_mask = 1 << cf_bit;

// cnd (condition code)
// 000 = NZ
// 001 = Z
// 010 = NC
// 011 = C
// 100 = PO
// 101 = PE
// 110 = P
// 111 = M
enum {
    CONDNZ, CONDZ, CONDNC, CONDC, CONDPO, CONDPE, CONDP, CONDM
};
static_assert(CONDM == 7);

// Note: Keep instructions with condition codes together
#define INSTRUCTIONS(X) \
    X(UNIMPLEMENTED)    \
    X(ADC)              \
    X(ADD)              \
    X(ACI)              \
    X(ADI)              \
    X(ANA)              \
    X(ANI)              \
    X(CALL)             \
    X(CNZ)              \
    X(CZ)               \
    X(CNC)              \
    X(CC)               \
    X(CPO)              \
    X(CPE)              \
    X(CP)               \
    X(CM)               \
    X(CMA)              \
    X(CMC)              \
    X(CPI)              \
    X(CMP)              \
    X(DAA)              \
    X(DAD)              \
    X(DCR)              \
    X(DCX)              \
    X(DI)               \
    X(EI)               \
    X(HLT)              \
    X(INR)              \
    X(INX)              \
    X(JMP)              \
    X(JNZ)              \
    X(JZ)               \
    X(JNC)              \
    X(JC)               \
    X(JPO)              \
    X(JPE)              \
    X(JP)               \
    X(JM)               \
    X(LDA)              \
    X(LDAX)             \
    X(LHLD)             \
    X(LXI)              \
    X(MOV)              \
    X(MVI)              \
    X(NOP)              \
    X(ORA)              \
    X(ORI)              \
    X(PCHL)             \
    X(POP)              \
    X(PUSH)             \
    X(RET)              \
    X(RNZ)              \
    X(RZ)               \
    X(RNC)              \
    X(RC)               \
    X(RPO)              \
    X(RPE)              \
    X(RP)               \
    X(RM)               \
    X(RAL)              \
    X(RAR)              \
    X(RLC)              \
    X(RRC)              \
    X(RST)              \
    X(SUI)              \
    X(SBB)              \
    X(SBI)              \
    X(SHLD)             \
    X(SPHL)             \
    X(SUB)              \
    X(STA)              \
    X(STAX)             \
    X(STC)              \
    X(XCHG)             \
    X(XRA)              \
    X(XRI)              \
    X(XTHL)

const char* const instruction_names[] = {
#define INAME(name) #name,
    INSTRUCTIONS(INAME)
#undef INAME
};

enum instruction_type {
#define ITYPE(name) name,
    INSTRUCTIONS(ITYPE)
#undef INAME
};

static_assert(UNIMPLEMENTED == 0);

enum regnum {
    B, C, D, E, H, L, PSW, A, SP
};
static_assert(A == 7);

constexpr uint8_t reg_mask = 0x10;
enum arg {
    NONE,
    IMM8,
    IMM16,
    MEM, // referenced through HL
    RST_ARG,
    REGB = B | reg_mask,
    REGC = C | reg_mask,
    REGD = D | reg_mask,
    REGE = E | reg_mask,
    REGH = H | reg_mask,
    REGL = L | reg_mask,
    RPSW = PSW | reg_mask,
    REGA = A | reg_mask,
    RSP = SP | reg_mask
};

std::ostream& operator<<(std::ostream& os, arg a)
{
    switch (a) {
    case NONE:
        return os << "NONE";
    case IMM8:
        return os << "IMM8";
        break;
    case IMM16:
        return os << "IMM16"; 
        break;
    case MEM:
        return os << 'M';
        break;
    case RPSW:
        return os << "PSW";
    case RSP:
        return os << "SP";
    default:
        assert(a & reg_mask);
        return os << "BCDEHL?A"[a & 7];
    }
}

struct instruction {
    instruction_type type;
    arg args[2];
};

instruction instructions[256];

void init_instructions()
{
    instructions[0b00000000] /* 00 */ = { NOP, NONE, NONE };
    instructions[0b00000010] /* 02 */ = { STAX, REGB, NONE };
    instructions[0b00000111] /* 07 */ = { RLC, NONE, NONE };
    instructions[0b00001010] /* 0A */ = { LDAX, REGB, NONE };
    instructions[0b00001111] /* 0F */ = { RRC, NONE, NONE };
    instructions[0b00010010] /* 12 */ = { STAX, REGD, NONE };
    instructions[0b00010111] /* 17 */ = { RAL, NONE, NONE };
    instructions[0b00011010] /* 1A */ = { LDAX, REGD, NONE };
    instructions[0b00011111] /* 1F */ = { RAR, NONE, NONE };
    instructions[0b00100010] /* 22 */ = { SHLD, IMM16, NONE };
    instructions[0b00100111] /* 27 */ = { DAA, NONE, NONE };
    instructions[0b00101010] /* 2A */ = { LHLD, IMM16, NONE };
    instructions[0b00101111] /* 2F */ = { CMA, NONE, NONE };
    instructions[0b00110010] /* 32 */ = { STA, IMM16, NONE }; // STA  00110010 ----- (IMM16) <- A
    instructions[0b00110111] /* 37 */ = { STC, NONE, NONE };
    instructions[0b00111010] /* 3A */ = { LDA, IMM16, NONE };
    instructions[0b00111111] /* 3F */ = { CMC, NONE, NONE };
    instructions[0b01110110] /* 76 */ = { HLT, NONE, NONE };
    instructions[0b11100011] /* E3 */ = { XTHL, NONE, NONE };
    instructions[0b11101001] /* E9 */ = { PCHL, NONE, NONE };
    instructions[0b11101011] /* EB */ = { XCHG, NONE, NONE };
    instructions[0b11111001] /* F9 */ = { SPHL, NONE, NONE };
    instructions[0b11110011] /* F3 */ = { DI, NONE, NONE };
    instructions[0b11111011] /* FB */ = { EI, NONE, NONE };

    auto reg_or_mem = [](int val) {
        assert(val>= 0 && val <= 7);
        return static_cast<arg>(val == 6 ? MEM : reg_mask | val);
    };

    auto reg_pair = [](int val, bool is_pushpop = false) {
        assert(val >= 0 && val <= 3);
        return val == 3 ? (is_pushpop ? RPSW : RSP) : static_cast<arg>(REGB + val * 2);
    };

    // INR  00reg100 SZAP- reg+=1
    for (int r = 0; r < 8; ++r) {
        auto& inst = instructions[0b00000100 | r << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = INR;
        inst.args[0] = reg_or_mem(r);
    }
    // DCR  00reg101 SZAP- reg-=1
    for (int r = 0; r < 8; ++r) {
        auto& inst = instructions[0b00000101 | r << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = DCR;
        inst.args[0] = reg_or_mem(r);
    }
    // LXI  00rp0001 ----- Load register pair with 16 bit immediate data
    for (int rp = 0; rp < 4; ++rp) {
        auto& inst = instructions[0b00000001 | rp << 4];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = LXI;
        inst.args[0] = reg_pair(rp);
        inst.args[1] = IMM16;
    }
    // INX  00rp0011 ----- rp++
    for (int rp = 0; rp < 4; ++rp) {
        auto& inst = instructions[0b00000011 | rp << 4];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = INX;
        inst.args[0] = reg_pair(rp);
        inst.args[1] = NONE;
    }
    // DAD  00rp1001 ----C HL += rp
    for (int rp = 0; rp < 4; ++rp) {
        auto& inst = instructions[0b00001001 | rp << 4];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = DAD;
        inst.args[0] = reg_pair(rp);
        inst.args[1] = NONE;
    }
    // DCX  00rp1011 ----- rp--
    for (int rp = 0; rp < 4; ++rp) {
        auto& inst = instructions[0b00001011 | rp << 4];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = DCX;
        inst.args[0] = reg_pair(rp);
        inst.args[1] = NONE;
    }
    // MVI  00reg110 ----- Load register/memory with 8 bit immediate data
    for (int reg = 0; reg < 8; ++reg) {
        auto& inst = instructions[0b00000110 | reg << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = MVI;
        inst.args[0] = reg_or_mem(reg);
        inst.args[1] = IMM8;
    }
    // MOV  01dstsrc ----- dst = src (110 for dst + src is illegal)
    for (int dst = 0; dst < 8; ++dst) {
        for (int src = 0; src < 8; ++src) {
            if (src == 6 && dst == 6)
                continue;
            auto& inst = instructions[0b01000000 | dst << 3 | src];
            assert(inst.type == UNIMPLEMENTED);
            inst.type = MOV;
            inst.args[0] = reg_or_mem(dst);
            inst.args[1] = reg_or_mem(src);
        }
    }
    for (int ope = 0; ope < 8; ++ope) {
        constexpr instruction_type optype[8] = {
            ADD, ADC, SUB, SBB, ANA, XRA, ORA, CMP
        };
        // OP   10opereg SZAPC ope = opreation, reg = reg or mem, A <- A op reg/mem
        for (int reg = 0; reg < 8; ++reg) {
            auto& inst = instructions[0b10000000 | ope << 3 | reg];
            assert(inst.type == UNIMPLEMENTED);
            inst.type = optype[ope];
            inst.args[0] = reg_or_mem(reg);
        }
        // opI  11ope110 SZAPC ope = operation, A = A op 8 bit immediate

        constexpr instruction_type immop[8] = {
            ADI, ACI, SUI, SBI, ANI, XRI, ORI, CPI
        };
        auto& inst = instructions[0b11000110 | ope << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = immop[ope];
        inst.args[0] = IMM8;
    }

    instructions[0b11001001] /* C9 */ = { RET, NONE, NONE };
    // Rcc  11cnd000
    for (int cc = 0; cc < 8; ++cc) {
        auto& inst = instructions[0b11000000 | cc << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = static_cast<instruction_type>(RNZ + cc);
    }

    // POP  11rp0001 -----
    for (int rp = 0; rp < 4; ++rp) {
        auto& inst = instructions[0b11000001 | rp << 4];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = POP;
        inst.args[0] = reg_pair(rp, true);
    }

    instructions[0b11000011] /* C3 */ = { JMP, IMM16, NONE };
    // Jcc  11cnd01x -----
    for (int cc = 0; cc < 8; ++cc) {
        auto& inst = instructions[0b11000010 | cc << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = static_cast<instruction_type>(JNZ + cc);
        inst.args[0] = IMM16;
    }

    // PUSH 11rp0101 -----
    for (int rp = 0; rp < 4; ++rp) {
        auto& inst = instructions[0b11000101 | rp << 4];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = PUSH;
        inst.args[0] = reg_pair(rp, true);
    }

    instructions[0b11001101] /* CD */ = { CALL, IMM16, NONE };
    // Ccc  11cnd10x ----- 
    for (int cc = 0; cc < 8; ++cc) {
        auto& inst = instructions[0b11000100 | cc << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = static_cast<instruction_type>(CNZ + cc);
        inst.args[0] = IMM16;
    }

    // RST  11exp111 -----
    for (int i = 0; i < 8; ++i) {
        auto& inst = instructions[0b11000111 | i << 3];
        assert(inst.type == UNIMPLEMENTED);
        inst.type = RST;
        inst.args[0] = RST_ARG;
    }
}

uint16_t disasm_one(std::ostream& os, const uint8_t* mem, uint16_t pc)
{
    const auto orig_pc = pc;
    auto get = [&]() {
        return mem[(pc++) & 0xffff];
    };

    uint8_t ibytes[3];
    uint16_t immval = 0;
    unsigned len = 1;
    ibytes[0] = get();
    const auto& inst = instructions[ibytes[0]];
    for (unsigned i = 0; i < 2 && inst.args[i] != NONE; ++i) {
        if (inst.args[i] == IMM8) {
            immval = ibytes[1] = get();
            len = 2;
        } else if (inst.args[i] == IMM16) {
            ibytes[1] = get();
            ibytes[2] = get();
            immval = ibytes[1] | ibytes[2] << 8;
            len = 3;
        }
    }

    os << hex(orig_pc) << "  ";
    for (unsigned i = 0; i < 3; ++i) {
        os << " ";
        if (i < len)
            os << hex(ibytes[i]);
        else
            os << "  ";
    }
    os << "  " << instruction_names[inst.type];

    for (unsigned i = 0; i < 2 && inst.args[i] != NONE; ++i) {
        os << (i ? ", " : "\t");
        switch (inst.args[i]) {
        case IMM8:
            os << hex(immval, 2) << 'H';
            break;
        case IMM16:
            os << hex(immval, 4) << 'H';
            break;
        case RST_ARG:
            os << ((ibytes[0] >> 3) & 7);
            break;
        default:
            assert(inst.args[i] & reg_mask);
            [[fallthrough]];
        case MEM:
            os << inst.args[i];
        }
    }
    os << "\n";

    if (inst.type == UNIMPLEMENTED)
        throw std::runtime_error { "TODO: Handle instruction " + hexstring(ibytes[0]) + "h " + binstring(ibytes[0]) };

    assert(pc == orig_pc + len);
    return pc;
}

struct machine_state {
    uint8_t regs[8];
    uint16_t sp;
    uint16_t pc;
    bool inte; // interrupts enabled
    uint64_t instruction_count;
};

std::ostream& operator<<(std::ostream& os, const machine_state& state)
{
    os << "A = " << hex(state.regs[A]);
    os << " B = " << hex(state.regs[B]);
    os << " C = " << hex(state.regs[C]);
    os << " D = " << hex(state.regs[D]);
    os << " E = " << hex(state.regs[E]);
    os << " H = " << hex(state.regs[H]);
    os << " L = " << hex(state.regs[L]);
    os << " SP = " << hex(state.sp);
    os << " PC = " << hex(state.pc);
    os << " PSW = " << hex(state.regs[PSW]) << " ";
    const auto psw = state.regs[PSW];
    os << (psw & sf_mask ? 'S' : 's');
    os << (psw & zf_mask ? 'Z' : 'z');
    os << ((psw >> az_bit) & 1);
    os << (psw & af_mask ? 'A' : 'a');
    os << (psw & pf_mask ? 'P' : 'p');
    os << ((psw >> ao_bit) & 1);
    os << (psw & cf_mask ? 'C' : 'c');
    os << " INTE=" << static_cast<int>(state.inte);
    os << " #INS: " << state.instruction_count;
    return os;
}

class machine {
public:
    using trap_func = std::function<void (machine&)>;

    explicit machine()
    {
        reset();
    }

    void reset()
    {
        memset(mem_, 0, sizeof(mem_));
        memset(&state_, 0, sizeof(state_));
        state_.inte = true;
        state_.regs[PSW] |= ao_mask;
    }

    uint8_t* mem()
    {
        return mem_;
    }

    machine_state& state()
    {
        return state_;
    }

    uint8_t read8(arg a) const;
    uint16_t read16(arg a) const;
    uint16_t read16(uint16_t addr) const;
    void write8(uint16_t addr, uint8_t val);
    void write8(arg a, uint8_t val);
    void write16(uint16_t addr, uint16_t val);
    void write16(arg a, uint16_t val);
    void push(uint16_t val);
    uint16_t pop();

    void trace(std::ostream* os)
    {
        trace_ = os;
    }

    void step();

    void add_trap(uint16_t addr, const trap_func& tf)
    {
        if (!traps_.insert({addr, tf}).second)
            throw std::runtime_error {"Trap already installed at 0" + hexstring(addr) + "H" };
    }

private:
    machine_state state_ {};
    uint8_t mem_[1 << 16];
    std::ostream* trace_ = nullptr;
    std::map<uint16_t, trap_func> traps_;

    uint8_t pc_read()
    {
        return mem_[state_.pc++];
    }

    uint16_t pc_read16()
    {
        const auto val = read16(state_.pc);
        state_.pc += 2;
        return val;
    }

    bool test_cond(uint8_t cond)
    {
        assert(cond <= CONDM);
        const auto f = state_.regs[PSW];
        switch (cond) {
        case CONDNZ:
            return !(f & zf_mask);
        case CONDZ:
            return !!(f & zf_mask);
        case CONDNC:
            return !(f & cf_mask);
        case CONDC:
            return !!(f & cf_mask);
        case CONDPO:
            return !(f & pf_mask);
        case CONDPE:
            return !!(f & pf_mask);
        case CONDP:
            return !(f & sf_mask);
        case CONDM:
            return !!(f & sf_mask);
        }
        std::ostringstream oss;
        oss << "Invalid condition: " << bin(cond) << "b\n";
        throw std::runtime_error { oss.str() };
    }

    void update_flags(uint8_t flags, uint8_t mask)
    {
        state_.regs[PSW] = (state_.regs[PSW] & ~mask) | (flags & mask);
    }

    void set_flags(uint8_t val, uint8_t carry_mask, uint8_t mask)
    {
        uint8_t flags = 0;
        if (carry_mask & 0x80)
            flags |= cf_mask;
        // https://graphics.stanford.edu/~seander/bithacks.html#ParityWith64Bits
        if (!((((val * 0x0101010101010101ULL) & 0x8040201008040201ULL) % 0x1FF) & 1))
            flags |= pf_mask; // Set for even parity
        if (carry_mask & 0x08)
            flags |= af_mask;
        if (!val)
            flags |= zf_mask;
        if (val & 0x80)
            flags |= sf_mask;
        update_flags(flags, mask);
    }
};

uint16_t machine::read16(uint16_t addr) const
{
    return mem_[addr] | mem_[(addr + 1) & 0xffff] << 8;
}

void machine::write8(uint16_t addr, uint8_t val)
{
    mem_[addr] = val;
}

void machine::write16(uint16_t addr, uint16_t val)
{
    mem_[addr] = static_cast<uint8_t>(val);
    mem_[(addr + 1) & 0xffff] = static_cast<uint8_t>(val >> 8);
}

uint8_t machine::read8(arg a) const
{
    switch (a) {
    case MEM:
        return mem_[read16(REGH)];
    case REGB:
    case REGC:
    case REGD:
    case REGE:
    case REGH:
    case REGL:
    case REGA:
        return state_.regs[a - REGB];
    }
    std::ostringstream oss;
    oss << "Invalid argument in read8: " << a << "\n";
    throw std::runtime_error { oss.str() };
}

uint16_t machine::read16(arg a) const
{
    switch (a) {
    case REGB:
        return state_.regs[B] << 8 | state_.regs[C];
    case REGD:
        return state_.regs[D] << 8 | state_.regs[E];
    case REGH:
        return state_.regs[H] << 8 | state_.regs[L];
    case RPSW:
        return state_.regs[A] << 8 | state_.regs[PSW];
    case RSP:
        return state_.sp;
    }
    std::ostringstream oss;
    oss << "Invalid argument in read16: " << a << "\n";
    throw std::runtime_error { oss.str() };
}

void machine::write8(arg a, uint8_t val)
{
    switch (a) {
    case MEM:
        mem_[read16(REGH)] = val;
        return;
    case REGB:
    case REGC:
    case REGD:
    case REGE:
    case REGH:
    case REGL:
    case REGA:
        state_.regs[a - REGB] = val;
        return;
    }
    std::ostringstream oss;
    oss << "Invalid argument in write8: " << a << "\n";
    throw std::runtime_error { oss.str() };
}

void machine::write16(arg a, uint16_t val)
{
    switch (a) {
    case REGB:
        state_.regs[B] = static_cast<uint8_t>(val >> 8);
        state_.regs[C] = static_cast<uint8_t>(val);
        return;
    case REGD:
        state_.regs[D] = static_cast<uint8_t>(val >> 8);
        state_.regs[E] = static_cast<uint8_t>(val);
        return;
    case REGH:
        state_.regs[H] = static_cast<uint8_t>(val >> 8);
        state_.regs[L] = static_cast<uint8_t>(val);
        return;
    case RPSW:
        state_.regs[A] = static_cast<uint8_t>(val >> 8);
        state_.regs[PSW] = ao_mask | (static_cast<uint8_t>(val) & ~az_mask);
        return;
    case RSP:
        state_.sp = val;
        return;
    }
    std::ostringstream oss;
    oss << "Invalid argument in write16: " << a << "\n";
    throw std::runtime_error { oss.str() };
}

void machine::push(uint16_t val)
{
    write16(state_.sp -= 2, val);
}

uint16_t machine::pop()
{
    const auto val = read16(state_.sp);
    state_.sp += 2;
    return val;
}

constexpr std::pair<uint8_t, uint8_t> add8(uint8_t l, uint8_t r, uint8_t carry = 0)
{
    const uint8_t res = l + r + !!carry;
    return { res, static_cast<uint8_t>((l & r) | ((l | r) & ~res)) };
}

constexpr std::pair<uint8_t, uint8_t> sub8(uint8_t l, uint8_t r, uint8_t carry = 0)
{
    auto [res, carryout] = add8(l, ~r, !carry);
    carryout ^= 0x80;
    //std::cout << hex(l) << "-" << hex(r) << "-" << (int)carry << " -> " << hex(res) << " c=" << hex(carryout) << "\n";
    return { res, carryout };
    //const uint8_t res = l - r - !!carry;
    //return { res, static_cast<uint8_t>((~l & r) | (~(l ^ r) & res)) };
}

void machine::step()
{
    if (auto it = traps_.find(state_.pc); it != traps_.end()) {
        it->second(*this);
        return;
    }

    if (trace_)
        disasm_one(*trace_, mem_, state_.pc);

    ++state_.instruction_count;

    assert((state_.regs[PSW] & (az_mask | ao_mask)) == ao_mask);

    const auto start_pc = state_.pc;
    const auto inst_num = pc_read();
    const auto& inst = instructions[inst_num];
    const auto a0 = inst.args[0];
    const auto a1 = inst.args[1];

    static constexpr uint8_t alu_all_mask = sf_mask | zf_mask | af_mask | pf_mask | cf_mask;
    switch (inst.type) {
    case ACI: {
        const auto [res, carry] = add8(state_.regs[A], pc_read(), state_.regs[PSW] & cf_mask);
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case ADI: {
        const auto [res, carry] = add8(state_.regs[A], pc_read());
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case ADD: {
        const auto [res, carry] = add8(state_.regs[A], read8(a0));
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case ADC: {
        const auto [res, carry] = add8(state_.regs[A], read8(a0), state_.regs[PSW] & cf_mask);
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case ANA:
        // TODO: AF
        //std::cout << state_ << "\n";
        //disasm_one(std::cout, mem_, start_pc);
        set_flags(state_.regs[A] &= read8(a0), 0, alu_all_mask);
        //std::cout << state_ << "\n";
        //std::cout << "---------\n";
        //trace(&std::cout);
        break;
    case ANI:
        // TODO: AF
        set_flags(state_.regs[A] &= pc_read(), 0, alu_all_mask);
        break;
    case CALL:
        push(state_.pc + 2);
        state_.pc = pc_read16();
        break;
    case CNZ:
    case CZ:
    case CNC:
    case CC:
    case CPO:
    case CPE:
    case CP:
    case CM: {
        const auto target = pc_read16();
        if (test_cond(static_cast<uint8_t>(inst.type - CNZ))) {
            push(state_.pc);
            state_.pc = target;
        }
        break;
    }
    case CMA:
        // CMA  00101111 ----- A = ~A
        state_.regs[A] = ~state_.regs[A];
        break;
    case CMC:
        // CMC  00111111 ----C F.C=~F.C
        state_.regs[PSW] ^= cf_mask;
        break;
    case CMP: {
        const auto [res, carry] = sub8(state_.regs[A], read8(a0));
        set_flags(res, carry, alu_all_mask);
        break;
    }
    case CPI: {
        const auto [ res, carry ] = sub8(state_.regs[A], pc_read());
        set_flags(res, carry, alu_all_mask);
        break;
    }
    case DAA: {
        uint8_t lo = state_.regs[A] & 0xf;
        uint16_t hi = state_.regs[A] & 0xf0;
        uint8_t f = 0;
        if (lo > 9 || (state_.regs[PSW] & af_mask))
            lo += 6;
        if (lo >= 0x10) {
            hi += 0x10;
            f |= af_mask;
        }
        if (hi > 0x90 || (state_.regs[PSW] & cf_mask)) {
            hi += 0x60;
            if (hi >= 0x100)
                f |= cf_mask;
        }
        state_.regs[A] = (hi & 0xf0) | (lo & 0x0f);
        set_flags(state_.regs[A], 0, sf_mask | zf_mask | pf_mask);
        update_flags(f, cf_mask | af_mask);
        break;
    }
    case DAD: {
        const auto res = read16(REGH) + read16(a0);
        update_flags(res & 0x10000 ? cf_mask : 0, cf_mask);
        write16(REGH, res & 0xffff);
        break;
    }
    case DCR: {
        const auto [res, carry] = sub8(read8(a0), 1);
        set_flags(res, carry, sf_mask | zf_mask | af_mask | pf_mask);
        write8(a0, res);
        break;
    }
    case DCX:
        write16(a0, read16(a0) - 1);
        break;
    case DI:
        state_.inte = false;
        break;
    case EI:
        state_.inte = true;
        break;
    case HLT: {
        std::ostringstream oss;
        oss << "Halt instruction executed\n";
        disasm_one(oss, mem_, start_pc);
        oss << state_;
        throw std::runtime_error { oss.str() };
    }
    case INR: {
        const auto [res, carry] = add8(read8(a0), 1);
        set_flags(res, carry, sf_mask | zf_mask | af_mask | pf_mask);
        write8(a0, res);
        break;
    }
    case INX:
        write16(a0, read16(a0) + 1);
        break;
    case JMP:
        state_.pc = pc_read16();
        break;
    case JNZ:
    case JZ:
    case JNC:
    case JC:
    case JPO:
    case JPE:
    case JP:
    case JM: {
        const auto target = pc_read16();
        if (test_cond(static_cast<uint8_t>(inst.type - JNZ)))
            state_.pc = target;
        break;
    }
    case LDA:
        state_.regs[A] = mem_[pc_read16()];
        break;
    case LDAX:
        // LDAX 000x1010 ----- x=0: pair B, x=1 pair D  A <- (Rpair)
        state_.regs[A] = mem_[read16(a0)];
        break;
    case LHLD:
        write16(REGH, read16(pc_read16()));
        break;
    case LXI:
        write16(a0, pc_read16());
        break;
    case MOV:
        write8(a0, read8(a1));
        break;
    case MVI:
        write8(a0, pc_read());
        break;
    case NOP:
        break;
    case ORA:
        set_flags(state_.regs[A] |= read8(a0), 0, alu_all_mask);
        break;
    case ORI:
        set_flags(state_.regs[A] |= pc_read(), 0, alu_all_mask);
        break;
    case PCHL:
        state_.pc = read16(REGH);
        break;
    case POP:
        write16(a0, pop());
        break;
    case PUSH:
        push(read16(a0));
        break;            
    case RAL: {
        uint8_t c = !!(state_.regs[PSW] & cf_mask);
        update_flags(state_.regs[A] & 0x80 ? cf_mask : 0, cf_mask);
        state_.regs[A] = state_.regs[A] << 1 | c;
        break;
    }
    case RAR: {
        uint8_t c = !!(state_.regs[PSW] & cf_mask);
        update_flags(state_.regs[A] & 0x01 ? cf_mask : 0, cf_mask);
        state_.regs[A] = state_.regs[A] >> 1 | c << 7;
        break;
    }
    case RET:
        state_.pc = pop();
        break;
    case RNZ:
    case RZ:
    case RNC:
    case RC:
    case RPO:
    case RPE:
    case RP:
    case RM:
        if (test_cond(static_cast<uint8_t>(inst.type - RNZ)))
            state_.pc = pop();
        break;
    case RLC:
        update_flags(state_.regs[A] & 0x80 ? cf_mask : 0, cf_mask);
        state_.regs[A] = state_.regs[A] << 1 | state_.regs[A] >> 7;
        break;
    case RRC:
        update_flags(state_.regs[A] & 0x01 ? cf_mask : 0, cf_mask);
        state_.regs[A] = state_.regs[A] >> 1 | state_.regs[A] << 7;
        break;
    case RST:
        push(state_.pc);
        state_.pc = inst_num & 0b00111000;
        break;
    case SHLD:
        write16(pc_read16(), read16(REGH));
        break;
    case SPHL:
        // SPHL 11111001 ----- SP <- HL
        state_.sp = read16(REGH);
        break;
    case STA:
        write8(pc_read16(), state_.regs[A]);
        break;
    case STAX:
        // STAX 000x0010 ----- x=0: pair B, x=1 pair D  (Rpair) <- A
        write8(read16(a0), state_.regs[A]);
        break;
    case STC:
        state_.regs[PSW] |= cf_mask;
        break;
    case SBB: {
        const auto [res, carry] = sub8(state_.regs[A], read8(a0), state_.regs[PSW] & cf_mask);
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case SBI: {
        const auto [res, carry] = sub8(state_.regs[A], pc_read(), state_.regs[PSW] & cf_mask);
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case SUB: {
        const auto [res, carry] = sub8(state_.regs[A], read8(a0));
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case SUI: {
        const auto [res, carry] = sub8(state_.regs[A], pc_read());
        set_flags(res, carry, alu_all_mask);
        state_.regs[A] = res;
        break;
    }
    case XCHG:
        // XCHG 11101011 ----- HL <=> DE
        std::swap(state_.regs[D], state_.regs[H]);
        std::swap(state_.regs[E], state_.regs[L]);
        break;
    case XRA:
        set_flags(state_.regs[A] ^= read8(a0), 0, alu_all_mask);
        break;
    case XRI:
        set_flags(state_.regs[A] ^= pc_read(), 0, alu_all_mask);
        break;
    case XTHL:
        // XTHL 11100011 ----- H <=> (SP+1), L <=> (SP)
        std::swap(state_.regs[L], mem_[state_.sp + 0]);
        std::swap(state_.regs[H], mem_[state_.sp + 1]);
        break;
    default:
        std::ostringstream oss;
        oss << "Unhandled instruction " << hexstring(inst_num) + "h " << binstring(inst_num) + ": " << instruction_names[inst.type] << "\n";
        disasm_one(oss, mem_, start_pc);
        oss << state_;
        throw std::runtime_error { oss.str() };
    }

    if (trace_)
        *trace_ << state_ << "\n";
}

std::string output_buffer;

void output_flush()
{
    if (output_buffer.empty())
        return;
    std::cout << ">>> " << output_buffer << "\n";
    output_buffer.clear();
}

void output_char(char ch)
{
    if (ch == '\n') {
        output_flush();
        return;
    }
    if (ch < 0 || ch > 0x7f) {
        output_buffer += "\\x" + hexstring(ch, 2);
        return;
    }
    output_buffer += ch;
}

void dos_call(machine& m)
{
    auto& state = m.state();
    uint8_t* mem = m.mem();
    switch (state.regs[C]) {
    case 0: // System reset
        throw std::runtime_error { "System reset" };
    case 2: // Console output
        output_char(state.regs[E]);
        break;
    case 9: // Print string
        {
            uint16_t addr = m.read16(REGD);
            for (unsigned i = 0; i < 65536 && mem[addr] != '$'; ++i, ++addr)
                output_char(mem[addr]);
        }
        break;
    default:
        std::cerr << "TODO: DOSCALL " << static_cast<int>(state.regs[C]) << "\n";
        std::cerr << state << "\n";
        throw std::runtime_error { "TODO: Handle DOSCALL " + std::to_string(state.regs[C]) };
    }
    state.pc = m.pop();
}

void run_test()
{
    constexpr uint16_t DOS_ADDR = 0xE400;
    constexpr uint16_t BIOS_ADDR = 0xF200;

    machine m;

    //const auto data = read_file("../misc/cputest/8080PRE.COM");
    //const auto data = read_file("../misc/cputest/TST8080.COM");
    //const auto data = read_file("../misc/cputest/CPUTEST.COM");
    const auto data = read_file("../misc/cputest/8080EXM.COM");

    m.add_trap(DOS_ADDR, &dos_call);
    m.add_trap(BIOS_ADDR, [](machine&) { throw std::runtime_error { "Soft reset" }; });

    constexpr uint8_t HLT_INS = 0b01110110;

    assert(data.size() + 0x100 < (1<<16) - 0x100);
    uint8_t* mem = m.mem();
    auto& state = m.state();
    mem[0x00] = 0xC3; // JMP;
    mem[0x01] = BIOS_ADDR & 0xff;
    mem[0x02] = BIOS_ADDR >> 8;
    mem[0x03] = 0; // IO byte
    mem[0x04] = 0; // Disk byte
    mem[0x05] = 0xC3; // JMP
    mem[0x06] = DOS_ADDR & 0xff;
    mem[0x07] = DOS_ADDR >> 8;
    // Restart vectors
    mem[0x08] = HLT_INS;
    mem[0x10] = HLT_INS;
    mem[0x18] = HLT_INS;
    mem[0x20] = HLT_INS;
    mem[0x28] = HLT_INS;
    mem[0x30] = HLT_INS;
    mem[0x38] = HLT_INS;
    memcpy(&mem[0x100], data.data(), data.size());
    memset(&mem[DOS_ADDR], HLT_INS, 65536 - DOS_ADDR);
    state.pc = 0x100;
    const auto start_time = std::chrono::steady_clock::now();
    //m.trace(&std::cout);
    try {
        for (;;) {
            //if (state.instruction_count == 33806185 - 5000)
            //    m.trace(&std::cout);
            m.step();
        }
    } catch (...) {
        output_flush();
        const auto secs = std::chrono::duration<double> { std::chrono::steady_clock::now() - start_time }.count();
        std::cout << state.instruction_count << " instructions executed in " << secs << " seconds (" << (state.instruction_count / 1e6) / secs << " MIPS)\n";
        throw;
    }
}

int main()
{
    try {
        init_instructions();
        run_test();
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
