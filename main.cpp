#include <iostream>
#include <sstream>
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

//      76543210
// STC  00110111 ----C F.C=1
// CMC  00111111 ----C F.C=~F.C
// INR  00reg100 SZAP- reg+=1
// DCR  00reg101 SZAP- reg-=1
// CMA  00101111 ----- A = ~A
// DAA  00100111 SZAPC TODO
// NOP  00000000 ----- NOP
// STAX 000x0010 ----- x=0: pair B, x=1 pair D  (Rpair) <- A
// LDAX 000x1010 ----- x=0: pair B, x=1 pair D  A <- (Rpair)
// ROT  000op111 ----C
// PUSH 11rp0101 -----
// POP  11rp0001 -----
// DAD  00rp1001 ----C HL += rp
// XCHG 11101011 ----- HL <=> DE
// XTHL 11100011 ----- H <=> (SP+1), L <=> (SP)
// SPHL 11111001 ----- SP <- HL

// opI  11ope110 SZAPC ope = operation, A = A op 8 bit immediate

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

// cnd (condition code)
// 000 = NZ
// 001 = Z
// 010 = NC
// 011 = C
// 100 = PO
// 101 = PE
// 110 = P
// 111 = M


constexpr uint16_t DOS_ADDR = 0xE400;

// Flags: Carry, Aux carry (carry out on bit3), Sign, Zero, Parity
// SZ0A0P1C

constexpr uint8_t sf_bit = 7;
constexpr uint8_t zf_bit = 6;
constexpr uint8_t af_bit = 4;
constexpr uint8_t pf_bit = 2;
constexpr uint8_t cf_bit = 0;

constexpr uint8_t sf_mask = 1 << sf_bit;
constexpr uint8_t zf_mask = 1 << zf_bit;
constexpr uint8_t af_mask = 1 << af_bit;
constexpr uint8_t pf_mask = 1 << pf_bit;
constexpr uint8_t cf_mask = 1 << cf_bit;

// Note: Keep instructions with condition codes together
#define INSTRUCTIONS(X) \
    X(UNIMPLEMENTED)    \
    X(ADC)              \
    X(ADD)              \
    X(ANA)              \
    X(CALL)             \
    X(CMP)              \
    X(DCX)              \
    X(HLT)              \
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
    X(LHLD)             \
    X(LXI)              \
    X(MOV)              \
    X(MVI)              \
    X(NOP)              \
    X(ORA)              \
    X(POP)              \
    X(PUSH)             \
    X(RET)              \
    X(SPHL)             \
    X(SBB)              \
    X(SUB)              \
    X(XRA)              \

// Keep above line clear

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
    RB = B | reg_mask,
    RC = C | reg_mask,
    RD = D | reg_mask,
    RE = E | reg_mask,
    RH = H | reg_mask,
    RL = L | reg_mask,
    RPSW = PSW | reg_mask,
    RA = A | reg_mask,
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
    instructions[0b00101010] /* 2A */ = { LHLD, IMM16, NONE };
    instructions[0b01110110] /* 76 */ = { HLT, NONE, NONE };
    instructions[0b11111001] /* F9 */ = { SPHL, NONE, NONE };

    auto reg_or_mem = [](int val) {
        assert(val>= 0 && val <= 7);
        return static_cast<arg>(val == 6 ? MEM : reg_mask | val);
    };

    auto reg_pair = [](int val, bool is_pushpop = false) {
        assert(val >= 0 && val <= 3);
        return val == 3 ? (is_pushpop ? RPSW : RSP) : static_cast<arg>(RB + val * 2);
    };

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
    // OP   10opereg SZAPC ope = opreation, reg = reg or mem, A <- A op reg/mem
    for (int ope = 0; ope < 8; ++ope) {
        constexpr instruction_type optype[8] = {
            ADD, ADC, SUB, SBB, ANA, XRA, ORA, CMP
        };
        for (int reg = 0; reg < 8; ++reg) {
            auto& inst = instructions[0b10000000 | ope << 3 | reg];
            assert(inst.type == UNIMPLEMENTED);
            inst.type = optype[ope];
            inst.args[0] = reg_or_mem(reg);
        }
    }

    // Rcc  11cnd000
    instructions[0b11001001] /* C9 */ = { RET, NONE, NONE };

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
        default:
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
    os << (psw & af_mask ? 'A' : 'a');
    os << (psw & pf_mask ? 'P' : 'p');
    os << (psw & cf_mask ? 'C' : 'c');
    return os;
}

class machine {
public:
    explicit machine()
    {
        reset();
    }

    void reset()
    {
        memset(mem_, 0, sizeof(mem_));
        memset(&state_, 0, sizeof(state_));
    }

    uint8_t* mem()
    {
        return mem_;
    }

    machine_state& state()
    {
        return state_;
    }

    void step();

private:
    machine_state state_ {};
    uint8_t mem_[1 << 16];

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

    uint16_t read16(uint16_t addr) const
    {
        return mem_[addr] | mem_[(addr + 1) & 0xffff] << 8;
    }

    void write16(uint16_t addr, uint16_t val)
    {
        mem_[addr] = static_cast<uint8_t>(val);
        mem_[(addr + 1) & 0xffff] = static_cast<uint8_t>(val >> 8);
    }

    uint8_t read8(arg a) const
    {
        switch (a) {
        case MEM:
            return mem_[read16(RH)];
        }
        std::ostringstream oss;
        oss << "Invalid argument in read8: " << a << "\n";
        throw std::runtime_error { oss.str() };
    }

    uint16_t read16(arg a) const
    {
        switch (a) {
        case RB:
            return state_.regs[B] << 8 | state_.regs[C];
        case RD:
            return state_.regs[D] << 8 | state_.regs[E];
        case RH:
            return state_.regs[H] << 8 | state_.regs[L];
        case RPSW:
            return state_.regs[PSW] << 8 | state_.regs[A];
        }
        std::ostringstream oss;
        oss << "Invalid argument in read16: " << a << "\n";
        throw std::runtime_error { oss.str() };
    }

    void write8(arg a, uint8_t val)
    {
        switch (a) {
        case RB:
        case RC:
        case RD:
        case RE:
        case RH:
        case RL:
        case RA:
            state_.regs[a - RB] = val;
            return;
        }
        std::ostringstream oss;
        oss << "Invalid argument in write8: " << a << "\n";
        throw std::runtime_error { oss.str() };
    }

    void write16(arg a, uint16_t val)
    {
        switch (a) {
        case RB:
            state_.regs[B] = static_cast<uint8_t>(val >> 8);
            state_.regs[C] = static_cast<uint8_t>(val);
            return;
        case RD:
            state_.regs[D] = static_cast<uint8_t>(val >> 8);
            state_.regs[E] = static_cast<uint8_t>(val);
            return;
        case RH:
            state_.regs[H] = static_cast<uint8_t>(val >> 8);
            state_.regs[L] = static_cast<uint8_t>(val);
            return;
        case RPSW:
            state_.regs[PSW] = static_cast<uint8_t>(val >> 8);
            state_.regs[A]   = static_cast<uint8_t>(val);
            return;
        }
        std::ostringstream oss;
        oss << "Invalid argument in write16: " << a << "\n";
        throw std::runtime_error { oss.str() };
    }

    void push(uint16_t val)
    {
        write16(state_.sp -= 2, val);
    }

    uint16_t pop()
    {
        const auto val = read16(state_.sp);
        state_.sp += 2;
        return val;
    }

    void dos_call();
};

void machine::step()
{
    if (state_.pc == DOS_ADDR) {
        dos_call();
        state_.pc = pop();
        return;
    }

    const auto inst_num = pc_read();
    const auto& inst = instructions[inst_num];

    switch (inst.type) {
    case CALL:
        push(state_.pc + 2);
        state_.pc = pc_read16();
        break;
    case JMP:
        state_.pc = pc_read16();
        break;
    case LHLD:
        write16(RH, read16(pc_read16()));
        break;
    case LXI:
        write16(inst.args[0], pc_read16());
        break;
    case MOV:
        write8(inst.args[0], read8(inst.args[1]));
        break;
    case MVI:
        write8(inst.args[0], pc_read());
        break;
    case POP:
        write16(inst.args[0], pop());
        break;
    case PUSH:
        push(read16(inst.args[0]));
        break;
    case RET:
        state_.pc = pop();
        break;
    case SPHL:
        state_.sp = read16(RH);
        break;
    default:
        throw std::runtime_error { "TODO: Handle instruction " + hexstring(inst_num) + "h " + binstring(inst_num) + ": " + instruction_names[inst.type] };
    }
}

void machine::dos_call()
{
    switch (state_.regs[C]) {
    case 9: // Print string
        {
            uint16_t addr = read16(RD);
            std::string s;
            for (unsigned i = 0; i < 65536 && mem_[addr] != '$'; ++i, ++addr)
                s.push_back(mem_[addr]);
            std::cout << "Program writes: \"" << s << "\"\n";
            return;
        }
    }
    std::cout << "TODO: DOSCALL " << static_cast<int>(state_.regs[C]) << "\n";
    throw std::runtime_error { "TODO: Handle DOSCALL " + std::to_string(state_.regs[C]) };
}

void run_test()
{
    machine m;
    const auto data = read_file("../misc/cputest/8080EXER.COM");
    assert(data.size() + 0x100 < (1<<16) - 0x100);
    m.mem()[0x05] = 0xC3; // JMP
    m.mem()[0x06] = DOS_ADDR & 0xff;
    m.mem()[0x07] = DOS_ADDR >> 8;
    m.mem()[DOS_ADDR] = 0b01110110; // HLT
    memcpy(&m.mem()[0x100], data.data(), data.size());
    auto& state = m.state();
    m.state().pc = 0x100;
    for (uint16_t pc = 0x100; pc < 0x100 + data.size();) {
        disasm_one(std::cout, m.mem(), state.pc);
        m.step();
        std::cout << m.state() << "\n";
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
