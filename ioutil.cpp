#include "ioutil.h"
#include <ostream>
#include <cassert>
#include <sstream>
#include <fstream>

std::ostream& operator<<(std::ostream& os, const num_formatter& nf)
{
    assert(nf.base_ == 2 || nf.base_ == 16); // TODO: Binary at least
    assert(nf.width_ > 0);

    const uint8_t mask = static_cast<uint8_t>(nf.base_-1);
    const uint8_t shift = nf.base_ == 16 ? 4 : 1;

    for (int w = nf.width_; w--;) {
        os << ("0123456789ABCDEF"[(nf.num_ >> (w*shift)) & mask]);
    }
    return os;
}

std::string detail::do_format(const num_formatter& nf)
{
    std::ostringstream os;
    os << nf;
    return os.str();
}

std::vector<uint8_t> read_file(const std::string& path)
{
    std::ifstream in { path, std::ifstream::binary };
    if (!in) {
        throw std::runtime_error { "Error opening " + path };
    }

    in.seekg(0, std::ifstream::end);
    const auto len = static_cast<unsigned>(in.tellg());
    in.seekg(0, std::ifstream::beg);

    std::vector<uint8_t> buf(len);
    if (len) {
        in.read(reinterpret_cast<char*>(&buf[0]), len);
    }
    if (!in) {
        throw std::runtime_error { "Error reading from " + path };
    }
    return buf;
}

void hexdump(std::ostream& os, const uint8_t* data, size_t size, uint16_t address)
{
    constexpr size_t width = 16;
    for (size_t i = 0; i < size;) {
        const size_t here = std::min(size - i, width);

        os << hex(address + i, 4) << "  ";

        for (size_t j = 0; j < here; ++j)
            os << hex(data[i + j]) << ' ';
        for (size_t j = here; j < width; ++j)
            os << "   ";
        for (size_t j = 0; j < here; ++j) {
            const uint8_t d = data[i + j];
            os << static_cast<char>(d >= 32 && d < 128 ? d : '.');
        }
        os << "\n";
        i += here;
    }
}

uint8_t get_digit(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        return 10 + ch - 'a';
    else if (ch >= 'A' && ch <= 'F')
        return 10 + ch - 'A';
    else
        return 0xff;
}

std::optional<uint16_t> read_number(const char* s)
{
    if (!*s)
        return {};

    uint8_t base = 16;
    uint16_t num = 0;
    while (*s) {
        const uint8_t digit = get_digit(*s);
        if (digit > base)
            return {};
        if (num * base + digit >= 65536)
            return {};
        num = num * base + digit;
        ++s;
    }
    return { num };
}