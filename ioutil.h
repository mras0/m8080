#ifndef IOUTIL_H
#define IOUTIL_H

#include <iosfwd>
#include <stdint.h>
#include <string>
#include <vector>
#include <optional>

class num_formatter {
public:
    explicit num_formatter(uint64_t num, int base, int width) : num_{num}, base_{base}, width_{width} {}
    friend std::ostream& operator<<(std::ostream& os, const num_formatter& hf);
private:
    uint64_t num_;
    int base_;
    int width_;
};


template <typename T>
num_formatter hex(T n, int w = static_cast<int>(sizeof(T) * 2))
{
    return num_formatter { static_cast<uint64_t>(n), 16, w };
}

template <typename T>
num_formatter bin(T n, int w = static_cast<int>(sizeof(T) * 8))
{
    return num_formatter { static_cast<uint64_t>(n), 2, w };
}

namespace detail {
std::string do_format(const num_formatter& nf);
}

template <typename T>
std::string hexstring(T n, int w = static_cast<int>(sizeof(T) * 2))
{
    return detail::do_format(hex(n, w));
}

template <typename T>
std::string binstring(T n, int w = static_cast<int>(sizeof(T) * 8))
{
    return detail::do_format(bin(n, w));
}

std::vector<uint8_t> read_file(const std::string& path);
void hexdump(std::ostream& os, const uint8_t* data, size_t size, uint16_t address = 0);

std::optional<uint16_t> read_number(const char* s);

#endif
