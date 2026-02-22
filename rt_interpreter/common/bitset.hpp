#pragma once

#include <cstdint>

namespace holly_bitset{

enum class Bit : uint8_t {

    DO0 = 0,
    DO1 = 1,
    DO2 = 2,
    DO3 = 3,
    DO4 = 4,
    DO5 = 5,
    DO6 = 6,
    DO7 = 7

};

constexpr uint8_t mask(Bit b){

    return static_cast<uint8_t>(1u << static_cast<uint8_t>(b));
}

inline void setBit(uint8_t& req, Bit b){
    
    req |= mask(b);
}

inline void clearBit(uint8_t& req, Bit b){


    req &= static_cast<uint8_t>(~mask(b));
    
}

inline void toggleBit(uint8_t& req, Bit b){

    req ^=mask(b);

}

inline bool test(uint8_t req, Bit b){
    return (req & mask(b)) != 0;
}

}
