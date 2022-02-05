#include "Random.h"

std::mt19937_64 &PRNG64Bit()
{
    static thread_local std::mt19937_64 gen{std::random_device{}()};
    return gen;
}