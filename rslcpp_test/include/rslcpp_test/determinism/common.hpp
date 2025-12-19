// Copyright 2025 Simon Sagmeister
#pragma once
#include <cstdint>
/// @brief SplitMix64 hashing function
/// @note Taken from https://prng.di.unimi.it/splitmix64.c
/// @note See http://dx.doi.org/10.1145/2714064.2660195
uint64_t splitmix64(uint64_t x)
{
  x += 0x9e3779b97f4a7c15;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9;
  x = (x ^ (x >> 27)) * 0x94d049bb133111eb;
  x = x ^ (x >> 31);
  return x;
}
/// @brief Calculate a hash for a 64-bit integer value.
/// @param value
/// @return
uint64_t hash(uint64_t value) { return splitmix64(value); }
/// @brief Combines two 64-bit hash values into one high-quality hash.
/// @param h1 First hash value.
/// @param h2 Second hash value.
/// @return Combined hash value.
/// @note Adopted from
/// https://github.com/google/cityhash/blob/f5dc54147fcce12cefd16548c8e760d68ac04226/src/city.h#L101
uint64_t hash_combine(uint64_t h1, uint64_t h2)
{
  const uint64_t k = 0x9ddfea08eb382d69ULL;  // large mixing constant

  uint64_t a = (h1 ^ h2) * k;
  a ^= (a >> 47);

  uint64_t b = (h2 ^ a) * k;
  b ^= (b >> 47);
  b *= k;

  return hash(b);
}
