// Copyright 2025 Simon Sagmeister
#pragma once
#include <cstdint>
#include <iostream>

#include "rslcpp_test/determinism/common.hpp"

// Analyse if the state hash match in the end
enum class Nodes { NodeA, NodeB, NodeC, NodeD };

constexpr uint64_t __expected_final_states__[4] = {
  3924739178523851918ul,   // NodeA
  416215330994369847ul,    // NodeB
  18314823502387776320ul,  // NodeC
  1601759294353606794ul,   // NodeD

};
static uint64_t __final_states__[4] = {0, 0, 0, 0};
void register_final_state(Nodes node, uint64_t state_hash)
{
  std::size_t index = static_cast<std::size_t>(node);
  __final_states__[index] = state_hash;
}
void check_final_states()
{
  for (std::size_t i = 0; i < 4; ++i) {
    std::cout << "Final State Node" << static_cast<char>('A' + i) << ": " << __final_states__[i]
              << " | Expected State: " << __expected_final_states__[i];
    if (__final_states__[i] == __expected_final_states__[i]) {
      std::cout << " | Test PASSED!" << std::endl;
    } else {
      std::cout << " | Test FAILED!" << std::endl;
    }
  }

  // Calculate a final combined hash
  u_int64_t final_hash = 505510631ul;
  for (auto hash : __final_states__) {
    final_hash = hash_combine(final_hash, hash);
  }

  u_int64_t const expected_final_hash = 7448837717382456148ul;
  std::cout << std::endl
            << "================================================================" << std::endl;
  if (expected_final_hash == final_hash) {
    std::cout << "Determinism Check SUCCESSFUL | Final Hash: " << final_hash << std::endl;
  } else {
    std::cout << "Determinism Check FAILED | Expected: " << expected_final_hash
              << " | Got: " << final_hash << std::endl;
  }
  std::cout << "================================================================" << std::endl;
}
