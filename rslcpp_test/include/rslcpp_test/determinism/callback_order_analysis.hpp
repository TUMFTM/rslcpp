// Copyright 2025 Simon Sagmeister
#pragma once
#include <unordered_set>
#include <vector>

#include "rslcpp_test/determinism/common.hpp"

// Used to track executed callbacks to ensure each is printed exactly once
static std::unordered_set<uint64_t> __executed_callback_hashes__;
static std::vector<uint64_t> __callback_execution_order__;
void register_callback(uint64_t callback_seed)
{
  if (__executed_callback_hashes__.find(callback_seed) == __executed_callback_hashes__.end()) {
    std::cout << "Executing callback | Seed: " << callback_seed << std::endl;
    __executed_callback_hashes__.insert(callback_seed);
  }
  __callback_execution_order__.push_back(callback_seed);
}
void print_report()
{
  std::cout << "Executed " << __executed_callback_hashes__.size() << " different callbacks!"
            << std::endl;
  std::cout << "Executed " << __callback_execution_order__.size() << " callbacks in total!"
            << std::endl;
}