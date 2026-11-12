#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
using String=std::string;
struct TestESP { uint64_t getEfuseMac() const { return 0; } };
static TestESP ESP;
