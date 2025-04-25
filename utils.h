// Description: Utility functions and macros for C++ projects

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

#define sleep_s(x) std::this_thread::sleep_for(std::chrono::seconds(x))
#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
