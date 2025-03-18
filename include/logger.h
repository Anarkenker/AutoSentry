#pragma once
#include <iostream>


template<typename... Args>
void log_info(Args&&... args)
{
#ifndef NO_LOG_INFO
    ((std::cout << args << "    "), ...);
    std::cout << std::flush;
#endif
}

inline void log_new_line()
{
#ifndef NO_LOG_INFO
    std::cout << std::endl;
#endif
}