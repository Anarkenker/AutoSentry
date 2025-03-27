#pragma once
#include <iostream>
#include <fstream>

inline auto logout = std::ofstream("log.txt");
template<typename... Args>
void log_info(Args&&... args)
{
#ifndef NO_LOG_INFO
    ((std::cout << args << "    "), ...);
    std::cout << std::flush;
    
    ((logout << args << "    "), ...);
    logout << std::flush;
#endif
}

inline void log_new_line()
{
#ifndef NO_LOG_INFO
    std::cout << std::endl;
    logout << std::endl;
#endif
}