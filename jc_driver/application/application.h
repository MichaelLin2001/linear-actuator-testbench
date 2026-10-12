
#ifndef _APPLICATION_H_
#define _APPLICATION_H_
#include "stdint.h"
#include "Console.hpp"
#include "jc01.h"

void app_init(CppReadline::Console& console, std::string& info);
void app_deinit(void);
#endif  //_APPLICATION_H_