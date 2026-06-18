/**
 * @file    app_init.cpp
 * @brief   应用初始化桥接
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

#include "app_init.h"
#include "global_instances.hpp"

extern "C" void super_cap_app_init(void)
{
    supercap::g_control_task.init();
}
