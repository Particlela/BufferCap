/**
 * @file    global_instances.cpp
 * @brief   全局 C++ 对象实例定义
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

#include "global_instances.hpp"

namespace supercap {

SuperCapComm g_super_cap_comm;
ControlTask g_control_task;

} // namespace supercap
