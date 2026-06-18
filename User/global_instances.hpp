/**
 * @file    global_instances.hpp
 * @brief   全局 C++ 对象实例声明
 */
#ifndef GLOBAL_INSTANCES_HPP
#define GLOBAL_INSTANCES_HPP

#include "super_cap_comm.hpp"
#include "control_task.hpp"

namespace supercap {

extern SuperCapComm g_super_cap_comm;
extern ControlTask g_control_task;

} // namespace supercap

#endif // GLOBAL_INSTANCES_HPP
