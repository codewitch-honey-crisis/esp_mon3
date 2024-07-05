#pragma once
#include "interface.hpp"

bool serial_init();
bool serial_read_packet(read_status_t* out_status);