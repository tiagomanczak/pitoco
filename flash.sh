#!/bin/bash

# Flash the device using OpenOCD, the command cargo run passes the binary as the first parameter
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 9000" -c "program $1 verify reset exit"