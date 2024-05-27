Cortex-Debug: VSCode debugger extension version 1.12.1 git(652d042). Usage info: https://github.com/Marus/cortex-debug#usage
Reading symbols from arm-none-eabi-objdump --syms -C -h -w /Volumes/X82/Development/Korg/logue-sdk/platform/nts-1_mkii/dummy-osc/build/dummy_osc.elf
Reading symbols from arm-none-eabi-nm --defined-only -S -l -C -p /Volumes/X82/Development/Korg/logue-sdk/platform/nts-1_mkii/dummy-osc/build/dummy_osc.elf
Launching GDB: arm-none-eabi-gdb -q --interpreter=mi2
    IMPORTANT: Set "showDevDebugOutput": "raw" in "launch.json" to see verbose GDB transactions here. Very helpful to debug issues or report problems
Launching gdb-server: /
Users/andrewcapon/Development/openocd_safe/bin/openocd -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" -s /Volumes/X82/Development/Korg/logue-sdk/platform/nts-1_mkii -f /Users/andrewcapon/.vscode/extensions/marus25.cortex-debug-1.12.1/support/openocd-helpers.tcl -f "/Volumes/X82/Development2/Daisy/STM32/TestKorg/TestKorg Debug.cfg"
    Please check TERMINAL tab (gdb-server) for output from /Users/andrewcapon/Development/openocd_safe/bin/openocd
Finished reading symbols from objdump: Time: 21 ms
Finished reading symbols from nm: Time: 19 ms
OpenOCD: GDB Server Quit Unexpectedly. See gdb-server output in TERMINAL tab for more details.