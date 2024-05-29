Simple VSCode setup for building/debugging for Korg NTS1 mkii, you need the Cortex-Debug extension installed

The makefile is setup for running with this folder at the same level as the logue-sdk folder, if yours is different then set LOGUE_SDK in the makefile to the correct location.

In the logue sdk make sure you have downloaded gcc by running the correct get_gcc_10_3-2021_10 script for your system. 

There is a build task to make the project, currently the only way I can see to copy the code across to the NTS1 is to use the Korg Librarian.

There is a debug launch that will attach to the NTS1 via a STLink.

There is some basic USART output code now, unfortunetaly it is taking up a bit too much space. Needs some work to stop some things being linked in.

void DebugOut(const char *pStr)
  Output 0 terminated string to USART.

void DebugOutParams(const char *pStr, ...)
  printf to USART, only supports basic %d, %u, %f, %x, %s


At the top of the Makefile set USE_USART, DEBUG to turn DEBUG and USART on.

DEBUG=0, USE_USART=0
   text    data     bss     dec     hex filename
   3139     216      60    3415     d57 /Volumes/X82/Development/Korg/nts1mkiiExample//build/test_osc.elf

DEBUG=0, USE_USART=1
   text    data     bss     dec     hex filename
  11034     264      60   11358    2c5e /Volumes/X82/Development/Korg/nts1mkiiExample//build/test_osc.elf

DEBUG=1, USE_USART=0
   text    data     bss     dec     hex filename
   7076     292      60    7428    1d04 /Volumes/X82/Development/Korg/nts1mkiiExample//build/test_osc.elf

DEBUG=1, USE_USART=1
   text    data     bss     dec     hex filename
  21287     340      60   21687    54b7 /Volumes/X82/Development/Korg/nts1mkiiExample//build/test_osc.elf


