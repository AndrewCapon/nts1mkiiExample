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

in -Os optimser adds 1180 bytes to size
in -O0 optimser adds 1448 bytes to size

You may also see the size go up more depending on the maths you are using!
