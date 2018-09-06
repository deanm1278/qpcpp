About this Example
==================
This example can be built from the command prompt with the provided
Makefile. The example can also be imported as a Makefile-based
project into Eclipse-based IDEs.

This example uses the kernel timer to blink the LED on a BF706 EZ-Kit MINI board.


The Makefile
============
The provided Makefile should be easy to adapt for your own projects.
It contains three build configurations: Debug (default), Release, and
Spy.

Also, the Makefile has been specifically designed to work as an external
Makefile with the Eclipse CDT. 

The various build configurations are built as follows:

make
make clean

***
NOTE:
The installation folder of the GNU-BFIN toolset on YOUR machine needs
to be adjusted in the provided Makefile, by editing the symbol: GNU_BFIN.
As described in the comment for this symbol, the GNU-BFIN toolset is taken
from: https://github.com/deanm1278/blackfin-plus-gnu/releases
***


Adjusting Stack and Heap Sizes
==============================
The stack and heap sizes are determined in this project by the GCC linker
script (.ld file), which provides a template of the recommended GCC linker
script for QP applications.
   

Startup Code
============
TODO:

***
NOTE: The function assert_failed() typically should NOT use the stack,
because stack might be corrupted by the time this function is called.
Also, assert_failed() is intended to handle catastrophic errors and
should NOT return.
***

