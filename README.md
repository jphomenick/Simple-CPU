# Simple-CPU
This a simple 16-bit CPU coded in Verilog. Below is some information about the CPU.
<br>
### Instruction Processing

Each instruction is 16-bits (one word) on the register.
<br>
The bits are numbered left to right from [15] to [0].
<br>
Bits [15:8] contain the Address/Operand
<br>
Bits [7:0] contain the Opcode

### ALU Operation Instructions
0x01 Addition
<br>
0x02 Subtraction
<br>
0x03 Multiplication
<br>
0x04 Division
<br>
0x05 XOR
<br>
0x06 JUMP
<br>
0x07 JUMPZ
<br>
0x08 STORE
<br>
0x09 LOAD
### Modules
Memory
<br>
ALU
<br>
Controller
<br>
Registers
<br>
Datapath
<br>
Top Level Module

