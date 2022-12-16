//Joseph Homenick
//Simple CPU

//Module 1: Memory (RAM)

module ram (we, d, q, addr);

input we;
input [15:0] d;
output reg [15:0] q;
input [7:0] addr;
reg [15:0] MEM[0:255]; //256 x 16 memory

always @(*)
begin
    if (we) //we = 1: write to memory
    MEM[addr] <= d;
    else //we = 0: read from memory
    q <= MEM[addr];
end

endmodule

//Module 2: ALU

module alu (A, B, opALU, Rout);

input [15:0] A, B;
input [2:0] opALU;
wire [15:0] add_out, xor_out, negate_out, mul_out, div_out;
wire carry_out;
output reg [15:0] Rout;

my16bitaddsub_gate add(add_out, carry_out, A, B, 1'b0); //add
my16bitXOR X_OR(xor_out, A, B); //xor
my16bitaddsub_gate negate(negate_out, carry_out, A, B, 1'b1); //negate
my16bitmultiplier mul(A, B, mul_out); //multiply
my16bitdivide div(div_out, A, B); //divide


always @(*)
begin
    case (opALU)
        3'b000 : Rout <= xor_out; //if opalu = 0 -> Rout = A ^ B
        3'b001 : Rout <= add_out; //if opalu = 1 -> Rout = A + B
        3'b010 : Rout <=mul_out; //if opalu = 2 -> Rout = A * B
        3'b011 : Rout <= negate_out; //if opalu = 3 -> Rout = A - B
        3'b100 : Rout <= div_out; //if opalu = 4 -> Rout = A / B
        default : Rout <= Rout;
    endcase

end

endmodule

//Module 3: Controller

module ctr (clk, rst, zflag, opcode, muxPC, muxMAR, muxACC, loadMAR, loadPC,
loadACC, loadMDR, loadIR, opALU, MemRW);

input clk, rst, zflag;
input [7:0] opcode;
output reg muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR;
output reg [2:0] opALU;
output reg MemRW;

//IMPLEMENT
parameter Fetch_1 = 5'd0;  //0
parameter Fetch_2 = 5'd1; //1
parameter Fetch_3 = 5'd2; //2
parameter Decode = 5'd3; //3
parameter ExecADD_1 = 5'd4; //4
parameter ExecADD_2 = 5'd5; //5
parameter ExecXOR_1 = 5'd6; //6
parameter ExecXOR_2 = 5'd7; //7
parameter ExecLoad_1 = 5'd8; //8
parameter ExecLoad_2 = 5'd9; //9
parameter ExecStore_1 = 5'd10; //10
parameter ExecJump = 5'd11; //11
parameter ExecMUL_1 = 5'd12; //12
parameter ExecMUL_2 = 5'd13; //13
parameter ExecSUB_1 = 5'd14; //14
parameter ExecSUB_2 = 5'd15; //15
parameter ExecDIV_1 = 5'd16; //16
parameter ExecDIV_2 = 5'd17;

parameter opcode_add = 8'h01;
parameter opcode_sub = 8'h02;
parameter opcode_mul = 8'h03;
parameter opcode_div = 8'h04;
parameter opcode_xor = 8'h05;
parameter opcode_jump = 8'h06;
parameter opcode_jumpz = 8'h07;
parameter opcode_store = 8'h08;
parameter opcode_load = 8'h09;


reg [4:0] state;
reg [4:0] next;

always @ (posedge clk)
begin
    if (rst)
        state = Fetch_1;
    else
        state = next;
end

always @(state or opcode or zflag)
begin
    case(state)
        Fetch_1: next = Fetch_2;
        Fetch_2: next = Fetch_3;
        Fetch_3: next = Decode;
        Decode: 
            begin
                case (opcode)
                    opcode_add: next = ExecADD_1;
                    opcode_xor: next = ExecXOR_1;
                    opcode_load: next = ExecLoad_1;
                    opcode_store: next = ExecStore_1;
                    opcode_jump: next = ExecJump;
                    opcode_jumpz:
                        begin
                            if (zflag == 1'b1)
                                next = ExecJump;
                            else 
                                next = Fetch_1;
                        end
                    opcode_mul: next = ExecMUL_1;
                    opcode_sub: next = ExecSUB_1;
                    opcode_div: next = ExecDIV_1;
                endcase
            end

        ExecADD_1: next = ExecADD_2;
        ExecADD_2: next = Fetch_1;
        ExecXOR_1: next = ExecXOR_2;
        ExecXOR_2: next = Fetch_1;
        ExecLoad_1: next = ExecLoad_2;
        ExecLoad_2: next = Fetch_1;
        ExecStore_1: next = Fetch_1;
        ExecJump: next = Fetch_1;
        ExecMUL_1: next = ExecMUL_2;
        ExecMUL_2: next = Fetch_1;
        ExecSUB_1: next = ExecSUB_2;
        ExecSUB_2: next = Fetch_1;
        ExecDIV_1: next = ExecDIV_2;
        ExecDIV_2: next = Fetch_1;

    endcase

end

always @(state)
begin

    muxPC = 1'b0; 
    muxMAR = 1'b0; 
    muxACC = 1'b0; 
    loadMAR = 1'b0; 
    loadPC = 1'b0; 
    loadACC = 1'b0; 
    loadMDR = 1'b0; 
    loadIR = 1'b0; 
    opALU = 3'b000; 
    MemRW = 1'b0;

    case(state)
        Fetch_1:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b1; 
                loadPC = 1'b1; 
                loadACC = 1'b0; 
                loadMDR = 1'b0; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        Fetch_2:
            begin  
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b0; 
                loadMDR = 1'b1; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        Fetch_3:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b0; 
                loadMDR = 1'b0; 
                loadIR = 1'b1; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        Decode:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b1; 
                muxACC = 1'b0; 
                loadMAR = 1'b1; 
                loadPC = 1'b0; 
                loadACC = 1'b0; 
                loadMDR = 1'b0; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        ExecADD_1:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b0; 
                loadMDR = 1'b1; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        ExecADD_2:
            begin
                loadACC = 1'b1; 
                muxACC = 1'b0;
                opALU = 3'b001; 
                
            end
        ExecXOR_1:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b0; 
                loadMDR = 1'b1; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        ExecXOR_2:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b1; 
                loadMDR = 1'b0; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        ExecLoad_1:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b0; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b0; 
                loadMDR = 1'b1; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0;
            end
        ExecLoad_2:
            begin
                muxPC = 1'b0; 
                muxMAR = 1'b0; 
                muxACC = 1'b1; 
                loadMAR = 1'b0; 
                loadPC = 1'b0; 
                loadACC = 1'b1; 
                loadMDR = 1'b0; 
                loadIR = 1'b0; 
                opALU = 3'b000; 
                MemRW = 1'b0; 
            end
        ExecStore_1:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b0; 
            loadMDR = 1'b0; 
            loadIR = 1'b0; 
            opALU = 3'b000; 
            MemRW = 1'b1;
        end
        ExecJump:
        begin
            muxPC = 1'b1; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b1; 
            loadACC = 1'b0; 
            loadMDR = 1'b0; 
            loadIR = 1'b0; 
            opALU = 3'b000; 
            MemRW = 1'b0; 
        end
        ExecMUL_1:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b0; 
            loadMDR = 1'b1; 
            loadIR = 1'b0; 
            opALU = 3'b000; 
            MemRW = 1'b0;
        end
        ExecMUL_2:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b1; 
            loadMDR = 1'b0; 
            loadIR = 1'b0; 
            opALU = 3'b010; 
            MemRW = 1'b0;
        end
        ExecSUB_1:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b0; 
            loadMDR = 1'b1; 
            loadIR = 1'b0; 
            opALU = 3'b000; 
            MemRW = 1'b0;
        end
        ExecSUB_2:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b1; 
            loadMDR = 1'b0; 
            loadIR = 1'b0; 
            opALU = 3'b011; 
            MemRW = 1'b0;
        end
        ExecDIV_1:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b0; 
            loadMDR = 1'b1; 
            loadIR = 1'b0; 
            opALU = 3'b000; 
            MemRW = 1'b0; 
        end
        ExecDIV_2:
        begin
            muxPC = 1'b0; 
            muxMAR = 1'b0; 
            muxACC = 1'b0; 
            loadMAR = 1'b0; 
            loadPC = 1'b0; 
            loadACC = 1'b1; 
            loadMDR = 1'b0; 
            loadIR = 1'b0; 
            opALU = 3'b100; 
            MemRW = 1'b0;
        end
    endcase

end

endmodule

//Module 4: Register Bank

module registers(clk, rst, PC_reg, PC_next, IR_reg, IR_next, ACC_reg,
ACC_next, MDR_reg, MDR_next, MAR_reg, MAR_next, Zflag_reg, Zflag_next);

input wire clk, rst;
output reg [7:0] PC_reg;
input wire [7:0] PC_next;
output reg [15:0] IR_reg;
input wire [15:0] IR_next;
output reg [15:0] ACC_reg;
input wire [15:0] ACC_next;
output reg [15:0] MDR_reg;
input wire [15:0] MDR_next;
output reg [7:0] MAR_reg;
input wire [7:0] MAR_next;
output reg Zflag_reg;
input wire Zflag_next;

always @ (posedge clk)
begin
    if (rst)
    begin
        PC_reg <= 8'b0;
        IR_reg <= 16'b0;
        ACC_reg <= 16'b0;
        MDR_reg <= 16'b0;
        MAR_reg <= 8'b0;
        Zflag_reg <= 1'b0;
    end
    else 
    begin
        PC_reg <= PC_next;
        IR_reg <= IR_next;
        ACC_reg <= ACC_next;
        MDR_reg <= MDR_next;
        MAR_reg <= MAR_next;
        Zflag_reg <= Zflag_next;

    end
end

endmodule

//Module 5: Data Path

module datapath(clk, rst, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, 
loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ);

input clk, rst, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR;
input [2:0] opALU;
output reg zflag;
output reg [7:0] opcode, MemAddr;
output reg [15:0] MemD;
input [15:0] MemQ;

reg [7:0]PC_next;
reg [15:0]IR_next;
reg [15:0]ACC_next;
reg [15:0]MDR_next;
reg [7:0]MAR_next;
reg zflag_next;
wire [7:0]PC_reg;
wire [15:0]IR_reg;
wire [15:0]ACC_reg;
wire [15:0]MDR_reg;
wire [7:0]MAR_reg;
wire zflag_reg;
wire [15:0] R_out; //alu output

//1 instance of ALU
alu A1(ACC_reg, MDR_reg, opALU, R_out);
//1 instance of Register Bank
registers R1(clk, rst, PC_reg, PC_next, IR_reg, IR_next, ACC_reg,
ACC_next, MDR_reg, MDR_next, MAR_reg, MAR_next, Zflag_reg, Zflag_next);

always @(*)
begin
    //Assign next values for registers
    if (loadPC) //set PC_next
    PC_next = muxPC ? IR_reg[15:8] : PC_reg + 1'b1;
    else
    PC_next = PC_reg;

    if (loadIR) //set IR_next
    IR_next = MDR_reg;
    else
    IR_next = IR_reg;

    if (loadACC) //set ACC_next
    ACC_next = muxACC ? MDR_reg : R_out;
    else
    ACC_next = ACC_reg;

    if (loadMDR) //set MDR_next
    MDR_next = MemQ; //get value from memory if loadMDR is set
    else
    MDR_next = MDR_reg;

    if(loadMAR) //set MAR_next
    MAR_next = muxMAR ? IR_reg[15:8] : PC_reg;
    else 
    MAR_next = MAR_reg;

    if (ACC_reg == 0) //set Zflag
    zflag_next = 1'b1;
    else
    zflag_next = 1'b0;

    //Assign Outputs
    if(ACC_reg == 0) //Assign Z flag based on contents of accumulator
    zflag = 1'b1;
    else
    zflag = 1'b0;

    opcode = IR_reg[7:0]; //Assign opcode based on first 8 bits of instruction register

    MemAddr = MAR_reg; //Assign memory address same as MAR

    MemD = ACC_reg; //Assign data going into memory same as contents of accumulator

end

endmodule

//Module 6: Top Level

module proj1(clk, rst, MemRW_IO, MemAddr_IO, MemD_IO);
input clk;
input rst;
output MemRW_IO;
output[7:0] MemAddr_IO;
output[15:0] MemD_IO;

wire zflag, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, MemRW;
wire [2:0] opALU;
wire[7:0] opcode, MemAddr;
wire[15:0] MemD, MemQ;

ram r1(MemRW, MemD, MemQ, MemAddr);

ctr c1(clk, rst, zflag, opcode, muxPC, muxMAR, muxACC, loadMAR, loadPC,
    loadACC, loadMDR, loadIR, opALU, MemRW);

datapath d1(clk, rst, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, 
    loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ);

assign MemAddr_IO = MemAddr;
assign MemD_IO = MemD;
assign MemRW_IO = MemRW ;

endmodule

module proj1_tb;
           reg clk;
           reg rst;
	   wire MemRW_IO;
	   wire [7:0]MemAddr_IO;
           wire [15:0]MemD_IO;
	   
proj1 dut(
           clk,
           rst,
		   MemRW_IO,
		   MemAddr_IO,
           MemD_IO
		    );


always @(proj1_tb.dut.c1.state)
case (proj1_tb.dut.c1.state)
0: $display($time," Fetch_1");
1: $display($time," Fetch_2");
2: $display($time," Fetch_3");
3: $display($time," Decode");
4: $display($time," ExecADD_1");
5: $display($time," ExecADD_2");
6: $display($time," ExecXOR_1");
7: $display($time," ExecXOR_2");
8: $display($time," ExecLoad_1");
9: $display($time," ExecLoad_2");
10: $display($time," ExecStore_1");
11: $display($time," ExecJump");
12: $display($time," ExecMUL_1");
13: $display($time," ExecMUL_2");
14: $display($time," ExecSUB_1");
15: $display($time," ExecSUB_2");
16: $display($time," ExecDIV_1");
17: $display($time," ExecDIV_2");

endcase

always 
      #5  clk =  !clk; 
		
initial begin
clk=1'b0;
rst=1'b1;
$readmemh("memory.list", proj1_tb.dut.r1.MEM);
#20 rst=1'b0;
#435
$display("Final value\n");
$display("0x00e %d\n",proj1_tb.dut.r1.MEM[16'h000e]);
$finish;
end


endmodule

//16-bit MUX-Based XOR Implementation

module my1bitXOR (out, A, B);
output out;
input A, B;
wire w1;

my1bitmux M1(w1, 1'b1, 1'b0, B);
my1bitmux M2(out, B, w1, A);

endmodule

module my16bitXOR(out, A, B);
output [15:0] out;
input [15:0] A, B;
wire [15:0] w1;

my1bitXOR X0(out[0], A[0], B[0]);
my1bitXOR X1(out[1], A[1], B[1]);
my1bitXOR X2(out[2], A[2], B[2]);
my1bitXOR X3(out[3], A[3], B[3]);
my1bitXOR X4(out[4], A[4], B[4]);
my1bitXOR X5(out[5], A[5], B[5]);
my1bitXOR X6(out[6], A[6], B[6]);
my1bitXOR X7(out[7], A[7], B[7]);
my1bitXOR X8(out[8], A[8], B[8]);
my1bitXOR X9(out[9], A[9], B[9]);
my1bitXOR X10(out[10], A[10], B[10]);
my1bitXOR X11(out[11], A[11], B[11]);
my1bitXOR X12(out[12], A[12], B[12]);
my1bitXOR X13(out[13], A[13], B[13]);
my1bitXOR X14(out[14], A[14], B[14]);
my1bitXOR X15(out[15], A[15], B[15]);

endmodule

//16 Bit Multiply Implementation

module my16bitmultiplier(y, x, s);
	input [15:0] y, x;
	output[15:0] s;
	reg[15:0] s;
	reg[15:0] a;
	
	
	
	always @(*)
	begin
	s = 32'b0; //zero sum
	a = y; //set y equal to 16 bit a so we can left shift 8 bit y
	
		
		if (x[0]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[1]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[2]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[3]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[4]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[5]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[6]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
				a = a << 1;
		end
		
		if (x[7]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[8]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[9]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[10]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[11]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[12]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[13]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[14]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end

		if (x[15]) begin
				s = s + a;
				a = a << 1;
		end
		else begin
		        a = a << 1;
				
		end


	end
	
endmodule


module my1bitmux (output out, input i0, i1, sel);
	wire n_sel, x1, x2;
	or(out, x1, x2);
	and(x1, i0, n_sel);
	and(x2, i1, sel);
	not(n_sel, sel);
endmodule

module my1bithalfadder (output sum, carry, input A, B);
	supply1 pwr;
	supply0 gnd;
	
	wire x1;

	my1bitmux M1(x1, pwr, gnd, B); //XOR
	my1bitmux M2(sum, B, x1, A); //XOR
	my1bitmux M3(carry, gnd, A, B); //AND
	
endmodule


module my1bitfulladder(output C_out, sum, input A, B, C_in);
	supply1 pwr;
	
	wire x1, x2, x3;
	
	my1bithalfadder HA1(x2, x1, A, B); //First half adder
	my1bitmux M4(C_out, x3, pwr, x1); // OR gate
	my1bithalfadder HA2(sum, x3, x2, C_in); //Second half adder

endmodule

	
module my16bitfulladder(output C_out, output [15:0]S, input [15:0]A, B, input C_in);

	wire x1, x2, x3, x4, x5, x6, x7, x8, x9, x_10, x_11, x_12, x_13, x_14, x_15;
	
	my1bitfulladder FA0(x1, S[0], A[0], B[0], C_in); 
	my1bitfulladder FA1(x2, S[1], A[1], B[1], x1);
	my1bitfulladder FA2(x3, S[2], A[2], B[2], x2);
    my1bitfulladder FA3(x4, S[3], A[3], B[3], x3);
    my1bitfulladder FA4(x5, S[4], A[4], B[4], x4);
    my1bitfulladder FA5(x6, S[5], A[5], B[5], x5);
    my1bitfulladder FA6(x7, S[6], A[6], B[6], x6);
	my1bitfulladder FA7(x8, S[7], A[7], B[7], x7);
    my1bitfulladder FA8(x9, S[8], A[8], B[8], x8);
    my1bitfulladder FA9(x_10, S[9], A[9], B[9], x9);
    my1bitfulladder FA10(x_11, S[10], A[10], B[10], x_10);
    my1bitfulladder FA11(x_12, S[11], A[11], B[11], x_11);
    my1bitfulladder FA12(x_13, S[12], A[12], B[12], x_12);
    my1bitfulladder FA13(x_14, S[13], A[13], B[13], x_13);
    my1bitfulladder FA14(x_15, S[14], A[14], B[14], x_14);
    my1bitfulladder FA15(C_out, S[15], A[15], B[15], x_15);
    


    

endmodule

module my16bitmux(output [15:0] Out, input [15:0]A, B, input sel);
	
    my1bitmux m15(Out[15], A[15], B[15], sel);
    my1bitmux m14(Out[14], A[14], B[14], sel);
    my1bitmux m13(Out[13], A[13], B[13], sel);
    my1bitmux m12(Out[12], A[12], B[12], sel);
    my1bitmux m11(Out[11], A[11], B[11], sel);
    my1bitmux m10(Out[10], A[10], B[10], sel);
    my1bitmux m9(Out[9], A[9], B[9], sel);
    my1bitmux m8(Out[8], A[8], B[8], sel);
    my1bitmux m7(Out[7], A[7], B[7], sel);
    my1bitmux m6(Out[6], A[6], B[6], sel);
    my1bitmux m5(Out[5], A[5], B[5], sel);
    my1bitmux m4(Out[4], A[4], B[4], sel);
    my1bitmux m3(Out[3], A[3], B[3], sel);
	my1bitmux m2(Out[2], A[2], B[2], sel);
	my1bitmux m1(Out[1], A[1], B[1], sel);
	my1bitmux m0(Out[0], A[0], B[0], sel);
	
endmodule


module my16bitaddsub_gate(output [15:0] O, output Cout, input [15:0] A, B, input S);
	supply1 pwr;
	supply0 gnd;
	
	wire [15:0] x1, n_B;
	
	//Implement inverters
	my1bitmux n_B0(n_B[0], pwr, gnd, B[0]);
	my1bitmux n_B1(n_B[1], pwr, gnd, B[1]);
	my1bitmux n_B2(n_B[2], pwr, gnd, B[2]);
	my1bitmux n_B3(n_B[3], pwr, gnd, B[3]);
    my1bitmux n_B4(n_B[4], pwr, gnd, B[4]);
    my1bitmux n_B5(n_B[5], pwr, gnd, B[5]);
    my1bitmux n_B6(n_B[6], pwr, gnd, B[6]);
    my1bitmux n_B7(n_B[7], pwr, gnd, B[7]);
    my1bitmux n_B8(n_B[8], pwr, gnd, B[8]);
    my1bitmux n_B9(n_B[9], pwr, gnd, B[9]);
    my1bitmux n_B10(n_B[10], pwr, gnd, B[10]);
    my1bitmux n_B11(n_B[11], pwr, gnd, B[11]);
    my1bitmux n_B12(n_B[12], pwr, gnd, B[12]);
    my1bitmux n_B13(n_B[13], pwr, gnd, B[13]);
    my1bitmux n_B14(n_B[14], pwr, gnd, B[14]);
    my1bitmux n_B15(n_B[15], pwr, gnd, B[15]);
	
	//Implement add/subtract
	my16bitmux mux_16(x1, B, n_B, S);
	my16bitfulladder adder(Cout, O, A, x1, S);
	
endmodule


module my16bitdivide(output reg [15:0] Q, input [15:0] A, B);

always @(*)
Q = A / B;

endmodule