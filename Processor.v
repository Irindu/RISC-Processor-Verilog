

module processor;

	reg LoadInstructionMemory,clearRegFile;
	reg [15:0] WriteAddIns;
	reg [15:0] InstructionIn;
	wire [3:0] ALU,outMux1;
	wire [15:0] ProgramCounterin,ProgramCounterout,instruction,ReadData1,ReadData2,zMain,zalu2,zalu3,DataReadMem,SignExtenderOut,outMux4,outMux3,outMux2;
	wire RegWrite,BEQ,MemRead,MemWrite,MemtoReg,lt,eq,gt,c_out,ALUSrc,RegDest,Branch,Jump;
		
	control TheController(RegDest,Branch,Jump,MemRead,MemWrite,MemtoReg,ALU,ALUSrc,RegWrite,instruction,CLK);
	InstructionMemory TheInstructionMemory(LoadInstructionMemory,WriteAddIns,InstructionIn,ProgramCounterout,instruction);
	reg_file TheRegisterFile(ReadData1,ReadData2,outMux3,instruction[11:8],instruction[7:4],outMux1,RegWrite,clearRegFile,CLK);
	DataMemory TheDataMemory(DataReadMem,zMain,ReadData2,zMain,MemRead,MemWrite);
	ALU Main(zMain,c_out,lt,BEQ,gt,1'b0,ALU,ReadData1,outMux2);
	ALU alu2(zalu2,c_out,lt,eq,gt,1'b0,4'b0010,ProgramCounterout,16'b10);
	ALU alu3(zalu3,c_out,lt,eq,gt,1'b0,4'b0010,zalu2,SignExtenderOut << 1);
	muxquarter Mux14bit(outMux1,RegDest,instruction[3:0],instruction[7:4]);
	mux2_to_1 Mux2(outMux2,ALUSrc,ReadData2,SignExtenderOut);
	mux2_to_1 Mux3(outMux3,MemtoReg,zMain,DataReadMem);
	mux2_to_1 Mux4(outMux4,(Branch & (~BEQ)),zalu2,zalu3);
	mux2_to_1 Mux5(ProgramCounterin,Jump,outMux4,{ProgramCounterout[15:13],instruction[12:1],1'b0});
	SignExtend TheSignExtender(SignExtenderOut,instruction[3:0]);
	ProgramCounter TheProgramCounter(ProgramCounterout,ProgramCounterin);
	clock_always clock(CLK);

	initial
		begin
		clearRegFile = 0;
		#1 clearRegFile = 1;
		#1 LoadInstructionMemory = 1;
		#1 WriteAddIns = 0;
		   InstructionIn = 16'b1000000100000000;
		#1 WriteAddIns = 2;
		   InstructionIn = 16'b1000001000100000;
		#1 WriteAddIns = 4;
		   InstructionIn = 16'b0010001100110001;
		#1 WriteAddIns = 6;
		   InstructionIn = 16'b1110001100101110;
		#1 WriteAddIns = 8;
		   InstructionIn = 16'b1010001001000000;
		
		#1 LoadInstructionMemory = 0;


	#1 $display("out %16b",instruction);
	
	#1 $display("%16b",instruction);
	
	#1 $display("%16b",instruction);
	
	#1 $display("%16b",instruction); 
	
	#1 $display("%16b",instruction);


		end
	
	
endmodule

module ProgramCounter(out,in);
	input [15:0] in;
	output reg [15:0] out;
	
	initial
	begin
	$display("PC Loading");
	out = 0;
	$display("PC Loaded");
	end
	
	always @(*)begin
	#10 out = in;  $display("PC out %16b",out); end
	
endmodule

module clock_always(Out);

output Out;
reg clock;


initial
	clock = 1'b0;

always
	#5 clock = ~clock;

assign Out = clock;

endmodule


module SignExtend(out,in);
	output reg [15:0] out;
	input [3:0]in;
	
	always @(*)
		begin
			if (in[3]==1)
				out = {1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,in};
			else
				out = {1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,in};
		end
		
endmodule


module reg_file(A,B,C,Addr,Baddr,Caddr,load,clear,clk); 

output reg [15:0] A,B;
input [15:0] C;
input [3:0] Addr,Baddr,Caddr;
input load,clear,clk;
reg [15:0] register_file [15:0];

integer i,j;

always @(clear)
	if(!clear)
	begin
		for(i=0;i<16;i=i+1)
		begin
			 register_file [i] <= 0;
			$display(register_file [i]);
		end
	end

always @(posedge clk)
	 if(load & clk & clear)
		begin
			register_file[Caddr] = C;
		end
always @(*)
	if(clear & !load)
	begin
		A <= register_file[Addr];
		B <= register_file[Baddr];
	end
endmodule


module mux2_to_1 #(parameter word_size=16) (out,s,i0,i1);
	output [word_size-1:0] out;
	input [word_size-1:0] i0,i1;
	input s;

	reg [word_size-1:0]tempout;
	
	always @(s,i0,i1)
	begin	
	
		case ({s})
			2'd0 : tempout = i0;
			2'd1 : tempout = i1;
			default : $display("Oops Something wrong with mux");	
		endcase
	

	end	
	
	assign out=tempout;

endmodule

module muxquarter #(parameter word_size=4) (out,s,i0,i1);
//	output [word_size-1:0] out;
//	input [word_size-1:0] i0,i1;
	input s;

	output [3:0] out;
	input [3:0] i0,i1;
	
	reg [word_size-1:0]tempout;
	
	always @(s,i0,i1)
	begin	
	
		case ({s})
			2'd0 : tempout = i0;
			2'd1 : tempout = i1;
			default : $display("Oops Something wrong with mux");	
		endcase
	

	end	
	
	assign out=tempout;

endmodule

module ALU(z,c_out,lt,eq,gt,c_in,c,x,y);

	input c_in;
	input [15:0] x,y;
	input [3:0] c;  //without having a seperate 3 bit opcode for ALU having the instructions OPCODE directly (our design descision)

	output reg [15:0] z;
	output reg lt,eq,gt;
	output reg c_out;
	
	always @(*)
		begin
			$display("ALU op %4b",{c[3],c[2],c[1],c[0]});
			case ({c[3],c[2],c[1],c[0]})
			
			4'b0000: //AND op
				begin
					z = x & y;
				end
			4'b0001: //OR op
				begin
					z = x | y;
				end
			4'b0110: //Substract op
				begin
					z = x - y;
				end
			4'b0111: //SLT
				begin
					if (x<y) z =1;
					else z = 0;
				end
			4'b0010: //Add op
				begin
					{c_out,z} = x+y+c_in;
				end
			4'b1000: //Add op for LW
				begin
					z = x+y;
				end
			4'b1010: //Add op for SW
				begin
					z = x+y;
				end
			default:
				begin
				$display($time);
				$display("Oops Something Wrong! with ALU OP");
				end
			endcase
		end
		
		always @(*)
			begin
			if (x<y)  begin eq = 0; lt = 1; gt = 0; end
			else if (x == y) begin eq = 1; lt = 0; gt = 0; end
			else begin  eq = 0; lt = 0; gt = 1; end
			end
	
		

endmodule


module DataMemory(DataRead,ReadAddr,WriteData,WriteAddr,MemRead,MemWrite);
// Byte addressed DataMemory
// 2 bytes a word

	output reg [15:0] DataRead;
	input  MemRead,MemWrite;
	input [15:0] ReadAddr,WriteData,WriteAddr;
	
	reg [7:0] memory [65537:0];
	
	always @(*)
		begin
		if(MemWrite)
			{memory[WriteAddr],memory[WriteAddr+1]} = WriteData; //Little Endian
		end
		
	always @(*)
		begin
		if(MemRead)
			DataRead = {memory[ReadAddr],memory[ReadAddr+1]};
		end

endmodule

module InstructionMemory(Load,WriteAdd,in,ReadAdd,out);

output reg [15:0] out;

input Load;
input [15:0] in;
input [15:0] WriteAdd,ReadAdd;

reg [7:0] memory [65537:0];

always @(*)
	begin
		out = {memory[ReadAdd] , memory[ReadAdd+1]}; //Little Endian
	
	end
	
always @(*)
	begin
		{memory[WriteAdd],memory[WriteAdd+1]} = in;
		//	$display(in);
	end
	
endmodule



module control #(parameter width = 16)(RegDest,Branch,Jump,MemRead,MemWrite,MemtoReg,ALU,ALUSrc,RegWrite,instruction,clock);
input [width-1:0] instruction;
input clock;
output RegDest,Branch,Jump,MemRead,MemWrite,MemtoReg,ALUSrc,RegWrite;
output [3:0]ALU;

reg [7:0]controlReg;

always @(posedge clock) 

begin
	case ({instruction[15],instruction[14],instruction[13],instruction[12]})

	4'b0010:begin  controlReg[0] = 0; controlReg[1] = 0; controlReg[2] = 0; controlReg[4] = 0;  controlReg[5] = 0;  controlReg[6] = 0;  controlReg[7] = 1; $display("Add"); end//Add
	4'b0110:begin controlReg[0] = 0; controlReg[1] = 0; controlReg[2] = 0;  controlReg[4] = 0;  controlReg[5] = 0;  controlReg[6] = 0;  controlReg[7] = 1; $display("Sub"); end//Sub
	4'b0000:begin controlReg[0] = 0; controlReg[1] = 0; controlReg[2] = 0;  controlReg[4] = 0;  controlReg[5] = 0;  controlReg[6] = 0;  controlReg[7] = 1; $display("And"); end//And
	4'b0001:begin controlReg[0] = 0; controlReg[1] = 0; controlReg[2] = 0;  controlReg[4] = 0;  controlReg[5] = 0;  controlReg[6] = 0;  controlReg[7] = 1; $display("Or"); end//Or
	4'b0111:begin controlReg[0] = 0; controlReg[1] = 0; controlReg[2] = 0;  controlReg[4] = 0;  controlReg[5] = 0;  controlReg[6] = 0;  controlReg[7] = 1; $display("SLT"); end//SLT
	4'b1000:begin controlReg[0] = 1; controlReg[1] = 0; controlReg[2] = 0;  controlReg[3] = 1;  controlReg[4] = 0;  controlReg[5] = 1;  controlReg[6] = 1; controlReg[7] = 1; $display("LW"); end//LW
	4'b1010:begin controlReg[0] = 1; controlReg[1] = 0; controlReg[2] = 0;  controlReg[3] = 0;  controlReg[4] = 1;  controlReg[6] = 1;  controlReg[7] = 0; $display("SW"); end//SW
	4'b1110:begin controlReg[1] = 1; controlReg[2] = 0; controlReg[4] = 0;  controlReg[5] = 1;  controlReg[6] = 1;  controlReg[7] = 0; $display("BNE"); end//BNE 
	4'b1111:begin controlReg[1] = 0; controlReg[2] = 1; controlReg[4] = 0;  controlReg[7] = 0;  $display("Jump"); end//Jump

	endcase
end
assign RegDest = controlReg[0];
assign Branch = controlReg[1];
assign Jump = controlReg[2];
assign MemRead = controlReg[3]; 
assign MemWrite = controlReg[4]; 
assign MemtoReg = controlReg[5];
assign ALUSrc =controlReg[6];
assign RegWrite = controlReg[7];
assign ALU = instruction[15:12];
endmodule
