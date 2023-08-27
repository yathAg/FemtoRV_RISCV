`include "includes/clockworks.v"
`include "includes/emitter_uart.v"
`include "includes/defines.v"
`include "includes/spi_flash.v"

module Memory (
  input              clk,
  input       [31:0] mem_addr,      // Address to be read
  output reg  [31:0] mem_rdata,     // Data Read
  input              mem_rstrb,     // Goes High when the processor wants to Read
  input       [31:0] mem_wdata,     // Data to be written
  input       [3:0]  mem_wmask      // Mask for writing the 4 Bytes
  );

   reg [31:0] MEM [0:1535]; // 1536 4-bytes words = 6 Kb of RAM in total

  initial begin
      $readmemh("firmware.hex",MEM);
  end

  // `ifdef BENCH
  //   localparam slow_bit=12;
  // `else
  //   localparam slow_bit=19;
  // `endif

  // // Memory-mapped IO in IO page, 1-hot addressing in word address.
  // localparam  IO_LEDS_bit   = 0;  // write led bits
  // localparam  IO_UART_DAT_bit = 1;  // write data to send 8 bits
  // localparam  IO_UART_CNTL_bit = 2;  // Read Status

  // // Converts an IO_xxx_bit constant into an offset in IO page.
  // function [31:0] IO_BIT_TO_OFFSET;
  //   input  [31:0] bit_id;
  //   begin
  //     IO_BIT_TO_OFFSET = 1 << (bit_id + 2);
  //   end
  // endfunction


  // `include "includes/riscv_assembly.v"
  // // ******************code here**************

  //    `define mandel_shift 10
  //    `define mandel_mul (1 << `mandel_shift)
  //    `define xmin (-2*`mandel_mul)
  //    `define xmax ( 2*`mandel_mul)
  //    `define ymin (-2*`mandel_mul)
  //    `define ymax ( 2*`mandel_mul)
  //    `define dx ((`xmax-`xmin)/80)
  //    `define dy ((`ymax-`ymin)/80)
  //    `define norm_max (4 << `mandel_shift)

  //    integer    mandelstart_ = 12;
  //    integer    blink_       = 16;
  //    integer    loop_y_      = 76;
  //    integer    loop_x_      = 84;
  //    integer    loop_Z_      = 96;
  //    integer    exit_Z_      = 188;
  //    integer    wait_        = 264;
  //    integer    wait_L0_     = 272;
  //    integer    putc_        = 284;
  //    integer    putc_L0_     = 292;
  //    integer    mulsi3_      = 308;
  //    integer    mulsi3_L0_   = 316;
  //    integer    mulsi3_L1_   = 328;

  //    integer    colormap_    = 344;

  //    // X,Y         : s0,s1
  //    // Cr,Ci       : s2,s3
  //    // Zr,Zi       : s4,s5
  //    // Zrr,2Zri,Zii: s6,s7,s8
  //    // cnt: s10
  //    // 128: s11

  //    initial begin
  //       LI(sp,32'h1800);   // End of RAM, 6kB
  //       LI(gp,32'h400000); // IO page

  //    Label(mandelstart_);

  //       // Blink 5 times.
  //       LI(s0,5);
  //    Label(blink_);
  //       LI(a0,5);
  //       SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));
  //       CALL(LabelRef(wait_));
  //       LI(a0,10);
  //       SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));
  //       CALL(LabelRef(wait_));
  //       ADDI(s0,s0,-1);
  //       BNEZ(s0,LabelRef(blink_));
  //       LI(a0,0);
  //       SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));


  //       LI(s1,0);
  //       LI(s3,`xmin);
  //       LI(s11,80);

  //    Label(loop_y_);
  //       LI(s0,0);
  //       LI(s2,`ymin);

  //    Label(loop_x_);
  //       MV(s4,s2); // Z <- C
  //       MV(s5,s3);

  //       LI(s10,9); // iter <- 9

  //    Label(loop_Z_);
  //       MV(a0,s4); // Zrr  <- (Zr*Zr) >> mandel_shift
  //       MV(a1,s4);
  //       CALL(LabelRef(mulsi3_));
  //       SRLI(s6,a0,`mandel_shift);
  //       MV(a0,s4); // Zri <- (Zr*Zi) >> (mandel_shift-1)
  //       MV(a1,s5);
  //       CALL(LabelRef(mulsi3_));
  //       SRAI(s7,a0,`mandel_shift-1);
  //       MV(a0,s5); // Zii <- (Zi*Zi) >> (mandel_shift)
  //       MV(a1,s5);
  //       CALL(LabelRef(mulsi3_));
  //       SRLI(s8,a0,`mandel_shift);
  //       SUB(s4,s6,s8); // Zr <- Zrr - Zii + Cr
  //       ADD(s4,s4,s2);
  //       ADD(s5,s7,s3); // Zi <- 2Zri + Cr

  //       ADD(s6,s6,s8); // if norm > norm max, exit loop
  //       LI(s7,`norm_max);
  //       BGT(s6,s7,LabelRef(exit_Z_));

  //       ADDI(s10,s10,-1);  // iter--, loop if non-zero
  //       BNEZ(s10,LabelRef(loop_Z_));

  //    Label(exit_Z_);
  //       LI(a0,colormap_);
  //       ADD(a0,a0,s10);
  //       LBU(a0,a0,0);
  //       CALL(LabelRef(putc_));

  //       ADDI(s0,s0,1);
  //       ADDI(s2,s2,`dx);
  //       BNE(s0,s11,LabelRef(loop_x_));

  //       LI(a0," ");
  //       CALL(LabelRef(putc_));
  //       LI(a0,"\n");
  //       CALL(LabelRef(putc_));

  //       ADDI(s1,s1,1);
  //       ADDI(s3,s3,`dy);
  //       BNE(s1,s11,LabelRef(loop_y_));


  //       J(LabelRef(mandelstart_));

  //       EBREAK(); // I systematically keep it before functions
  //                 // in case I decide to remove the loop...

  //    Label(wait_);
  //       LI(t0,1);
  //       SLLI(t0,t0,slow_bit);
  //    Label(wait_L0_);
  //       ADDI(t0,t0,-1);
  //       BNEZ(t0,LabelRef(wait_L0_));
  //       RET();

  //    Label(putc_);
  //       // Send character to UART
  //       SW(a0,gp,IO_BIT_TO_OFFSET(IO_UART_DAT_bit));
  //       // Read UART status, and loop until bit 9 (busy sending)
  //       // is zero.
  //       LI(t0,1<<9);
  //    Label(putc_L0_);
  //       LW(t1,gp,IO_BIT_TO_OFFSET(IO_UART_CNTL_bit));
  //       AND(t1,t1,t0);
  //       BNEZ(t1,LabelRef(putc_L0_));
  //       RET();

  //       // Mutiplication routine,
  //       // Input in a0 and a1
  //       // Result in a0
  //    Label(mulsi3_);
  //       MV(a2,a0);
  //       LI(a0,0);
  //    Label(mulsi3_L0_);
  //       ANDI(a3,a1,1);
  //       BEQZ(a3,LabelRef(mulsi3_L1_));
  //       ADD(a0,a0,a2);
  //    Label(mulsi3_L1_);
  //       SRLI(a1,a1,1);
  //       SLLI(a2,a2,1);
  //       BNEZ(a1,LabelRef(mulsi3_L0_));
  //       RET();

  //    Label(colormap_);
  //       DATAB(" ",".",",",":");
  //       DATAB(";","o","x","%");
  //       DATAB("#","@", 0 , 0 );
  //       endASM();
  // end
  // // *****************************************

  wire [29:0] word_addr = mem_addr[31:2];

  always @ ( posedge clk ) begin
    if (mem_rstrb) begin
      mem_rdata <= MEM[word_addr];
    end

  if (mem_wmask[0]) MEM[word_addr][ 7:0 ] <= mem_wdata [ 7:0 ];
  if (mem_wmask[1]) MEM[word_addr][15:8 ] <= mem_wdata [15:8 ];
  if (mem_wmask[2]) MEM[word_addr][23:16] <= mem_wdata [23:16];
  if (mem_wmask[3]) MEM[word_addr][31:24] <= mem_wdata [31:24];

  end
endmodule // memory

module Processor (
    input 	  clk,
    input 	  resetn,
    output [31:0] mem_addr, 
    input [31:0]  mem_rdata,
    input         mem_rbusy,		  
    output 	  mem_rstrb,
    output [31:0] mem_wdata,
    output [3:0]  mem_wmask
);

   // Internal width for addresses.
   //localparam ADDR_WIDTH=24;
   
   reg [31:0] PC=0; // program counter
   reg [31:2] instr;        // current instruction

   // See the table P. 105 in RISC-V manual
   
   // The 10 RISC-V instructions
   wire isALUreg  =  (instr[6:2] == 5'b01100); // rd <- rs1 OP rs2   
   wire isALUimm  =  (instr[6:2] == 5'b00100); // rd <- rs1 OP Iimm
   wire isBranch  =  (instr[6:2] == 5'b11000); // if(rs1 OP rs2) PC<-PC+Bimm
   wire isJALR    =  (instr[6:2] == 5'b11001); // rd <- PC+4; PC<-rs1+Iimm
   wire isJAL     =  (instr[6:2] == 5'b11011); // rd <- PC+4; PC<-PC+Jimm
   wire isAUIPC   =  (instr[6:2] == 5'b00101); // rd <- PC + Uimm
   wire isLUI     =  (instr[6:2] == 5'b01101); // rd <- Uimm   
   wire isLoad    =  (instr[6:2] == 5'b00000); // rd <- mem[rs1+Iimm]
   wire isStore   =  (instr[6:2] == 5'b01000); // mem[rs1+Simm] <- rs2
   wire isSYSTEM  =  (instr[6:2] == 5'b11100); // special

   // The 5 immediate formats
   wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
   wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
   /* verilator lint_off UNUSED */ // MSBs of SBJimms are not used by addr adder.
   wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
   wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
   wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};
   /* verilator lint_on UNUSED */

   // Destination registers
   wire [4:0] rdId  = instr[11:7];
   
   // function codes
   wire [2:0] funct3 = instr[14:12];
   wire [6:0] funct7 = instr[31:25];
   
   // The registers bank
   reg [31:0] RegisterBank [0:31];
   reg [31:0] rs1; // value of source
   reg [31:0] rs2; //  registers.
   wire [31:0] writeBackData; // data to be written to rd
   wire        writeBackEn;   // asserted if data should be written to rd

`ifdef BENCH   
   integer     i;
   initial begin
      for(i=0; i<32; ++i) begin
	 RegisterBank[i] = 0;
      end
   end
`endif   

   // The ALU
   wire [31:0] aluIn1 = rs1;
   wire [31:0] aluIn2 = isALUreg | isBranch ? rs2 : Iimm;

   wire [4:0] shamt = isALUreg ? rs2[4:0] : instr[24:20]; // shift amount

   // The adder is used by both arithmetic instructions and JALR.
   wire [31:0] aluPlus = aluIn1 + aluIn2;

   // Use a single 33 bits subtract to do subtraction and all comparisons
   // (trick borrowed from swapforth/J1)
   wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0,aluIn1} + 33'b1;
   wire        LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
   wire        LTU = aluMinus[32];
   wire        EQ  = (aluMinus[31:0] == 0);

   // Flip a 32 bit word. Used by the shifter (a single shifter for
   // left and right shifts, saves silicium !)
   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
		x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
		x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
		x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(aluIn1) : aluIn1;
   
   /* verilator lint_off WIDTH */
   wire [31:0] shifter = 
               $signed({instr[30] & aluIn1[31], shifter_in}) >>> aluIn2[4:0];
   /* verilator lint_on WIDTH */

   wire [31:0] leftshift = flip32(shifter);
   

   
   // ADD/SUB/ADDI: 
   // funct7[5] is 1 for SUB and 0 for ADD. We need also to test instr[5]
   // to make the difference with ADDI
   //
   // SRLI/SRAI/SRL/SRA: 
   // funct7[5] is 1 for arithmetic shift (SRA/SRAI) and 
   // 0 for logical shift (SRL/SRLI)
   reg [31:0]  aluOut;
   always @(*) begin
      case(funct3)
	3'b000: aluOut = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
	3'b001: aluOut = leftshift;
	3'b010: aluOut = {31'b0, LT};
	3'b011: aluOut = {31'b0, LTU};
	3'b100: aluOut = (aluIn1 ^ aluIn2);
	3'b101: aluOut = shifter;
	3'b110: aluOut = (aluIn1 | aluIn2);
	3'b111: aluOut = (aluIn1 & aluIn2);	
      endcase
   end

   // The predicate for branch instructions
   reg takeBranch;
   always @(*) begin
      case(funct3)
	3'b000: takeBranch = EQ;
	3'b001: takeBranch = !EQ;
	3'b100: takeBranch = LT;
	3'b101: takeBranch = !LT;
	3'b110: takeBranch = LTU;
	3'b111: takeBranch = !LTU;
	default: takeBranch = 1'b0;
      endcase
   end
   

   // Address computation


   /* verilator lint_off WIDTH */

   // An adder used to compute branch address, JAL address and AUIPC.
   // branch->PC+Bimm    AUIPC->PC+Uimm    JAL->PC+Jimm
   // Equivalent to PCplusImm = PC + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm)

   // Note: doing so with ADDR_WIDTH < 32, AUIPC may fail in 
   // some RISC-V compliance tests because one can is supposed to use 
   // it to generate arbitrary 32-bit values (and not only addresses).
   
   wire [31:0] PCplusImm = PC + ( instr[3] ? Jimm[31:0] :
					    instr[4] ? Uimm[31:0] :
				            Bimm[31:0] );
   wire [31:0] PCplus4 = PC+4;
   

   wire [31:0] nextPC = ((isBranch && takeBranch) || isJAL) ? PCplusImm   :
	                                  isJALR   ? {aluPlus[31:1],1'b0} :
	                                             PCplus4;

   wire [31:0] loadstore_addr = rs1 + (isStore ? Simm : Iimm);


   // register write back
   assign writeBackData = (isJAL || isJALR) ? PCplus4   :
			      isLUI         ? Uimm      :
			      isAUIPC       ? PCplusImm :
			      isLoad        ? LOAD_data :
			                      aluOut;
   /* verilator lint_on WIDTH */
   
   wire mem_byteAccess     = funct3[1:0] == 2'b00;
   wire mem_halfwordAccess = funct3[1:0] == 2'b01;


   wire [15:0] LOAD_halfword =
	       loadstore_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];

   wire  [7:0] LOAD_byte =
	       loadstore_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];

   wire LOAD_sign =
	!funct3[2] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

   wire [31:0] LOAD_data =
         mem_byteAccess ? {{24{LOAD_sign}},     LOAD_byte} :
     mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
                          mem_rdata ;
   // Store
   // ------------------------------------------------------------------------

   assign mem_wdata[ 7: 0] = rs2[7:0];
   assign mem_wdata[15: 8] = loadstore_addr[0] ? rs2[7:0]  : rs2[15: 8];
   assign mem_wdata[23:16] = loadstore_addr[1] ? rs2[7:0]  : rs2[23:16];
   assign mem_wdata[31:24] = loadstore_addr[0] ? rs2[7:0]  :
			     loadstore_addr[1] ? rs2[15:8] : rs2[31:24];

   wire [3:0] STORE_wmask =
	      mem_byteAccess      ?
	            (loadstore_addr[1] ?
		          (loadstore_addr[0] ? 4'b1000 : 4'b0100) :
		          (loadstore_addr[0] ? 4'b0010 : 4'b0001)
                    ) :
	      mem_halfwordAccess ?
	            (loadstore_addr[1] ? 4'b1100 : 4'b0011) :
              4'b1111;
   
   // The state machine
   localparam FETCH_INSTR = 0;
   localparam WAIT_INSTR  = 1;
   localparam EXECUTE     = 2;
   localparam WAIT_DATA   = 3;
   reg [1:0] state = FETCH_INSTR;
   
   always @(posedge clk) begin
      if(resetn) begin
	 PC    <= 0;
	 state <= FETCH_INSTR;
      end else begin
	 if(writeBackEn && rdId != 0) begin
	    RegisterBank[rdId] <= writeBackData;
	 end
	 case(state)
	   FETCH_INSTR: begin
	      state <= WAIT_INSTR;
	   end
	   WAIT_INSTR: begin
	      instr <= mem_rdata[31:2];
	      rs1 <= RegisterBank[mem_rdata[19:15]];
	      rs2 <= RegisterBank[mem_rdata[24:20]];
	      state <= EXECUTE;
	   end
	   EXECUTE: begin
	      if(!isSYSTEM) begin
		 /* verilator lint_off WIDTH */
		 PC <= nextPC;
		 /* verilator lint_on WIDTH */		 
	      end
	      state <= isLoad  ? WAIT_DATA : FETCH_INSTR;
`ifdef BENCH      
	      if(isSYSTEM) $finish();
`endif      
	   end
	   WAIT_DATA: begin
	      if(!mem_rbusy) begin
		 state <= FETCH_INSTR;
	      end
	   end
	 endcase 
      end
   end

   assign writeBackEn = (state==EXECUTE && !isBranch && !isStore) ||
			(state==WAIT_DATA) ;

   /* verilator lint_off WIDTH */
   assign mem_addr = (state == WAIT_INSTR || state == FETCH_INSTR) ?
		     PC : loadstore_addr ;
   /* verilator lint_on WIDTH */
   
   assign mem_rstrb = (state == FETCH_INSTR || (state == EXECUTE & isLoad));
   assign mem_wmask = {4{(state == EXECUTE) & isStore}} & STORE_wmask;

endmodule


module SOC (
    input               CLK,        // system clock
    input               RESET,      // reset button
    output reg [4:0]    LEDS, // system LEDs
    input               RXD,        // UART receive
    output              TXD,         // UART transmit
    output 	            SPIFLASH_CLK,  // SPI flash clock
    output 	            SPIFLASH_CS_N, // SPI flash chip select (active low)
    output              SPIFLASH_MOSI,
    input               SPIFLASH_MISO    // SPI flash IO pins
  );

  wire clkout;
  wire resetn;

  wire [31:0] mem_addr;
  wire [31:0] mem_rdata;
  wire        mem_rbusy;
  wire mem_rstrb;
  wire [31:0] mem_wdata;
  wire [3:0]  mem_wmask;

  Processor CPU(
    .clk(clkout),
    .resetn(resetn),   
    .mem_addr(mem_addr),
    .mem_rdata(mem_rdata),
    .mem_rstrb(mem_rstrb),
    .mem_rbusy(mem_rbusy),
    .mem_wdata(mem_wdata),
    .mem_wmask(mem_wmask)
  );

  wire [31:0] ram_rdata;
  wire [29:0] mem_wordaddr = mem_addr[31:2];
  wire isSPIFlash  = mem_addr[25];
  wire is_io = mem_addr[25:24] == 2'b01;
  wire is_ram = !(mem_addr[25] | mem_addr[24]);
  wire mem_wstrb = |mem_wmask;   //bitwise or entire bus

  Memory RAM(
    .clk(clkout),
    .mem_addr(mem_addr),
    .mem_rdata(ram_rdata),
    .mem_rstrb(is_ram & mem_rstrb),
    .mem_wdata(mem_wdata),
    .mem_wmask({4{is_ram}} & mem_wmask)
  );

  wire [31:0] SPIFlash_rdata;
  wire SPIFlash_rbusy;
  MappedSPIFlash SPIFlash(
    .clk(clk),
    .word_address(mem_wordaddr[21:0]),
    .rdata(SPIFlash_rdata),
    .rstrb(isSPIFlash & mem_rstrb),
    .rbusy(SPIFlash_rbusy),
    .CLK(SPIFLASH_CLK),
    .CS_N(SPIFLASH_CS_N),
    .MOSI(SPIFLASH_MOSI),
    .MISO(SPIFLASH_MISO)			   
  );
  // Memory-mapped IO in IO page, 1-hot addressing in word address.
  localparam  IO_LEDS_bit   = 0;  // write led bits
  localparam  IO_UART_DAT_bit = 1;  // write data to send 8 bits
  localparam  IO_UART_CNTL_bit = 2;  // Read Status

  always @ (posedge clkout) begin
    if(is_io & mem_wstrb & mem_wordaddr[IO_LEDS_bit]) begin
      LEDS <= mem_wdata[3:0];
    end
  end

  wire uart_valid = is_io & mem_wstrb & mem_wordaddr[IO_UART_DAT_bit];
  wire uart_ready;

  corescore_emitter_uart #(
    .clk_freq_hz(100*1000000),
    .baud_rate(1000000)
  ) UART(
    .i_clk(clkout),
    .i_rst(resetn),
    .i_data(mem_wdata[7:0]),
    .i_valid(uart_valid),
    .o_ready(uart_ready),
    .o_uart_tx(TXD)
  );

  wire [31:0] io_rdata =
        mem_wordaddr[IO_UART_CNTL_bit] ? {22'b0 , !uart_ready , 9'b0}
                                   : 32'b0
  ;

  assign mem_rdata = is_ram     ? ram_rdata : 
                     isSPIFlash ? SPIFlash_rdata :
                                  io_rdata
  ;

  assign mem_rbusy = SPIFlash_rbusy;
   
  `ifdef BENCH
    always @ (posedge clkout) begin
      if(uart_valid)begin
        $write("%c" , mem_wdata[7:0]);
        $fflush(32'h8000_0001);
      end
    end
  `endif
    Clockworks CW(
      .CLK(CLK),
      .RESET(RESET),
      .clk(clkout),
      .resetn(resetn)
  );
endmodule
