`include "clockworks.v"

module SOC (
    input  CLK,        // system clock
    input  RESET,      // reset button
    output [3:0] LEDS, // system LEDs
    input  RXD,        // UART receive
    output TXD         // UART transmit
  );

  reg [4:0] leds;
  assign LEDS = leds;

  wire clk;    // internal clock
  wire reset; // internal reset signal, goes low on reset

  reg [31:0] MEM[0:255];
  reg [31:0] PC ; //Program counter
  reg [31:0] instr;

  `include "riscv_assembly.v"

 //instruction memory
  initial begin
    PC = 0;
    ADD(x0,x0,x0);
    ADD(x1,x0,x0);
    ADDI(x1,x1,1);
    ADDI(x1,x1,1);
    ADDI(x1,x1,1);
    ADDI(x1,x1,1);
    ADD(x2,x1,x0);
    ADD(x3,x1,x2);
    SRLI(x3,x3,3);
    SLLI(x3,x3,31);
    SRAI(x3,x3,5);  //0100000,00101,00011,101,00011,0010011
    SRLI(x1,x3,26);
    EBREAK();
  end

  // RV32 Base opcode defination
  wire is_LOAD    =  (instr[6:0] == 7'b0000011); // rd <- mem[src1_value+Iimm]
  wire is_STORE   =  (instr[6:0] == 7'b0100011); // mem[src1_value+Simm] <- src2_value
  wire is_BRANCH  =  (instr[6:0] == 7'b1100011); // if(src1_value OP src2_value) PC<-PC+Bimm

  wire is_JALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-src1_value+Iimm

  wire is_FENCE   =  (instr[6:0] == 7'b0001111);
  wire is_JAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm

  wire is_OPIM    =  (instr[6:0] == 7'b0010011); // rd <- src1_value OP Iimm
  wire is_OP      =  (instr[6:0] == 7'b0110011); // rd <- src1_value OP src2_value
  wire is_SYSTEM  =  (instr[6:0] == 7'b1110011); // special

  wire is_AUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
  wire is_LUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm

  // The 5 immediate formats
  wire [31:0] I_imm = {{21{instr[31]}}, instr[30:20]  };
  wire [31:0] S_imm = {{21{instr[31]}}, instr[30:25], instr[11:7] };
  wire [31:0] B_imm = {{20{instr[31]}}, instr[7],     instr[30:25] , instr[11:8],1'b0};
  wire [31:0] U_imm = {    instr[31],   instr[30:12], {12{1'b0}}  };
  wire [31:0] J_imm = {{12{instr[31]}}, instr[19:12], instr[20]    , instr[30:21],1'b0};

 // instruction fields
  wire [4:0] rs1 = instr[19:15];
  wire [4:0] rs2 = instr[24:20];
  wire [4:0] rd  = instr[11:7];

  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];
  wire [6:0] opcode = instr[6:0];

  //instruction decoder
  //RISCV32I base Instruction set

  wire [10:0] dec_bits = {instr[30],funct3,opcode};

  wire [10:0] is_lui   =  11'bx_xxx_0110111;
  wire [10:0] is_auipc =  11'bx_xxx_0010111;
  wire [10:0] is_jal   =  11'bx_xxx_1101111;

  wire [10:0] is_jalr  =  11'bx_000_1100111;
  wire [10:0] is_beq   =  11'bx_000_1100011;
  wire [10:0] is_bne   =  11'bx_001_1100011;
  wire [10:0] is_blt   =  11'bx_100_1100011;
  wire [10:0] is_bge   =  11'bx_101_1100011;
  wire [10:0] is_bltu  =  11'bx_110_1100011;
  wire [10:0] is_bgeu  =  11'bx_111_1100011;

  wire [10:0] is_load  =  11'bx_xxx_0000011;
  wire [10:0] is_addi  =  11'bx_000_0010011;
  wire [10:0] is_slti  =  11'bx_010_0010011;
  wire [10:0] is_sltiu =  11'bx_011_0010011;
  wire [10:0] is_xori  =  11'bx_100_0010011;
  wire [10:0] is_ori   =  11'bx_110_0010011;
  wire [10:0] is_andi  =  11'bx_111_0010011;

  wire [10:0] is_slli  =  11'b0_001_0010011;
  wire [10:0] is_srli  =  11'b0_101_0010011;
  wire [10:0] is_srai  =  11'b1_101_0010011;
  wire [10:0] is_add   =  11'b0_000_0110011;
  wire [10:0] is_sub   =  11'b1_000_0110011;
  wire [10:0] is_sll   =  11'b0_001_0110011;
  wire [10:0] is_slt   =  11'b0_010_0110011;
  wire [10:0] is_sltu  =  11'b0_011_0110011;
  wire [10:0] is_xor   =  11'b0_100_0110011;
  wire [10:0] is_srl   =  11'b0_101_0110011;
  wire [10:0] is_sra   =  11'b1_101_0110011;
  wire [10:0] is_or    =  11'b0_110_0110011;
  wire [10:0] is_and   =  11'b0_111_0110011;

  wire [10:0] is_fence    =  11'b0_111_0110011;
  wire [10:0] is_ecall    =  11'b0_111_0110011;
  wire [10:0] is_ebreak   =  11'b0_111_0110011;

  // The registers bank
  reg [15:0] RegisterBank [0:31];

  //alu registers
  reg [31:0] src1_value;
  reg [31:0] src2_value;
  reg [31:0] alu_out;

  wire [31:0] writeback_data;
  wire        writeback_en;

  reg [31:0] sltu_rslt ;
  reg [31:0] sltiu_rslt ;

  reg [63:0] sext_src1 ;
  reg [63:0] sra_rslt ;
  reg [63:0] srai_rslt ;

  // ALU
  always @ ( * ) begin

    sltu_rslt = {31'b0, src1_value < src2_value};
    sltiu_rslt = {31'b0, src1_value < I_imm};

    sext_src1 = { {32{src1_value[31]}},src1_value};
    sra_rslt = sext_src1 >> src2_value[4:0];
    srai_rslt = sext_src1 >> instr[24:20];

    casex (dec_bits)

      is_addi : alu_out = src1_value +  I_imm;
      is_slti : alu_out = (( src1_value[31]==I_imm[31] ) ? sltiu_rslt : {31'b0, src1_value[31]} );
      is_sltiu: alu_out = sltiu_rslt;
      is_xori : alu_out = src1_value ^  I_imm;
      is_ori  : alu_out = src1_value |  I_imm;
      is_andi : alu_out = src1_value &  I_imm;

      is_slli : alu_out = src1_value << instr[24:20];
      is_srli : alu_out = src1_value >> I_imm[5:0];
      is_srai : alu_out = srai_rslt[31:0];
      is_add  : alu_out = src1_value + src2_value;
      is_sub  : alu_out = src1_value - src2_value;
      is_sll  : alu_out = src1_value << rs2[4:0];
      is_slt  : alu_out = (( src1_value[31] == src2_value[31] ) ? sltu_rslt : {31'b0, src1_value[31]} ) ;
      is_sltu : alu_out = sltu_rslt;
      is_xor  : alu_out = src1_value ^ src2_value;
      is_srl  : alu_out = src1_value >> src2_value[5:0] ;
      is_sra  : alu_out = sra_rslt[31:0];
      is_or   : alu_out = src1_value | src2_value;
      is_and  : alu_out = src1_value & src2_value;

      default : alu_out = 32'b0;
    endcase

  end


 `ifdef BENCH
    integer i;
    initial begin
      for(i=0; i<16; ++i) begin
        RegisterBank[i] = 0;
      end
    end
 `endif

 //state machine

  localparam  FETCH_INSTR = 0;
  localparam  FETCH_REGS  = 1;
  localparam  EXECUTE     = 2;
  reg [1:0] state = FETCH_INSTR;

  assign writeback_data = alu_out;
  assign writeback_en = (state == EXECUTE && (is_OP || is_OPIM));


  always @(posedge clk) begin
    if(reset) begin
      PC <= 0;
      state <= FETCH_INSTR;
      //instr <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
    end

    else begin
      if(writeback_en && rd != 0)
      begin
        RegisterBank[rd] <= writeback_data;

        if(rd == 1) begin
  	      leds <= writeback_data;
  	    end

        `ifdef BENCH
  	      $display("x%0d <= %b",rd,writeback_data);
        `endif
      end

      case(state)
        FETCH_INSTR: begin
          instr <= MEM[PC[31:2]];
          state <= FETCH_REGS;
        end

        FETCH_REGS: begin
          src1_value <= RegisterBank[rs1];
          src2_value <= RegisterBank[rs2];
          state <= EXECUTE;
        end

        EXECUTE: begin
          if (!is_SYSTEM) begin
            PC <= PC + 4;
          end
          state <= FETCH_INSTR;

          `ifdef BENCH
            if(is_SYSTEM) $finish();
          `endif
        end
      endcase
    end
  end

  //assign LEDS = is_SYSTEM ? 31 : {PC[0],is_OP,is_OPIM,is_STORE};

  `ifdef BENCH
    always @(*)
      //dec_bits ={instr[30],funct3,opcode};
      if(state == FETCH_REGS) begin
        $display("");
        $display("PC=%0d",PC);
        $display("dec_bits=%b",dec_bits);
        $display("instruction =%b\n",instr);

        casex (dec_bits)

          is_lui   : $write("lUI");
          is_auipc : $write("AUIPC");
          is_jal   : $write("JAL");

          is_jalr  : $write("JALR");
          is_beq   : $write("BEQ");
          is_bne   : $write("BNE");
          is_blt   : $write("BLT");
          is_bge   : $write("BGE");
          is_bltu  : $write("BLTU");
          is_bgeu  : $write("BGEU");

          is_load  : $write("LOAD");

          is_addi  : $write("ADDI");
          is_slti  : $write("SLTI");
          is_sltiu : $write("SLTIU");
          is_xori  : $write("XORI");
          is_ori   : $write("ORI");
          is_andi  : $write("ANDI");

          is_slli  : $write("SLLI");
          is_srli  : $write("SRLI");
          is_srai  : $write("SRAI");
          is_add   : $write("ADD");
          is_sub   : $write("SUB");
          is_sll   : $write("SLL");
          is_slt   : $write("SLT");
          is_sltu  : $write("SLTU");
          is_xor   : $write("XOR");
          is_srl   : $write("SRL");
          is_sra   : $write("SRA");
          is_or    : $write("OR");
          is_and   : $write("AND");

          is_fence    : $write("FENCE");
          is_ecall    : $write("ECALL");
          is_ebreak   : $write("EBREAK");

        endcase

        case (1'b1)
          is_OP: $display(
            " rd=%d rs1=%d rs2=%d ",
            rd, rs1, rs2 );

          is_OPIM: $display(
          	" rd=%d rs1=%d imm=%0d funct3=%b ",
            rd, rs1, I_imm, funct3);

        endcase

        if(is_SYSTEM) begin
     	    $finish();
     	  end
      end
  `endif

  Clockworks #(.SLOW(19))
    CW(
      .CLK(CLK),
      .RESET(RESET),
      .clk(clk),
      .reset(reset)
    );

  assign TXD  = 1'b0; // not used for now
endmodule
