`include "clockworks.v"

module SOC (
    input  CLK,        // system clock
    input  RESET,      // reset button
    output [3:0] LEDS, // system LEDs
    input  RXD,        // UART receive
    output TXD         // UART transmit
);

  wire clk;    // internal clock
  wire reset; // internal reset signal, goes low on reset

  reg [31:0] MEM[0:255];
  reg [31:0] PC ; //Program counter
  reg [31:0] instr;


 //instruction memory
  initial begin
    PC = 0;

    // add x0, x0, x0
    //                   src2_value   src1_value  add  rd   ALUREG
    instr = 32'b0000000_00000_00000_000_00000_0110011;
    // add x1, x0, x0
    //                    src2_value   src1_value  add  rd  ALUREG
    MEM[0] = 32'b0000000_00000_00000_000_00001_0110011;
    // addi x1, x1, 1
    //             imm         src1_value  add  rd   ALUIMM
    MEM[1] = 32'b000000000001_00001_000_00001_0010011;
    // addi x1, x1, 1
    //             imm         src1_value  add  rd   ALUIMM
    MEM[2] = 32'b000000000001_00001_000_00001_0010011;
    // addi x1, x1, 1
    //             imm         src1_value  add  rd   ALUIMM
    MEM[3] = 32'b000000000001_00001_000_00001_0010011;
    // addi x1, x1, 1
    //             imm         src1_value  add  rd   ALUIMM
    MEM[4] = 32'b000000000001_00001_000_00001_0010011;
    // lw x2,0(x1)
    //             imm         src1_value   w   rd   LOAD
    MEM[5] = 32'b000000000000_00001_010_00010_0000011;
    // sw x2,0(x1)
    //             imm   src2_value   src1_value   w   imm  STORE
    MEM[6] = 32'b000000_00001_00010_010_00000_0100011;

    // ebreak
    //                                        SYSTEM
    MEM[7] = 32'b000000000001_00000_000_00000_1110011;

  end

  // RV32I instruction set
  wire is_LUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm
  wire is_AUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
  wire is_JAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
  wire is_JALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-src1_value+Iimm
  wire is_BRANCH  =  (instr[6:0] == 7'b1100011); // if(src1_value OP src2_value) PC<-PC+Bimm
  wire is_LOAD    =  (instr[6:0] == 7'b0000011); // rd <- mem[src1_value+Iimm]
  wire is_STORE   =  (instr[6:0] == 7'b0100011); // mem[src1_value+Simm] <- src2_value
  wire is_ALUI    =  (instr[6:0] == 7'b0010011); // rd <- src1_value OP Iimm
  wire is_ALUR    =  (instr[6:0] == 7'b0110011); // rd <- src1_value OP src2_value
  wire is_FENCE   =  (instr[6:0] == 7'b0001111);
  wire is_SYSTEM  =  (instr[6:0] == 7'b1110011); // special

  // The 5 immediate formats
  wire [31:0] I_imm = {{21{instr[31]}}, instr[30:20]  };
  wire [31:0] S_imm = {{21{instr[31]}}, instr[30:25], instr[11:7] };
  wire [31:0] B_imm = {{20{instr[31]}}, instr[7],     instr[30:25] , instr[11:8],1'b0};
  wire [31:0] U_imm = {    instr[31],   instr[30:12], {12{1'b0}}  };
  wire [31:0] J_imm = {{12{instr[31]}}, instr[19:12], instr[20]    , instr[30:21],1'b0};

  // Source and destination registers
  wire [4:0] rs1 = instr[19:15];
  wire [4:0] rs2 = instr[24:20];
  wire [4:0] rd  = instr[11:7];

  // function codes
  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];
  wire [6:0] opcode = instr[6:0];

  // The registers bank
  reg [15:0] RegisterBank [0:31];


  //instruction decoder

  wire dec_bits ={instr[30],funct3,opcode};

  wire is_lui   =  11'bx_xxx_0110111;
  wire is_auipc =  11'bx_xxx_0010111;
  wire is_jal   =  11'bx_xxx_1101111;

  wire is_jalr  =  11'bx_000_1100111;
  wire is_beq   =  11'bx_000_1100011;
  wire is_bne   =  11'bx_001_1100011;
  wire is_blt   =  11'bx_100_1100011;
  wire is_bge   =  11'bx_101_1100011;
  wire is_bltu  =  11'bx_110_1100011;
  wire is_bgeu  =  11'bx_111_1100011;

  wire is_load  =  11'bx_xxx_0000011;

  wire is_addi  =  11'bx_000_0010011;
  wire is_slti  =  11'bx_010_0010011;
  wire is_sltiu =  11'bx_011_0010011;
  wire is_xori  =  11'bx_100_0010011;
  wire is_ori   =  11'bx_110_0010011;
  wire is_andi  =  11'bx_111_0010011;

  wire is_slli  =  11'b0_001_0010011;
  wire is_srli  =  11'b0_101_0010011;
  wire is_srai  =  11'b1_101_0010011;
  wire is_add   =  11'b0_000_0110011;
  wire is_sub   =  11'b1_000_0110011;
  wire is_sll   =  11'b0_001_0110011;
  wire is_slt   =  11'b0_010_0110011;
  wire is_sltu  =  11'b0_011_0110011;
  wire is_xor   =  11'b0_100_0110011;
  wire is_srl   =  11'b0_101_0110011;
  wire is_sra   =  11'b1_101_0110011;
  wire is_or    =  11'b0_110_0110011;
  wire is_and   =  11'b0_111_0110011;

  //alu registers
  reg [31:0] src1_value;
  reg [31:0] src2_value;
  reg [31:0] alu_out;

  reg [31:0] sltu_rslt ;
  reg [31:0] sltiu_rslt ;

  reg [63:0] sext_src1 ;
  reg [63:0] sra_rslt ;
  reg [63:0] srai_rslt ;

  //alu
  always @ ( * ) begin

    sltu_rslt = {31'b0, src1_value < src2_value};
    sltiu_rslt = {31'b0, src1_value < I_imm};

    sext_src1 = { {32{src1_value[31]}},src1_value};
    sra_rslt = sext_src1 >> src2_value[4:0];
    srai_rslt = sext_src1 >> I_imm[4:0];

    case (dec_bits)

      is_addi : alu_out = src1_value +  I_imm;
      is_slti : alu_out = (( src1_value[31]==I_imm[31] ) ? sltiu_rslt : {31'b0, src1_value[31]} );
      is_sltiu: alu_out = sltiu_rslt;
      is_xori : alu_out = src1_value ^  I_imm;
      is_ori  : alu_out = src1_value |  I_imm;
      is_andi : alu_out = src1_value &  I_imm;
      is_slli : alu_out = src1_value << instr[24:20];
      is_srli : alu_out = src1_value >> I_imm[5:0];
      is_srai : alu_out = srai_rslt;

      is_add  : alu_out = src1_value + src2_value;
      is_sub  : alu_out = src1_value - src2_value;
      is_sll  : alu_out = src1_value << rs2[4:0];
      is_slt  : alu_out = (( src1_value[31] == src2_value[31] ) ? sltu_rslt : {31'b0, src1_value[31]} ) ;
      is_sltu : alu_out = sltu_rslt;
      is_xor  : alu_out = src1_value ^ src2_value;
      is_srl  : alu_out = src1_value >> src2_value[5:0] ;
      is_sra  : alu_out = sra_rslt;
      is_or   : alu_out = src1_value | src2_value;
      is_and  : alu_out = src1_value & src2_value;

      default : alu_out = 32'b0;
    endcase

  end



  //decode variables
  wire [31:0] writeback_Data;
  wire        writeback_En;
  assign writeback_Data = 0; // for now
  assign writeback_En = 0;   // for now

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

  always @(posedge clk) begin
    if(reset) begin
      PC <= 0;
      state <= FETCH_INSTR;
      instr <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
    end

    else begin
      if(writeback_En && rd != 0)
      begin
        RegisterBank[rd] <= writeback_Data;
      end

      case(state)
        FETCH_INSTR: begin
          instr <= MEM[PC];
          state <= FETCH_REGS;
        end

        FETCH_REGS: begin
          src1_value <= RegisterBank[rs1];
          src2_value <= RegisterBank[rs2];
          state <= EXECUTE;
        end

        EXECUTE: begin
          if (!is_SYSTEM) begin
            PC <= PC +1;
          end
          state <= FETCH_INSTR;

          `ifdef BENCH
            if(is_SYSTEM) $finish();
          `endif
        end
      endcase
    end
  end

  assign LEDS = is_SYSTEM ? 31 : {PC[0],is_ALUR,is_ALUI,is_STORE};

  `ifdef BENCH
    always @(posedge clk)
      if(state == FETCH_REGS) begin
        $display("PC=%0d",PC);
        case (1'b1)
          is_ALUR: $display(
            "ALUreg rd=%d src1_value=%d src2_value=%d funct3=%b",
            rd, rs1, rs2, funct3 );

          is_ALUI: $display(
          	"ALUimm rd=%d src1_value=%d imm=%0d funct3=%b",
            rd, rs1, I_imm, funct3);

          is_BRANCH: $display("BRANCH");
          is_JAL:    $display("JAL");
          is_JALR:   $display("JALR");
          is_AUIPC:  $display("AUIPC");
          is_LUI:    $display("LUI");
        	is_LOAD:   $display("LOAD");
        	is_STORE:  $display("STORE");
        	is_SYSTEM: $display("SYSTEM");
        endcase

        if(is_SYSTEM) begin
     	    $finish();
     	  end
      end
  `endif

  Clockworks #(.SLOW(24))
    CW(
      .CLK(CLK),
      .RESET(RESET),
      .clk(clk),
      .reset(reset)
    );

  assign TXD  = 1'b0; // not used for now
endmodule
