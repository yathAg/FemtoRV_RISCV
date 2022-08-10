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
  initial
    begin
      PC = 0;

      // add x0, x0, x0
      //                   rs2   rs1  add  rd   ALUREG
      instr = 32'b0000000_00000_00000_000_00000_0110011;
      // add x1, x0, x0
      //                    rs2   rs1  add  rd  ALUREG
      MEM[0] = 32'b0000000_00000_00000_000_00001_0110011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[1] = 32'b000000000001_00001_000_00001_0010011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[2] = 32'b000000000001_00001_000_00001_0010011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[3] = 32'b000000000001_00001_000_00001_0010011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[4] = 32'b000000000001_00001_000_00001_0010011;
      // lw x2,0(x1)
      //             imm         rs1   w   rd   LOAD
      MEM[5] = 32'b000000000000_00001_010_00010_0000011;
      // sw x2,0(x1)
      //             imm   rs2   rs1   w   imm  STORE
      MEM[6] = 32'b000000_00001_00010_010_00000_0100011;

      // ebreak
      //                                        SYSTEM
      MEM[7] = 32'b000000000001_00000_000_00000_1110011;

    end

  // RV32I instruction set
  wire is_LUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm
  wire is_AUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
  wire is_JAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
  wire is_JALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
  wire is_BRANCH  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
  wire is_LOAD    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
  wire is_STORE   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
  wire is_ALUI    =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
  wire is_ALUR    =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2
  wire is_FENCE   =  (instr[6:0] == 7'b0001111);
  wire is_SYSTEM  =  (instr[6:0] == 7'b1110011); // special

  // The 5 immediate formats
  wire [31:0] I_imm = {{21{instr[31]}}, instr[30:20]  };
  wire [31:0] S_imm = {{21{instr[31]}}, instr[30:25], instr[11:7] };
  wire [31:0] B_imm = {{20{instr[31]}}, instr[7],     instr[30:25] , instr[11:8],1'b0};
  wire [31:0] U_imm = {    instr[31],   instr[30:12], {12{1'b0}}  };
  wire [31:0] J_imm = {{12{instr[31]}}, instr[19:12], instr[20]    , instr[30:21],1'b0};

  // Source and destination registers
  wire [4:0] rs1_Id = instr[19:15];
  wire [4:0] rs2_Id = instr[24:20];
  wire [4:0] rd_Id  = instr[11:7];

  // function codes
  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];

  // The registers bank
  reg [15:0] RegisterBank [0:31];

  //alu registers
  reg [31:0] rs1;
  reg [31:0] rs2;

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
      if(writeback_En && rd_Id != 0)
      begin
        RegisterBank[rd_Id] <= writeback_Data;
      end

      case(state)
        FETCH_INSTR: begin
          instr <= MEM[PC];
          state <= FETCH_REGS;
        end

        FETCH_REGS: begin
          rs1 <= RegisterBank[rs1_Id];
          rs2 <= RegisterBank[rs2_Id];
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
      if(state == FETCH_REGS)
        begin
          $display("PC=%0d",PC);
          case (1'b1)
            is_ALUR: $display(
              "ALUreg rd=%d rs1=%d rs2=%d funct3=%b",
              rd_Id, rs1_Id, rs2_Id, funct3 );

            is_ALUI: $display(
            	"ALUimm rd=%d rs1=%d imm=%0d funct3=%b",
              rd_Id, rs1_Id, I_imm, funct3);

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
