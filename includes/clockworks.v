//`include "femtopll.v"


module Clockworks
(
   input  CLK, // clock pin of the board
   input  RESET, // reset pin of the board
   output clk,   // (optionally divided) clock for the design.
                 // divided if SLOW is different from zero.
   output reset // (optionally timed) negative reset for the design
);
  parameter SLOW=0;

  generate

  //  if(SLOW != 0) begin
      `ifdef BENCH
        localparam slow_bit=SLOW-4;
      `else
        localparam slow_bit=SLOW;
      `endif

      reg [slow_bit:0] slow_CLK = 0;
      always @(posedge CLK)
      begin
      	slow_CLK <= slow_CLK + 1;
      end

      assign clk = slow_CLK[slow_bit];
      assign reset = RESET;

    //end

    // else begin
    //   `ifdef CPU_FREQ
    //           femtoPLL #(
    //             .freq(`CPU_FREQ)
    //           ) pll(
    //              .pclk(CLK),
    //              .clk(clk)
    //   	);
    //   `else
    //           assign clk=CLK;
    //   `endif
    //
    // 	reg [15:0] 	    reset_cnt = 0;
    // 	assign reset = &reset_cnt;
    //
    // 	always @(posedge clk,posedge RESET) begin
    // 	  if(RESET) begin
    // 	    reset_cnt <= 0;
    // 	  end
    //
    //     else begin
    // 	       /* verilator lint_off WIDTH */
    // 	       reset_cnt <= reset_cnt + reset;
    // 	       /* verilator lint_on WIDTH */
    // 	    end
    // 	end
    // end


  endgenerate

endmodule
