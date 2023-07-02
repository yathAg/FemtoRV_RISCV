`ifdef BENCH
`else
  `include "femtopll.v"
`endif

module Clockworks
(
   input  CLK, // clock pin of the board
   input  RESET, // reset pin of the board
   output clk,   // (optionally divided) clock for the design.
                 // divided if SLOW is different from zero.
   output resetn // (optionally timed) negative reset for the design
);

  generate
    `ifdef BENCH
      assign clk = CLK;
      assign resetn = RESET;
    `else begin
      femtoPLL #(
        .freq(100)
      ) pll(
        .pclk(CLK),
        .clk(clk)
      );
      assign resetn = RESET;
      end
    `endif
    // else

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
