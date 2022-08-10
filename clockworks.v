
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
  endgenerate

endmodule
