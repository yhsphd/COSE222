module and2 (
    a,
    b,
    c
);
  input [1:0] a, b;
  output [1:0] c;

  assign c = a & b;
endmodule
