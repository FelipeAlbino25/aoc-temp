module top(
  input CLOCK_50, // 50 MHz clock
  output reg [9:0] LEDR,
  output [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
  
  wire memwrite, clk, reset;
  wire [31:0] pc, instr;
  wire [31:0] writedata, addr, readdata;
  integer counter = 0;  
  always @(posedge CLOCK_50) 
      counter <= counter + 1;
  assign clk = counter[0]; // 50MHz / 2^22 = 11.9 Hz

  // power-on reset
  power_on_reset por(clk, reset);
    
  // microprocessor
  riscvmulti cpu(clk, reset, addr, writedata, memwrite, readdata);

  // memory 
  mem ram(clk, memwrite, addr, writedata, readdata);

  // memory-mapped i/o
  wire isIO  = addr[8]; // 0x0000_0100
  wire isRAM = !isIO;
  localparam IO_LEDS_bit = 2; // 0x0000_0104
  localparam IO_HEX_bit  = 3; // 0x0000_0108
  reg [23:0] hex_digits; // memory-mapped I/O register for HEX
  dec7seg hex0(hex_digits[ 3: 0], HEX0);
  dec7seg hex1(hex_digits[ 7: 4], HEX1);
  dec7seg hex2(hex_digits[11: 8], HEX2);
  dec7seg hex3(hex_digits[15:12], HEX3);
  dec7seg hex4(hex_digits[19:16], HEX4);
  dec7seg hex5(hex_digits[23:20], HEX5);
  always @(posedge clk)
    if (memwrite & isIO) begin // memory-mapped I/O
      if (addr[IO_LEDS_bit])
        LEDR <= writedata;
      if (addr[IO_HEX_bit])
        hex_digits <= writedata;
  end
endmodule
