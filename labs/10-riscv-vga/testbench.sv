module testbench();
  logic        clk, memwrite;
  logic [31:0] pc, instr;
  logic [31:0] writedata, addr, readdata;
  logic [ 9:0]  leds;

  top dut (
    .CLOCK_50 (clk),
    .LEDR     (leds)
  );
    
  // initialize test
  initial
    begin
      $dumpfile("dump.vcd"); $dumpvars(0);
      $monitor("%5t LEDR=%b PC=%h instr=%h aluIn1=%h aluIn2=%h addr=%h writedata=%h memwrite=%b readdata=%h writeBackData=%h", $time, leds, dut.cpu.PC, dut.cpu.instr, dut.cpu.SrcA, dut.cpu.SrcB, dut.addr, dut.writedata, dut.memwrite, dut.readdata, dut.cpu.writeBackData);
      #120000 $writememh("riscv.out", dut.ram.RAM);
      $writememh("cpu_regs.out", dut.cpu.RegisterBank);
      $finish;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    if (dut.memwrite)
      if (dut.writedata === 32'h6d73e55f) begin
        #50 $display("Simulation succeeded!");
        $writememh("riscv.out", dut.ram.RAM);
        $finish;
      end
endmodule