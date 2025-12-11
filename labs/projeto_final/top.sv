module top(
  input CLOCK_50, 
  input [3:0] KEY, 
  input [9:0] SW,
  output reg [9:0] LEDR,
  output [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0
);
  
  wire memwrite, clk, reset;
  
  // Fios do processador
  wire [31:0] pc, instr;
  wire [31:0] writedata, addr, readdata;
  
  // Fios auxiliares de memória e IO
  wire [31:0] MEM_readdata;   // Dado vindo da RAM
  wire [31:0] IO_readdata;    // Dado vindo dos periféricos
  
  integer counter;  
  always @(posedge CLOCK_50) 
      counter <= counter + 1;
  
  assign clk = counter[23]; // Clock reduzido
  assign reset = ~KEY[0];   // Reset ativo baixo
    
  // ==========================================================
  // FIOS DOS CONTADORES (TELEMETRIA)
  // ==========================================================
   wire [8:0] jumpCounter;
   wire [8:0] memAccessCounter;
   wire [8:0] cpuCycleCounter;
   wire [8:0] instrNumberCounter;
   wire [8:0] userInteractionCounter;

  // ==========================================================
  // PROCESSADOR PIPELINE
  // ==========================================================
  riscvpipeline cpu(
    .clk(clk), 
    .reset(reset), 
    .PC(pc),                
    .Instr(instr),          
    .Address(addr),         
    .WriteData(writedata),  
    .MemWrite(memwrite),    
    .ReadData(readdata),
    // Conexão da Telemetria
    .jumps(jumpCounter),
    .memAccess(memAccessCounter),
    .cpuCycle(cpuCycleCounter),
    .intructions(instrNumberCounter),   
  );

  // ==========================================================
  // MEMÓRIA UNIFICADA (DUAL PORT)
  // ==========================================================
  wire isIO  = addr[9]; 
  wire isRAM = !isIO;
  
  mem ram(
    .clk(clk),
    .we(memwrite & isRAM),    
    .a_instr(pc),
    .rd_instr(instr),
    .a_data(addr),
    .wd_data(writedata),
    .rd_data(MEM_readdata)
  );

  //  USER INTERACTIONS 
  always @(posedge reset) 
    userInteractionCounter <= userInteractionCounter + 1;

  // ==========================================================
  // VISUALIZAÇÃO NOS DISPLAYS (HEX)
  // ==========================================================
  
  wire hexControl;
  assign hexControl = SW[0]; // SW[0] seleciona qual grupo de contadores ver

  // MUX para selecionar o que mostrar
  // Se SW[0] == 1: Mostra Jumps, MemAccess, Cycles
  // Se SW[0] == 0: Mostra Instruções, Interações, 00
  wire [8:0] valorHex0;
  wire [8:0] valorHex1;
  wire [8:0] valorHex2;

  assign valorHex0 = hexControl ? jumpCounter      : instrNumberCounter;
  assign valorHex1 = hexControl ? memAccessCounter : userInteractionCounter;
  assign valorHex2 = hexControl ? cpuCycleCounter  : 9'b0;

  // Instanciação dos decodificadores
  // HEX 1 e 0 mostram o primeiro contador
  dec7seg h0(valorHex0[3:0], HEX0);
  dec7seg h1(valorHex0[7:4], HEX1); // Bit 8 é ignorado se o contador for > 255 visualmente, ou truncado
  
  // HEX 3 e 2 mostram o segundo contador
  dec7seg h2(valorHex1[3:0], HEX2);
  dec7seg h3(valorHex1[7:4], HEX3);
  
  // HEX 5 e 4 mostram o terceiro contador
  dec7seg h4(valorHex2[3:0], HEX4);
  dec7seg h5(valorHex2[7:4], HEX5);

  // ==========================================================
  // MEMORY-MAPPED I/O
  // ==========================================================
  localparam IO_LEDS_bit = 2; 
  localparam IO_HEX_bit  = 3; 
  localparam IO_KEY_bit  = 4; 
  localparam IO_SW_bit   = 5; 
  
  reg [23:0] hex_digits; 

  // Nota: Você estava sobrescrevendo os HEX com dec7seg acima. 
  // O hardware FPGA só tem um conjunto de pinos HEX.
  // SE você quiser usar o Output Map para escrever nos HEX via software (instrução SW),
  // você teria um conflito aqui. 
  // Como você quer ver os CONTADORES, removi a lógica que ligava 'hex_digits' aos pinos HEX.
  // O registrador 'hex_digits' ainda existe e pode ser escrito pelo processador, 
  // mas não aparecerá visualmente pois os contadores têm prioridade.

  always @(posedge clk)
    if (memwrite & isIO) begin 
      if (addr[IO_LEDS_bit])
        LEDR <= writedata[9:0];
      if (addr[IO_HEX_bit])
        hex_digits <= writedata[23:0]; // O processador guarda o valor, mas não mostra
    end
  
  assign IO_readdata = addr[IO_KEY_bit] ? {28'b0, KEY} :
                       addr[ IO_SW_bit] ? {22'b0,  SW} : 
                                          32'b0;
                                          
  assign readdata = isIO ? IO_readdata : MEM_readdata; 

endmodule