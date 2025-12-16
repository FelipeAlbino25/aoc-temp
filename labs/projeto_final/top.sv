module top(
  input CLOCK_50, 
  input [3:0] KEY, 
  input [9:0] SW,
  output reg [9:0] LEDR,
  output [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0
);
  
  wire memwrite, clk, reset;
  
  wire [31:0] pc, instr;
  wire [31:0] writedata, addr, readdata;
  wire [31:0] MEM_readdata;
  wire [31:0] IO_readdata;
  
  integer counter;  
  always @(posedge CLOCK_50) 
      counter <= counter + 1;
  
  assign clk = counter[22]; 
  assign reset = ~KEY[0]; 
    
  // FIOS DOS CONTADORES 

   wire [8:0] jumpCounter;
   wire [8:0] memAccessCounter;
   wire [8:0] cpuCycleCounter;
   wire [8:0] instrNumberCounter;
   wire [8:0] stallCounter;      
   wire [8:0] brTakenCounter;    
   wire [8:0] brNotTakenCounter; 
   wire [8:0] loadCounter;       
   wire [8:0] storeCounter;      

   reg [8:0] userInteractionCounter; // Calculado aqui no top

   wire [8:0] flushCounter;
    wire [8:0] fwdCounter;
    wire [8:0] aluCounter;

  // PROCESSADOR PIPELINE
  riscvpipeline cpu(
    .clk(clk), 
    .reset(reset), 
    .PC(pc),                
    .Instr(instr),          
    .Address(addr),         
    .WriteData(writedata),  
    .MemWrite(memwrite),    
    .ReadData(readdata),
    // Conexão da Telemetria Completa
    .jumps(jumpCounter),
    .memAccess(memAccessCounter),
    .cpuCycle(cpuCycleCounter),
    .intructions(instrNumberCounter),
    .stallCount(stallCounter),
    .brTaken(brTakenCounter),
    .brNotTaken(brNotTakenCounter),
    .loadCount(loadCounter),
    .storeCount(storeCounter),
    .flushCount(flushCounter),
    .fwdCount(fwdCounter),
    .aluCount(aluCounter)
  );

  // ==========================================================
  // MEMÓRIA E PERIFÉRICOS
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

 reg prev_reset;
  reg prev_key1;

  always @(posedge clk) begin
      prev_reset <= reset; 
      prev_key1  <= KEY[1];

      // Detecta Borda de Subida no Reset (Apertou KEY[0])
      // (Atual é 1, Anterior era 0)
      if (reset && !prev_reset) 
          userInteractionCounter <= userInteractionCounter + 1;
      
      // (Apertou KEY[1])
      // (Atual é 0, Anterior era 1 - pois é active low)
      else if (!KEY[1] && prev_key1) 
          userInteractionCounter <= userInteractionCounter + 1;
  end
  
  wire [1:0] displayPage = SW[1:0];

  reg [8:0] valorHex0; // Vai para HEX 1 e 0
  reg [8:0] valorHex1; // Vai para HEX 3 e 2
  reg [8:0] valorHex2; // Vai para HEX 5 e 4

  always @(*) begin
    case (displayPage)
        // PÁGINA 0: VISÃO GERAL & CPI (Cycles per Instruction)
        // HEX5/4: Ciclos Totais
        // HEX3/2: Instruções Totais
        // HEX1/0: Interações do Usuário
        2'b00: begin
            valorHex2 = cpuCycleCounter;
            valorHex1 = instrNumberCounter;
            valorHex0 = userInteractionCounter;
        end

        // PÁGINA 1: BRANCH PREDICTION
        // HEX5/4: Jumps Totais (Incondicional + Condicional Taken)
        // HEX3/2: Branches Condicionais TOMADOS
        // HEX1/0: Branches Condicionais NÃO TOMADOS
        2'b01: begin
            valorHex2 = jumpCounter;
            valorHex1 = brTakenCounter;
            valorHex0 = brNotTakenCounter;
        end

        // PÁGINA 2: MEMÓRIA & HAZARDS
        // HEX5/4: Quantidade de Stalls (Bolhas)
        // HEX3/2: Total de Loads
        // HEX1/0: Total de Stores
        2'b10: begin
            valorHex2 = stallCounter;
            valorHex1 = loadCounter;
            valorHex0 = storeCounter;
        end

        // PÁGINA 3: DEBUG EXTRA
        // HEX5/4: Quantos Flushes (Branches que custaram caro)
        // HEX3/2: Quantos Forwardings (Stalls economizados)
        // HEX1/0: Quantas instruções Aritméticas (Cálculo puro)
        2'b11: begin
            valorHex2 = flushCounter;
            valorHex1 = fwdCounter;
            valorHex0 = aluCounter;
        end
    endcase
  end

  // Instanciação dos decodificadores
  dec7seg h0(valorHex0[3:0], HEX0);
  dec7seg h1(valorHex0[7:4], HEX1);
  
  dec7seg h2(valorHex1[3:0], HEX2);
  dec7seg h3(valorHex1[7:4], HEX3);
  
  dec7seg h4(valorHex2[3:0], HEX4);
  dec7seg h5(valorHex2[7:4], HEX5);

  // Lógica de IO (LEDs, etc) mantida
  reg [23:0] hex_digits_dummy; 

  always @(posedge clk)
    if (memwrite & isIO) begin 
      if (addr[2]) LEDR <= writedata[9:0];
      if (addr[3]) hex_digits_dummy <= writedata[23:0]; 
    end
  
  assign IO_readdata = addr[4] ? {28'b0, KEY} :
                       addr[5] ? {22'b0,  SW} : 
                                  32'b0;
                                  
  assign readdata = isIO ? IO_readdata : MEM_readdata; 

endmodule