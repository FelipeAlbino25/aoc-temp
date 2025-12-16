module riscvpipeline (
  input         clk,
    input         reset,
    output [31:0] PC,
    input  [31:0] Instr,
    output [31:0] Address,  
    output [31:0] WriteData, 
    output        MemWrite,  
    input  [31:0] ReadData,
    
    // --- TELEMETRIA NOVA ---
    output [8:0] jumps,
    output [8:0] memAccess,     // Mantido p/ compatibilidade (soma load+store)
    output [8:0] cpuCycle,
    output [8:0] intructions,
    output [8:0] stallCount,    
    output [8:0] brTaken,       
    output [8:0] brNotTaken,    
    output [8:0] loadCount,     
    output [8:0] storeCount,     
    output [8:0] flushCount,
    output [8:0] fwdCount,
    output [8:0] aluCount  
    );


   /* The 10 "recognizers" for the 10 codeops */
   function isALUreg; input [31:0] I; isALUreg=(I[6:0]==7'b0110011); endfunction
   function isALUimm; input [31:0] I; isALUimm=(I[6:0]==7'b0010011); endfunction
   function isBranch; input [31:0] I; isBranch=(I[6:0]==7'b1100011); endfunction
   function isJALR;   input [31:0] I; isJALR  =(I[6:0]==7'b1100111); endfunction
   function isJAL;    input [31:0] I; isJAL   =(I[6:0]==7'b1101111); endfunction
   function isAUIPC;  input [31:0] I; isAUIPC =(I[6:0]==7'b0010111); endfunction
   function isLUI;    input [31:0] I; isLUI   =(I[6:0]==7'b0110111); endfunction
   function isLoad;   input [31:0] I; isLoad  =(I[6:0]==7'b0000011); endfunction
   function isStore;  input [31:0] I; isStore =(I[6:0]==7'b0100011); endfunction
   function isSYSTEM; input [31:0] I; isSYSTEM=(I[6:0]==7'b1110011); endfunction
   
   /* Register indices */
   function [4:0] rs1Id; input [31:0] I; rs1Id = I[19:15];      endfunction
   function [4:0] rs2Id; input [31:0] I; rs2Id = I[24:20];      endfunction
   function [4:0] shamt; input [31:0] I; shamt = I[24:20];      endfunction
   function [4:0] rdId;  input [31:0] I; rdId  = I[11:7];       endfunction
   function [1:0] csrId; input [31:0] I; csrId = {I[27],I[21]}; endfunction

   /* funct3 and funct7 */
   function [2:0] funct3; input [31:0] I; funct3 = I[14:12]; endfunction
   function [6:0] funct7; input [31:0] I; funct7 = I[31:25]; endfunction

   /* EBREAK and CSRRS instruction "recognizers" */
   function isEBREAK; input [31:0] I; isEBREAK = (isSYSTEM(I) && funct3(I) == 3'b000); endfunction

   /* The 5 immediate formats */
   function [31:0] Uimm; input [31:0] I; Uimm={I[31:12],{12{1'b0}}}; endfunction
   function [31:0] Iimm; input [31:0] I; Iimm={{21{I[31]}},I[30:20]}; endfunction
   function [31:0] Simm; input [31:0] I; Simm={{21{I[31]}},I[30:25],I[11:7]}; endfunction
   function [31:0] Bimm; input [31:0] I; Bimm = {{20{I[31]}},I[7],I[30:25],I[11:8],1'b0}; endfunction
   function [31:0] Jimm; input [31:0] I; Jimm = {{12{I[31]}},I[19:12],I[20],I[30:21],1'b0}; endfunction

   /* Read/Write tests */
   function writesRd; input [31:0] I; writesRd = !isStore(I) && !isBranch(I); endfunction
   function readsRs1; input [31:0] I; readsRs1 = !(isJAL(I) || isAUIPC(I) || isLUI(I)); endfunction
   function readsRs2; input [31:0] I; readsRs2 = isALUreg(I) || isBranch(I) || isStore(I); endfunction

   /******************** Flush ************************************************/
   wire e_flush = E_JumpOrBranch;
   wire d_flush = E_JumpOrBranch;

   // Declaração antecipada dos registradores para uso na lógica de Stall
   reg [31:0] FD_instr; 
   reg [31:0] DE_instr;

   // ---------------- HAZARD DETECTION UNIT (STALL) ---------------- //
   // Detecta Hazard de Load-Use:
   // Se a instrução no estágio EX (DE_instr) é um LOAD...
   // E o destino desse LOAD (rd) é igual a um dos operandos da instrução no estágio ID (FD_instr)...
   // Então precisamos parar o pipeline (Stall).
   
   wire stall = isLoad(DE_instr) && (
                  (readsRs1(FD_instr) && (rs1Id(FD_instr) == rdId(DE_instr)) && (rdId(DE_instr) != 0)) || 
                  (readsRs2(FD_instr) && (rs2Id(FD_instr) == rdId(DE_instr)) && (rdId(DE_instr) != 0))
                );

   // ---------------- FIM HAZARD DETECTION UNIT ---------------- //


/********************** F: Instruction fetch *********************************/
   localparam NOP = 32'b0000000_00000_00000_000_00000_0110011; // ADDI x0, x0, 0
   reg [31:0] F_PC;
   reg [31:0] FD_PC;
   reg        FD_nop;
   assign PC = F_PC;

   /** These two signals come from the Execute stage **/
   wire [31:0] jumpOrBranchAddress;
   wire        jumpOrBranch;

   always @(posedge clk) begin
      // Se houver reset, reinicia.
      if (reset) begin
         F_PC <= 0;
         FD_instr <= NOP;
         FD_PC <= 0;
         FD_nop <= 0;
      end 
      else begin
          // Se houver Stall, congelamos o estágio Fetch e o registrador FD.
          // O PC não incrementa e a instrução em FD não muda.
          if (!stall) begin
             FD_instr <= Instr;
             FD_PC    <= F_PC;
             
             // Atualiza PC (Pula se for Jump/Branch, senão PC+4)
             if (jumpOrBranch)
                F_PC <= jumpOrBranchAddress;
             else
                F_PC <= F_PC + 4;

             FD_nop <= (d_flush);
          end
          // Se estiver em Stall, mas um Jump acontecer (caso raro/borda), o Jump tem prioridade
          // (embora num load-use hazard, o jump não estaria no estágio EX ainda, então isso é seguro)
          else if (jumpOrBranch) begin
              F_PC <= jumpOrBranchAddress;
              FD_nop <= 1'b1; // Flush no stall se houver branch
          end
      end
   end

/************************ D: Instruction decode *******************************/
   reg [31:0] DE_PC;
   reg [31:0] DE_rs1;
   reg [31:0] DE_rs2;

   /* These three signals come from the Writeback stage */
   wire        writeBackEn;
   wire [31:0] writeBackData;
   wire [4:0]  wbRdId;

   reg [31:0] RegisterBank [0:31];
   
   // IMPLEMENTAÇÃO DE WRITE-THROUGH NO BANCO DE REGISTRADORES
   always @(posedge clk) begin
      DE_PC    <= FD_PC;
      
      // INSERÇÃO DA BOLHA (NOP)
      // Se houver stall, reset, flush ou nop anterior, inserimos NOP no estágio EX.
      DE_instr <= (reset || FD_nop || e_flush || stall) ? NOP : FD_instr;

      // Logica para RS1: Write-Through
      if (writeBackEn && (wbRdId == rs1Id(FD_instr)) && (wbRdId != 0))
          DE_rs1 <= writeBackData;
      else
          DE_rs1 <= rs1Id(FD_instr) ? RegisterBank[rs1Id(FD_instr)] : 32'b0;

      // Logica para RS2: Write-Through
      if (writeBackEn && (wbRdId == rs2Id(FD_instr)) && (wbRdId != 0))
          DE_rs2 <= writeBackData;
      else
          DE_rs2 <= rs2Id(FD_instr) ? RegisterBank[rs2Id(FD_instr)] : 32'b0;

      if (writeBackEn && wbRdId != 0)
         RegisterBank[wbRdId] <= writeBackData;
   end

/************************ E: Execute *****************************************/
   reg [31:0] EM_PC;
   reg [31:0] EM_instr;
   reg [31:0] EM_rs2;
   reg [31:0] EM_Eresult;
   reg [31:0] EM_addr;

   // ---------------- FORWARDING UNIT ---------------- //
   
   wire EM_writesRd = writesRd(EM_instr) && (rdId(EM_instr) != 0);
   wire MW_writesRd = writesRd(MW_instr) && (rdId(MW_instr) != 0); 

   // Lógica de Forwarding para RS1
   reg [1:0] forwardA;
   always @(*) begin
       if (EM_writesRd && (rdId(EM_instr) == rs1Id(DE_instr)))
           forwardA = 2'b10; 
       else if (MW_writesRd && (rdId(MW_instr) == rs1Id(DE_instr)))
           forwardA = 2'b01; 
       else
           forwardA = 2'b00; 
   end

   // Lógica de Forwarding para RS2
   reg [1:0] forwardB;
   always @(*) begin
       if (EM_writesRd && (rdId(EM_instr) == rs2Id(DE_instr)))
           forwardB = 2'b10;
       else if (MW_writesRd && (rdId(MW_instr) == rs2Id(DE_instr)))
           forwardB = 2'b01;
       else
           forwardB = 2'b00;
   end

   // Muxes de Forwarding
   wire [31:0] E_aluIn1_fwd = (forwardA == 2'b10) ? EM_Eresult :
                              (forwardA == 2'b01) ? writeBackData : 
                              DE_rs1;

   wire [31:0] E_aluIn2_fwd = (forwardB == 2'b10) ? EM_Eresult :
                              (forwardB == 2'b01) ? writeBackData : 
                              DE_rs2;

   // ---------------- FIM FORWARDING UNIT ---------------- //

   wire [31:0] E_aluIn1 = E_aluIn1_fwd;
   wire [31:0] E_aluIn2 = isALUreg(DE_instr) | isBranch(DE_instr) ? E_aluIn2_fwd : Iimm(DE_instr);
   
   wire [4:0]  E_shamt  = isALUreg(DE_instr) ? E_aluIn2_fwd[4:0] : shamt(DE_instr); 
   wire E_minus = DE_instr[30] & isALUreg(DE_instr);
   wire E_arith_shift = DE_instr[30];

   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7],
      x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15],
      x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
      x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   wire [31:0] E_shifter_in = funct3(DE_instr) == 3'b001 ? flip32(E_aluIn1) : E_aluIn1;
   wire [31:0] E_shifter = $signed({E_arith_shift & E_aluIn1[31], E_shifter_in}) >>> E_aluIn2[4:0];
   wire [31:0] E_leftshift = flip32(E_shifter);

   reg [31:0] E_aluOut;
   always @(*) begin
      case(funct3(DE_instr))
         3'b000: E_aluOut = E_minus ? E_aluMinus[31:0] : E_aluPlus;
         3'b001: E_aluOut = E_leftshift;
         3'b010: E_aluOut = {31'b0, E_LT};
         3'b011: E_aluOut = {31'b0, E_LTU};
         3'b100: E_aluOut = E_aluIn1 ^ E_aluIn2;
         3'b101: E_aluOut = E_shifter;
         3'b110: E_aluOut = E_aluIn1 | E_aluIn2;
         3'b111: E_aluOut = E_aluIn1 & E_aluIn2;
      endcase
   end

   /*********** Branch, JAL, JALR ***********************************/
   reg E_takeBranch;
   always @(*) begin
      case (funct3(DE_instr))
         3'b000: E_takeBranch = E_EQ;
         3'b001: E_takeBranch = !E_EQ;
         3'b100: E_takeBranch = E_LT;
         3'b101: E_takeBranch = !E_LT;
         3'b110: E_takeBranch = E_LTU;
         3'b111: E_takeBranch = !E_LTU;
         default: E_takeBranch = 1'b0;
      endcase
   end

   wire E_JumpOrBranch = (
         isJAL(DE_instr)  ||
         isJALR(DE_instr) ||
         (isBranch(DE_instr) && E_takeBranch)
   );

   wire [31:0] E_JumpOrBranchAddr =
   isBranch(DE_instr) ? DE_PC + Bimm(DE_instr) :
   isJAL(DE_instr)    ? DE_PC + Jimm(DE_instr) :
   /* JALR */           {E_aluPlus[31:1],1'b0} ;

   wire [31:0] E_result =
   (isJAL(DE_instr) | isJALR(DE_instr)) ? DE_PC+4                :
   isLUI(DE_instr)                      ? Uimm(DE_instr)         :
   isAUIPC(DE_instr)                    ? DE_PC + Uimm(DE_instr) :
                                          E_aluOut               ;

   always @(posedge clk) begin
      EM_PC      <= DE_PC;
      EM_instr   <= DE_instr;
      EM_rs2     <= E_aluIn2_fwd; 
      EM_Eresult <= E_result;
      EM_addr    <= isStore(DE_instr) ? E_aluIn1_fwd + Simm(DE_instr) : 
                                        E_aluIn1_fwd + Iimm(DE_instr) ;
   end

/************************ M: Memory *******************************************/
   reg [31:0] MW_PC;
   reg [31:0] MW_instr;
   reg [31:0] MW_Eresult;
   reg [31:0] MW_addr;
   reg [31:0] MW_Mdata;
   reg [31:0] MW_IOresult;
   reg [31:0] MW_CSRresult;
   wire [2:0] M_funct3 = funct3(EM_instr);
   wire M_isB = (M_funct3[1:0] == 2'b00);
   wire M_isH = (M_funct3[1:0] == 2'b01);
   assign halt = !reset & isEBREAK(MW_instr);

   /*************** STORE **************************/
   wire [31:0] M_STORE_data = EM_rs2; 
   assign Address  = EM_addr;
   assign MemWrite    = isStore(EM_instr);
   assign WriteData = EM_rs2;

   always @(posedge clk) begin
      MW_PC        <= EM_PC;
      MW_instr     <= EM_instr;
      MW_Eresult   <= EM_Eresult;
      MW_Mdata     <= ReadData;
      MW_addr      <= EM_addr;
   end

/************************ W: WriteBack ****************************************/

   wire [2:0] W_funct3 = funct3(MW_instr);
   wire W_isB = (W_funct3[1:0] == 2'b00);
   wire W_isH = (W_funct3[1:0] == 2'b01);
   wire W_sext = !W_funct3[2];
   wire W_isIO = MW_addr[22];

   /*************** LOAD ****************************/
   assign writeBackData = isLoad(MW_instr) ? MW_Mdata : MW_Eresult;
   assign writeBackEn = writesRd(MW_instr) && rdId(MW_instr) != 0;
   assign wbRdId = rdId(MW_instr);

   assign jumpOrBranchAddress = E_JumpOrBranchAddr;
   assign jumpOrBranch        = E_JumpOrBranch;

/******************************************************************************/

/****** LÓGICA DE PROGRAM COUNTER *********************************************/

    reg [8:0] r_jumpCounter;
    reg [8:0] r_memAccessCounter;
    reg [8:0] r_cpuCycleCounter;
    reg [8:0] r_instrNumberCounter;
    reg [8:0] r_stallCounter;
    reg [8:0] r_brTakenCounter;
    reg [8:0] r_brNotTakenCounter;
    reg [8:0] r_loadCounter;
    reg [8:0] r_storeCounter;
    reg [8:0] r_flushCounter;
    reg [8:0] r_fwdCounter;
    reg [8:0] r_aluCounter;

    // Associações de saída
    assign jumps        = r_jumpCounter;
    assign memAccess    = r_memAccessCounter;
    assign cpuCycle     = r_cpuCycleCounter;
    assign intructions  = r_instrNumberCounter;
    assign stallCount   = r_stallCounter;
    assign brTaken      = r_brTakenCounter;
    assign brNotTaken   = r_brNotTakenCounter;
    assign loadCount    = r_loadCounter;
    assign storeCount   = r_storeCounter;
    assign flushCount = r_flushCounter;
    assign fwdCount = r_fwdCounter;
    assign aluCount = r_aluCounter;




    always @(posedge clk) begin
        if (reset) begin
            r_jumpCounter        <= 0;
            r_memAccessCounter   <= 0;
            r_cpuCycleCounter    <= 0;
            r_instrNumberCounter <= 0;
            r_stallCounter       <= 0;
            r_brTakenCounter     <= 0;
            r_brNotTakenCounter  <= 0;
            r_loadCounter        <= 0;
            r_storeCounter       <= 0;
            r_flushCounter       <= 0;
            r_fwdCounter         <= 0;
            r_aluCounter         <= 0;
        end else begin
            // 1. Contador de Ciclos (Sempre incrementa)
            r_cpuCycleCounter <= r_cpuCycleCounter + 1;

            // 2. Contador de Jumps (JAL, JALR ou Branch Tomado)
            if (E_JumpOrBranch) 
                r_jumpCounter <= r_jumpCounter + 1;

            // 3. Contador de Instruções (Instruções que saíram do Execute validamente)
            if (EM_instr != NOP)
                r_instrNumberCounter <= r_instrNumberCounter + 1;

            // 4. Stalls 
            if (stall) 
                r_stallCounter <= r_stallCounter + 1;

            // 5. Branches (Taken vs Not Taken)
            // isBranch verifica se é BEQ, BNE, BLT...
            if (isBranch(DE_instr) && DE_instr != NOP && !stall) begin
                if (E_takeBranch)
                    r_brTakenCounter <= r_brTakenCounter + 1;
                else
                    r_brNotTakenCounter <= r_brNotTakenCounter + 1;
            end

            // 6. Loads e Stores (Separados)
            // Usamos MW_instr para garantir que a instrução chegou à memória/WB
            if (MW_instr != NOP) begin
                if (isLoad(MW_instr))
                    r_loadCounter <= r_loadCounter + 1;
                
                if (isStore(MW_instr))
                    r_storeCounter <= r_storeCounter + 1;
                
                if (isLoad(MW_instr) || isStore(MW_instr))
                    r_memAccessCounter <= r_memAccessCounter + 1;
            end

            // 7. Flush
            // Usamos o e_flush que já estava disponibilizado no riscv pipeline
            if (e_flush) begin
               r_flushCounter <= r_flushCounter + 1;
            end

            // 8. Forwarding
            // Verifica se houve forwarding em A OU em B na instrução atual
            // (forward != 00 significa que houve bypass)
            // Importante: Checar se não é NOP para não contar lixo durante bolhas
            if ((forwardA != 2'b00 || forwardB != 2'b00) && EM_instr != NOP)begin
               r_fwdCounter <= r_fwdCounter + 1;
            end

            // 9. ALU Operations
            // Conta se for R-Type (ADD, SUB...) ou I-Type (ADDI, ANDI...)
            // Verifica no estágio MEM ou WB para garantir que a instrução completou
            if ((isALUreg(MW_instr) || isALUimm(MW_instr)) && MW_instr != NOP)begin
               r_aluCounter <= r_aluCounter + 1;

            end
            
        end
    end

    //Resto do código do menotti
   always @(posedge clk) begin
      if (halt) begin
         $writememh("regs.out", RegisterBank);
         $finish();
      end
   end

endmodule