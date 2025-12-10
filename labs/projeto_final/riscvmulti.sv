module riscvmulti (
    input         clk,
    input         reset,
    output [31:0] Address, 
    output [31:0] WriteData,
    output        MemWrite,
    input  [31:0] ReadData,
    output  [3:0] WriteMask, 
    output logic  halt = 0); 

    //  Contadores de instrução: Hardware Analysis
    integer r_type = 0;
    integer b_type = 0;
    integer i_type = 0;
    integer u_type = 0;
    integer j_type = 0;
    // wire isALUreg  =  (instr[6:0] == 7'b0110011); 
    // wire isALUimm  =  (instr[6:0] == 7'b0010011); 
    // wire isBranch  =  (instr[6:0] == 7'b1100011); 
    // wire isJALR    =  (instr[6:0] == 7'b1100111); 
    // wire isJAL     =  (instr[6:0] == 7'b1101111); 
    // wire isAUIPC   =  (instr[6:0] == 7'b0010111); 
    // wire isLUI     =  (instr[6:0] == 7'b0110111);    
    // wire isLoad    =  (instr[6:0] == 7'b0000011); 
    // wire isStore   =  (instr[6:0] == 7'b0100011); 
    // wire isSYSTEM  =  (instr[6:0] == 7'b1110011); 
    // wire isEBREAK  =  (isSYSTEM && (instr[14:12] == 3'b000));
    always @(posedge clk) begin
        // if (isALUimm || is)
    end

    logic [31:0] instr, PC = 0;

    // --- Lógica Auxiliar (Definida abaixo, mas usada aqui) ---
    // Precisamos processar o dado lido da memória (para lb/lh) e gerar a máscara (para sb/sh)
    logic [31:0] LoadedDataProcessed; 
    logic [3:0]  GenWriteMask;

    // --- Respostas do exercício ---

    // 1. Quando escreve no registro? 
    // No estado EXECUTE (para ALUs/Jumps/LUI) ou WAIT_DATA (para Loads).
    // Note: Branches e Stores não escrevem em rd.
    wire writeBackEn = ((state == EXECUTE) && (isALUreg || isALUimm || isJAL || isJALR || isLUI || isAUIPC)) || (state == WAIT_DATA);

    // 2. O que escreve no registro?
    // Se for Load, usa o dado processado (byte/half). Se JAL/JALR, PC+4. Se LUI, Uimm. Senão, resultado da ALU.
    wire [31:0] writeBackData = (state == WAIT_DATA) ? LoadedDataProcessed : 
                                (isJAL || isJALR)    ? PCplus4 : 
                                (isLUI)              ? Uimm :
                                (isAUIPC)            ? PCTarget : ALUResult;

    // 3. Endereço de Load/Store: rs1 + offset (Simm para Store, Iimm para Load)
    wire [31:0] LoadStoreAddress = rs1 + (isStore ? Simm : Iimm);

    // 4. Endereço da Memória: PC (busca instrução) ou Endereço calculado (dados)
    assign Address = (state == FETCH_INSTR || state == WAIT_INSTR) ? PC : LoadStoreAddress;

    // 5. Escrita na memória apenas no estado STORE
    assign MemWrite = (state == STORE);

    // 6. Dado para escrever na memória:
    // O dado (rs2) precisa ser deslocado para a posição correta do byte (alinhamento).
    // Ex: sb t0, 1(a0) -> byte t0 vai para bits [15:8] da palavra.
    assign WriteData = rs2 << (LoadStoreAddress[1:0] * 8);

    // Máscara de escrita conectada à saída
    assign WriteMask = GenWriteMask;

    // ---------------------------------------------------------
    // O RESTANTE DO CÓDIGO PERMANECE, MAS COM ADIÇÕES ABAIXO
    // ---------------------------------------------------------

    // The 10 RISC-V instructions
    wire isALUreg  =  (instr[6:0] == 7'b0110011); 
    wire isALUimm  =  (instr[6:0] == 7'b0010011); 
    wire isBranch  =  (instr[6:0] == 7'b1100011); 
    wire isJALR    =  (instr[6:0] == 7'b1100111); 
    wire isJAL     =  (instr[6:0] == 7'b1101111); 
    wire isAUIPC   =  (instr[6:0] == 7'b0010111); 
    wire isLUI     =  (instr[6:0] == 7'b0110111);    
    wire isLoad    =  (instr[6:0] == 7'b0000011); 
    wire isStore   =  (instr[6:0] == 7'b0100011); 
    wire isSYSTEM  =  (instr[6:0] == 7'b1110011); 
    wire isEBREAK  =  (isSYSTEM && (instr[14:12] == 3'b000));

    // The 5 immediate formats
    wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
    wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
    wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
    wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};

    // Source and destination registers
    wire [4:0] rs1Id_A1 = instr[19:15];
    wire [4:0] rs2Id_A2 = instr[24:20];
    wire [4:0] rdId_A3  = instr[11:7];

    // function codes
    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    // The registers bank
    reg [31:0] RegisterBank [0:31];
    reg [31:0] rs1; 
    reg [31:0] rs2; 

    // The ALU
    wire [31:0] SrcA = rs1;
    wire [31:0] SrcB = isALUreg | isBranch ? rs2 : Iimm;
    wire [ 4:0] shamt  = isALUreg ? rs2[4:0] : instr[24:20]; 

    wire [31:0] aluPlus = SrcA + SrcB;
    wire [32:0] aluMinus = {1'b1, ~SrcB} + {1'b0,SrcA} + 33'b1;
    wire        LT  = (SrcA[31] ^ SrcB[31]) ? SrcA[31] : aluMinus[32];
    wire        LTU = aluMinus[32];
    wire        EQ  = (aluMinus[31:0] == 0);

    function [31:0] flip32;
        input [31:0] x;
        flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
        x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
        x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
        x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
    endfunction

    wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(SrcA) : SrcA;
    wire [31:0] shifter = $signed({instr[30] & SrcA[31], shifter_in}) >>> SrcB[4:0];
    wire [31:0] leftshift = flip32(shifter);

    reg [31:0]  ALUResult;
    always @(*) begin
        case(funct3)
            3'b000: ALUResult = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
            3'b001: ALUResult = leftshift;
            3'b010: ALUResult = {31'b0, LT};
            3'b011: ALUResult = {31'b0, LTU};
            3'b100: ALUResult = (SrcA ^ SrcB);
            3'b101: ALUResult = shifter;
            3'b110: ALUResult = (SrcA | SrcB);
            3'b111: ALUResult = (SrcA & SrcB);  
        endcase
    end

    // --- NOVA LÓGICA: LOAD DATA PROCESSING (lb, lh, lw, lbu, lhu) ---
    // Precisamos deslocar o dado lido para a direita baseado nos 2 bits do endereço
    // e depois fazer a extensão de sinal ou zero.
    wire [1:0] addrOffset = LoadStoreAddress[1:0];
    wire [31:0] loadedShifted = ReadData >> (addrOffset * 8); // Desloca bytes

    always @(*) begin
        if (isLoad) begin
            case(funct3)
                3'b000: LoadedDataProcessed = {{24{loadedShifted[7]}}, loadedShifted[7:0]};   // lb (sign-extend)
                3'b001: LoadedDataProcessed = {{16{loadedShifted[15]}}, loadedShifted[15:0]}; // lh (sign-extend)
                3'b010: LoadedDataProcessed = loadedShifted;                                  // lw
                3'b100: LoadedDataProcessed = {24'b0, loadedShifted[7:0]};                    // lbu (zero-extend)
                3'b101: LoadedDataProcessed = {16'b0, loadedShifted[15:0]};                   // lhu (zero-extend)
                default: LoadedDataProcessed = loadedShifted;
            endcase
        end else begin
            LoadedDataProcessed = 32'b0;
        end
    end

    // --- NOVA LÓGICA: STORE MASK GENERATION (sb, sh, sw) ---
    // Gera a máscara de 4 bits baseada no endereço e no tamanho (funct3)
    always @(*) begin
        if (isStore) begin
            case(funct3) // funct3 define sb, sh, sw
                3'b000: GenWriteMask = 4'b0001 << addrOffset; // sb
                3'b001: GenWriteMask = 4'b0011 << addrOffset; // sh
                3'b010: GenWriteMask = 4'b1111;               // sw
                default: GenWriteMask = 4'b0000;
            endcase
        end else begin
            GenWriteMask = 4'b0000; // Não escreve se não for store
        end
    end

    reg takeBranch;
    always @(*) begin
        case(funct3)
            3'b000: takeBranch = EQ;
            3'b001: takeBranch = !EQ;
            3'b100: takeBranch = LT;
            3'b101: takeBranch = !LT;
            3'b110: takeBranch = LTU;
            3'b111: takeBranch = !LTU;
            default: takeBranch = 1'b0;
        endcase
    end

    wire [31:0] PCplus4  = PC + 4;
    wire [31:0] PCTarget = PC + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm);
    wire [31:0] PCNext = ((isBranch && takeBranch) || isJAL) ? PCTarget :
                                                      isJALR ? {aluPlus[31:1],1'b0} :
                                                               PCplus4;

    // The state machine
    localparam FETCH_INSTR = 0;
    localparam WAIT_INSTR  = 1;
    localparam FETCH_REGS  = 2;
    localparam EXECUTE     = 3;
    localparam LOAD        = 4;
    localparam WAIT_DATA   = 5;
    localparam STORE       = 6;
    reg [2:0] state = FETCH_INSTR;

    always @(posedge clk)
        if (reset) begin
            PC    <= 0;
            state <= FETCH_INSTR;
        end else begin
            if (writeBackEn) begin
                // Apenas debug:
                //$display("PC=%h WRITE r%0d <= %h", PC, rdId_A3, writeBackData);
                RegisterBank[rdId_A3] <= writeBackData;
            end
            case(state)
                FETCH_INSTR: begin
                    state <= WAIT_INSTR;
                end
                WAIT_INSTR: begin
                    instr <= ReadData;
                    state <= FETCH_REGS;
                end
                FETCH_REGS: begin
                    rs1 <= rs1Id_A1 ? RegisterBank[rs1Id_A1] : 32'b0;
                    rs2 <= rs2Id_A2 ? RegisterBank[rs2Id_A2] : 32'b0;
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    if (!isSYSTEM)
                        PC <= PCNext;
                    else
                        if (isEBREAK) begin
                            PC <= PC; 
                            halt <= 1;
                        end
                    state <= isLoad  ? LOAD  : 
                             isStore ? STORE : 
                                       FETCH_INSTR;
                end
                LOAD: begin
                    state <= WAIT_DATA;
                end
                WAIT_DATA: begin
                    state <= FETCH_INSTR;
                end
                STORE: begin
                    state <= FETCH_INSTR;
                end
            endcase 
        end

    always @(posedge clk) begin
        if (halt) begin
            $writememh("regs.out", RegisterBank);
            // Dump de memória opcional para verificar o framebuffer
            // $writememh("mem_dump.out", mem.RAM); 
            #10 $finish();
        end
    end
endmodule