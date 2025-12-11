module mem (
    input  logic        clk, 
    input  logic        we,
    // Porta de Instruções (Apenas Leitura - PC)
    input  logic [31:0] a_instr,
    output logic [31:0] rd_instr,
    // Porta de Dados (Leitura e Escrita - Load/Store)
    input  logic [31:0] a_data,
    input  logic [31:0] wd_data,
    output logic [31:0] rd_data
);

    // O banco de memória em si
    logic [31:0] MEM [0:511];

    initial begin
        $readmemh("riscv.hex", MEM);
    end

    // Leitura Assíncrona 1: Instruções (Baseado no PC)
    assign rd_instr = MEM[a_instr[31:2]];

    // Leitura Assíncrona 2: Dados (Baseado no Address da ALU)
    assign rd_data = MEM[a_data[31:2]];

    // Escrita Síncrona: Apenas na porta de dados
    always_ff @(posedge clk) begin
        if (we) begin
            MEM[a_data[31:2]] <= wd_data;
        end
    end

endmodule