.text
.globl main

main:
    li s0, 0x204        # Endereço dos LEDs (LEDR)
    li s1, 0x210        # Endereço das Chaves (KEY)
    
    # --- Inicialização ---
    li a0, 1            # a0 mantém a posição do LED (começa em 0000000001)
    li t0, 0x3ff        # 512          # Limite superior

loop:
    # 1. Atualiza a Saída Visual
    sw a0, 0(s0)        # Escreve o valor nos LEDs vermelhos

    # 2. Leitura do Input (Memory Input Map)
    lw t1, 0(s1)        # Lê o estado de todos os KEYs do endereço 0x210
    
    andi t2, t1, 2      # Aplica máscara para isolar o bit 1 (valor 2)
    
    # Se t2 for 0, significa que KEY[1] está pressionado
    beqz t2, mover_direita

mover_esquerda:
    # move o ponto para a esquerda (multiplica por 2)
    slli a0, a0, 1   
    # adiciona 1
    addi a0, a0, 1   
    
    # Verifica se passou do último LED
    bgt a0, t0, reset_inicio    # reseta os leds se ligar todos
    
    j loop              # Volta para o início do loop

mover_direita:
    #  move o ponto para a direita (divide por 2)
    srli a0, a0, 1
    
    # Verifica se o valor virou 0
    beqz a0, reset_inicio   # reseta os leds se apagar todos
    
    j loop              # Volta para o início do loop

reset_inicio:
    li a0, 1            # Reinicia o ponto na primeira posição
    j loop