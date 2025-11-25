[![Open in Codespaces](https://classroom.github.com/assets/launch-codespace-2972f46106e565e64193e422d61a12cf1da4916b45550586e14ef0a7c637dd04.svg)](https://classroom.github.com/open-in-codespaces?assignment_repo_id=21830821)
# Implementação de um RISC-V pipeline

O [código fornecido a você](riscvpipeline.sv) nesta simulação é uma implementação pipeline do RISC-V, adaptada do código de Bruno Levy [^1]. Ele roda, por sua vez, os programas adaptados do [nosso livro texto](*.asm) [^2] com um preâmbulo para inicializar os registradores usados nos exemplos do livro. 

Na versão *pipeline* voltamos a usar o esquema de *Harvard* (dados e instruções em memórias separadas). O código fornecido está completo, mas o processador não trata nenhum *Hazard*, então você precisa corrigir isso por etapas: 

1. O programa [`base.asm`](tests/base.asm) executa corretamente, pois não há nenhuma dependência entre as instruções; 
1. O programa [`forward.asm`](tests/forward.asm) precisa que você implemente o encaminhamento, pois há uma [dependência RAW](https://en.wikipedia.org/wiki/Data_dependency#Read_after_write_(RAW)) no registrador `s8`. Além disso, o banco de registradores precisa suportar [leitura/escrita no mesmo ciclo](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/PIPELINE.md#step-5-reading-and-writing-the-register-file-in-the-same-cycle), lendo o valor atualizado. 
1. O programa [`stall.asm`](tests/stall.asm) precisa que você implemente uma bolha após o `lw` quando houver dependência do valor gravado, pois isso não pode ser revolvido com encaminhamento, uma vez que o valor vem da memória. 
1. O programa [`flush.asm`](tests/flush.asm) precisa que você implemente o descarte das instruções quando um salto é tomado, já que o processador só desvia do estágio de execução e as instruções seguintes foram indevidamente carregadas no pipeline. 

Nesta simulação, ao invés de olhar para a saída da simulação e comparar com a saída esperada, optou-se por salvar o conteúdo final do banco de registradores após a execução. Assim, você pode descomentar e usar livremente o `$monitor` que está no [test bench](tests/testbench.sv) para depurar o seu código. 

> Atenção: se você colocar instruções `nop` após os trechos críticos dos programas eles irão executar corretamente. Você pode fazer isso para testar, mas não se esqueça de restaurar os programas originais, pois eles estão na [pasta de testes](tests) e nenhum arquivo nela pode ser alterado.  

## References
[^1]: [From Blinker to RISC-V](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/)
[^2]: [Digital Design and Computer Architecture, RISC-V Edition](https://shop.elsevier.com/books/digital-design-and-computer-architecture-risc-v-edition/harris/978-0-12-820064-3)

