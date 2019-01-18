# 1 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
# 61 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
;
# 80 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
;
# 98 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
        .global HardFault_Handler
        .type HardFault_Handler, function



;
# 120 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
        .extern HardFaultHandler



;
# 133 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
        .syntax unified
        .thumb
        .balign 4
        .text







;






;
# 163 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
HardFault_Handler:
# 190 "/Users/developer/Code/RUI_segger/nRF5SDK15.0.0-master/nRF5_SDK_15.0.0_a53641a/RUI_ses/HardFaultHandler.S"
        ;
        tst LR, #4 ;
        ite EQ
        mrseq R0, MSP ;
        mrsne R0, PSP ;




        ldr R1,=HardFaultHandler
        bx R1 ;


        .end




;
