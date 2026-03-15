; isr.asm - SPI DMA 中断向量跳转
; SPI DMA 真实中断号=49，向量=018BH
; interrupt 13 的向量=006BH
; 硬件触发018BH → 跳到006BH → 执行 interrupt 13 函数

    CSEG AT 018BH
    LJMP 006BH

    END