#include "ti_enet_config.h"
--retain="*(.vectors)"

--stack_size=16384
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=32768
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */
--entry_point=_c_int00

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > M55_VECS

    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .text:   {} palign(8)   /* This is where code resides */
        .rodata: {} palign(8)   /* This is where const's go */
    } > DDR0

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > DDR0

    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > DDR0

    /* Sections needed for C++ projects */
    GROUP {
        .ARM.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array: {} palign(8)   /* Contains function pointers called before main */
        .fini_array: {} palign(8)   /* Contains function pointers called after main */
    } > DDR0

    .enet_dma_mem (NOLOAD) : {
    *(*ENET_DMA_DESC_MEMPOOL)
    *(*ENET_DMA_RING_MEMPOOL)
#if (ENET_SYSCFG_PKT_POOL_ENABLE == 1)
    *(*ENET_DMA_PKT_MEMPOOL)
#endif
    } > DDR0
}

MEMORY
{
    M55_VECS : ORIGIN = 0x78000000 , LENGTH = 0x600
    M55_IRAM : ORIGIN = 0x24000000 + 0x600 , LENGTH = 0xFFFF - 0x600
    M55_DRAM : ORIGIN = 0x24010000 , LENGTH = 0xFFFF
    DDR0 : ORIGIN = 0x80000000 , LENGTH = 0x20000000
}
