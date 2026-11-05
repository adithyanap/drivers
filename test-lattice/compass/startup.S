/*******************************************************************************
 * Minimal THEJAS32 / ARIES startup for the bare-metal compass application.
 *
 * Replaces the FreeRTOS-specific entry.S: no FreeRTOS trap handler or
 * scheduler is required because main() polls the magnetometer directly.
 ******************************************************************************/

    .section .text.entry
    .globl _start

_start:
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop

    la      sp, __stack_top

    /* Any unexpected trap parks the CPU here instead of jumping randomly. */
    la      t0, trap_handler
    csrw    mtvec, t0
    csrw    mie, zero

    /*
     * .data/.sdata are part of the XMODEM-loaded binary and already reside
     * at their VMA. Only .sbss and .bss need to be cleared.
     */
    la      t0, __sbss_start
    la      t1, __bss_end

clear_bss:
    bgeu    t0, t1, bss_done
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       clear_bss

bss_done:
    call    main

hang:
    j       hang

    .align  2
trap_handler:
    j       trap_handler
