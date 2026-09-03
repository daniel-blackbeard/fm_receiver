/*
 * Minimal Cortex-A9 startup for bare-metal JTAG bring-up (no FSBL/BSP).
 * Sets a known CPU state, installs an exception vector table, enables
 * MMU + I-cache, sets up a stack, hands off to main(). xsdb sets PC
 * directly to _start over JTAG.
 *
 * Vector table: kept even though xsdb entry doesn't need it, so an
 * unexpected exception traps to an infinite loop instead of falling
 * through to whatever's at 0x0 -- see README.md's halt-and-inspect
 * procedure.
 *
 * MMU: flat identity-mapped (mmu_table below). DDR is Normal/Cacheable
 * (code lives there); everything else Strongly Ordered. Only I-cache is
 * enabled (SCTLR.I) -- data cache stays off.
 *
 * SCTLR.Z (branch prediction) is cleared for Cortex-A9 erratum 794073
 * (silicon r3p0) -- see README.md.
 */

.syntax unified
.arm

    .section .vectors, "ax"
    .global _vectors
_vectors:
    b   _start                  @ 0x00: Reset
    b   _undefined_handler      @ 0x04: Undefined instruction
    b   _swi_handler            @ 0x08: Software interrupt (SVC)
    b   _prefetch_abort_handler @ 0x0C: Prefetch abort
    b   _data_abort_handler     @ 0x10: Data abort
    b   .                       @ 0x14: reserved
    b   _irq_handler            @ 0x18: IRQ
    b   _fiq_handler            @ 0x1C: FIQ

/*
 * Fault reporting: dumps a marker + 3 diagnostic words over UART1 before
 * hanging, so a crash is readable without JTAG. Zero stack use (no bl,
 * no push/pop) so it stays robust even if the crash corrupted sp.
 * Clobbers r0-r9; never returns.
 */
fault_report:
    ldr     r9, =0xE0001000     @ UART1_BASE
    mov     r4, r3              @ marker
    mov     r5, r0              @ word 1
    mov     r6, r1              @ word 2
    mov     r7, r2              @ word 3
    mov     r8, #0              @ word index 0..3
fr_word_loop:
    cmp     r8, #0
    moveq   r0, r4
    cmp     r8, #1
    moveq   r0, r5
    cmp     r8, #2
    moveq   r0, r6
    cmp     r8, #3
    moveq   r0, r7

    mov     r1, #4              @ byte counter within this word
fr_byte_loop:
    mov     r2, r0, lsr #24     @ next byte, MSB first (matches uart1_put32)
fr_wait_txfull:
    ldr     r3, [r9, #0x2C]     @ UART_SR
    tst     r3, #(1 << 4)       @ TXFULL
    bne     fr_wait_txfull
    str     r2, [r9, #0x30]     @ UART_FIFO
    mov     r0, r0, lsl #8
    subs    r1, r1, #1
    bne     fr_byte_loop

    add     r8, r8, #1
    cmp     r8, #4
    bne     fr_word_loop

fault_hang:
    b       fault_hang

/*
 * Each handler reports: marker, then two fault-specific diagnostic words,
 * then lr (return-address-relative to the faulting instruction, per
 * ARM's standard +4/+8 conventions -- lr itself, unadjusted, so the
 * reader applies the right offset for the exception type). Undefined
 * Instruction has no fault status/address registers, so it reports lr
 * twice for a consistent 4-word shape.
 */
/*
 * FAULT_REGDUMP_ADDR: fixed scratch address where each handler stashes
 * the interrupted context's original r0-r12 before clobbering any of
 * them -- safe to do with a plain stmia since r0-r12 are NOT banked for
 * Undefined/Abort modes (only r13/r14 are), so at handler entry they
 * still hold exactly what the faulting code had. sp (r13) is banked and
 * otherwise unused/uninitialized in these modes, so it's reused here as
 * a throwaway base-address register. Lets a JTAG halt afterward read the
 * true fault-time general registers even though fault_report's own UART
 * dump destroys them. 0x00300000 sits in the unused DDR gap between this
 * program's own 1MB window and GEM_DESCRIPTOR_TX (0x00800000).
 */
.equ FAULT_REGDUMP_ADDR, 0x00300000

_undefined_handler:
    ldr     sp, =FAULT_REGDUMP_ADDR
    stmia   sp, {r0-r12}
    mov     r0, lr
    mov     r1, lr
    mov     r2, #0
    ldr     r3, =0xDEADFA11
    b       fault_report
_swi_handler:            b .
_prefetch_abort_handler:
    ldr     sp, =FAULT_REGDUMP_ADDR
    stmia   sp, {r0-r12}
    mrc     p15, 0, r0, c5, c0, 1   @ IFSR
    mrc     p15, 0, r1, c6, c0, 2   @ IFAR
    mov     r2, lr
    ldr     r3, =0xDEADFA12
    b       fault_report
_data_abort_handler:
    ldr     sp, =FAULT_REGDUMP_ADDR
    stmia   sp, {r0-r12}
    mrc     p15, 0, r0, c5, c0, 0   @ DFSR
    mrc     p15, 0, r1, c6, c0, 0   @ DFAR
    mov     r2, lr
    ldr     r3, =0xDEADFA13
    b       fault_report
_irq_handler:            b .
_fiq_handler:            b .

    .section .text
    .global _start
_start:
    @ Force a known CPU mode rather than trusting entry state -- this is
    @ a JTAG-driven PC set, not a real hardware reset. 0xD3 = Supervisor
    @ mode (0x13) | IRQ masked (0x80) | FIQ masked (0x40).
    mov     r0, #0xD3
    msr     cpsr_c, r0

    @ Erratum 794073 workaround: clear SCTLR.Z (bit 11, branch
    @ prediction). Same "don't trust entry state" reasoning -- this core
    @ already ran ps7_init.tcl before _start, not a clean reset.
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #(1 << 11)
    mcr     p15, 0, r0, c1, c0, 0
    isb

    @ VBAR -> our vector table, so exceptions actually trap here instead
    @ of the default 0x0 base.
    ldr     r0, =_vectors
    mcr     p15, 0, r0, c12, c0, 0

    ldr     sp, =__stack_top

    @ Build the MMU table: DDR (0x00000000-0x3FFFFFFF) as Normal
    @ Cacheable (Write-Back, Write-Allocate); everything else
    @ (0x40000000-0xFFFFFFFF: PL peripherals, PS7 UART/SPI/etc.) as
    @ Strongly Ordered.
    ldr     r0, =mmu_table
    mov     r1, #0                   @ r1 = running section base address
    ldr     r4, =0x1C0E              @ Normal, WB/WA Cacheable, AP=11,
                                      @ Domain 0, Section
    mov     r2, #1024                @ 1024 x 1MB = 0x40000000 (DDR range)
1:
    orr     r3, r1, r4
    str     r3, [r0], #4
    add     r1, r1, #0x00100000
    subs    r2, r2, #1
    bne     1b

    ldr     r4, =0xC02               @ Strongly Ordered, AP=11, Domain 0,
                                      @ Section
    mov     r2, #3072                @ remaining 3072 x 1MB, up to
                                      @ 0xFFFFFFFF
2:
    orr     r3, r1, r4
    str     r3, [r0], #4
    add     r1, r1, #0x00100000
    subs    r2, r2, #1
    bne     2b

    ldr     r0, =mmu_table
    mcr     p15, 0, r0, c2, c0, 0     @ TTBR0 = table base

    mvn     r0, #0
    mcr     p15, 0, r0, c3, c0, 0     @ DACR = all 16 domains Manager

    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0     @ invalidate entire I-cache before
                                      @ enabling it

    mrc     p15, 0, r0, c1, c0, 0     @ SCTLR
    orr     r0, r0, #1                @ M (bit 0) -- MMU enable
    orr     r0, r0, #(1 << 12)        @ I (bit 12) -- instruction cache
                                      @ enable
    mcr     p15, 0, r0, c1, c0, 0
    dsb
    isb

    @ Zero .bss: C assumes zero-initialized statics/globals actually read
    @ as zero on first use. Nothing before this point in the project
    @ relied on that guarantee (no static/global mutable state existed
    @ anywhere), so this was missing since the very first version of this
    @ file -- only surfaced once real code started depending on it.
    ldr     r0, =__bss_start
    ldr     r1, =__bss_end
    mov     r2, #0
3:
    cmp     r0, r1
    strlt   r2, [r0], #4
    blt     3b

    bl      main

hang:
    b       hang

    .section .data
    .balign 16384
mmu_table:
    .space 16384    /* 4096 x 4-byte section descriptors, 1MB each,
                      * covering the full 4GB space. Filled in by _start
                      * above before TTBR0 ever points at it. */
