/*
 * Minimal linker script for bare-metal Cortex-A9 bring-up on this board's
 * Zynq-7020, no FSBL/BSP.
 *
 * Load address: 0x00100000 (1 MB into DDR). This is the same convention
 * Xilinx's own standalone-BSP bare-metal apps link at, chosen (not just
 * copied blindly) because:
 *   - It's well clear of address 0x0, which some bring-up/boot flows treat
 *     specially (low exception vectors, OCM remapping) — starting there
 *     risks stepping on something.
 *   - It's comfortably inside this board's configured DDR range: the PS7
 *     config sets PCW_DDR_RAM_HIGHADDR = 0x3FFFFFFF, i.e. DDR spans
 *     [0x00000000, 0x3FFFFFFF] (1 GB addressable). 0x00100000 leaves that
 *     entire range available above it.
 *   - DDR must already be live for this to work at all — ps7_jtag_init.tcl
 *     (run over JTAG before this ELF is ever loaded) is what brings up the
 *     DDR controller; this script assumes that already happened.
 *
 * MEMORY window size (LENGTH) below is an arbitrary 1 MB — this program is
 * a tiny bring-up test, not a real application; adjust if it ever needs
 * more.
 */

ENTRY(_start)

MEMORY
{
    DDR (rwx) : ORIGIN = 0x00100000, LENGTH = 0x00100000

    /* Illustrative only — not used by this project, and not verified
     * against the TRM's exact byte boundaries, so don't trust this
     * byte-for-byte if it's ever actually wired up. Zynq-7000 also has
     * 256 KB of on-chip SRAM (OCM), independent of DDR and usable before
     * the DDR controller is even initialized. Its 4x 64 KB banks are
     * individually remappable (via the SLCR OCM_CFG register) to appear
     * either "low" (near address 0x0 — where this project's linker.ld
     * comment above already flags as risky, since BootROM/vector
     * remapping can live there too) or "high" (near 0xFFFF0000). A
     * second region for it might look like:
     *
     * OCM_LOW  (rwx) : ORIGIN = 0x00000000, LENGTH = 0x00030000
     * OCM_HIGH (rwx) : ORIGIN = 0xFFFF0000, LENGTH = 0x00010000
     *
     * A given output section would then target it the same way .text
     * targets DDR below, just with "> OCM_LOW" instead of "> DDR".
     */
}

STACK_SIZE = 0x00010000; /* 64 KB — generous for a program with no recursion
                             and no large local buffers */

SECTIONS
{
    /* .vectors first and only here: if the exception vector table isn't
     * at the very start of .text, the branch-offset encoding used by the
     * ARM exception vectors (see startup.S) still works since VBAR points
     * directly at _vectors regardless of position — but keeping it first
     * matches convention and makes the layout easy to reason about from
     * a memory dump. */
    .text : {
        *(.vectors)
        *(.text*)
    } > DDR

    .rodata : { *(.rodata*) } > DDR

    .data : { *(.data*) } > DDR

    .bss : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } > DDR

    . = ALIGN(8);
    . = . + STACK_SIZE;
    __stack_top = .;
}
