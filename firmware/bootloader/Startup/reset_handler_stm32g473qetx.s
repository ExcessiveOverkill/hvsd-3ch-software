  .syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.thumb

/* Symbols provided by the linker script. */
.word	_sram_start
.word	_estack
.word	_sidata
.word	_sdata
.word	_edata
.word	_sbss
.word	_ebss

    .section	.text.Reset_Handler
	.global	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0

  /*
   * Initialize all SRAM before any parity-protected reads can occur.
   * This loop is self-contained in flash and uses registers only, so it
   * does not depend on stack contents while the memory is being scrubbed.
   */
  ldr   r1, =_sram_start
  movs  r2, #0
  b     LoopFillSram

FillSram:
  str   r2, [r1], #4

LoopFillSram:
  cmp   r1, r0
  bcc   FillSram

/* Call the clock system initialization function. */
    bl  SystemInit

/* Copy the data segment initializers from flash to SRAM. */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b    LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call static constructors. */
    bl __libc_init_array
/* Call the application's entry point. */
	bl	main

LoopForever:
    b LoopForever

.size	Reset_Handler, .-Reset_Handler
