#include "mcu.h"

extern int
main(void);

__attribute__((naked, noreturn)) void
_reset(void)
{
  asm("ldr sp, = _estack");

  extern long _sbss, _ebss, _sdata, _edata, _sidata;

  /* Set bss region to 0 */
  for (long* src = &_sbss; src < &_ebss; src++)
    *src = 0;
  /* Copy from _sdata to _sidata */
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;)
    *src++ = *dst++;

  main();
  while (1)
    (void)0;
}

extern void
SysTick_Handler(void);

__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {
  0, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};
