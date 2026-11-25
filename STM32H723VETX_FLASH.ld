/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                512Kbytes FLASH and 560Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2026 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  ITCMRAM (xrw)    : ORIGIN = 0x00000000,   LENGTH = 128K
  DTCMRAM (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH    (rx)    : ORIGIN = 0x08000000,   LENGTH = 512K
  RAM_D1  (xrw)    : ORIGIN = 0x24000000,   LENGTH = 256K
  RAM_D2  (xrw)    : ORIGIN = 0x30000000,   LENGTH = 32K
  RAM_D3  (xrw)    : ORIGIN = 0x38000000,   LENGTH = 16K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* Define the special memory locations */
  /* FLASH porgram code */

  _siitcm_text = LOADADDR(.itcm_text);
  .itcm_text :
  {
    _sitcm_text = .;        /* define a global symbols at start of FLASH only code */
    . = ALIGN(4);

    /* src/application */
    EXCLUDE_FILE (*log.o */init.o */hil.o */health_checks.o */*_handler.o)
    *src/application/*(.text .text* .glue_7 .glue_7t .eh_frame)

    KEEP (*src/application/*(.init))
    KEEP (*src/application/*(.fini))

    *src/drivers/*(.text .text* .glue_7 .glue_7t .eh_frame)
    KEEP (*src/drivers/*(.init))
    KEEP (*src/drivers/*(.fini))

    /* */closedrocket/*
    */closedrocket/*(.text)           /* .text sections (FLASH only code) */
    */closedrocket/*(.text*)          /* .text* sections (FLASH only code) */
    */closedrocket/*(.glue_7)         /* glue arm to thumb FLASH only code */
    */closedrocket/*(.glue_7t)        /* glue thumb to arm FLASH only code */
    */closedrocket/*(.eh_frame)

    KEEP (*/closedrocket/*(.init))
    KEEP (*/closedrocket/*(.fini))

    /* HAL Drivers & src */
    EXCLUDE_FILE (*/FatFs/*) 
    */src/third_party/cubemx_autogen/Drivers/STM32H7xx_HAL_Driver/*(.text .text* .glue_7 .glue_7t .eh_frame)

    KEEP (*/src/third_party/cubemx_autogen/Drivers/*(.init))
    KEEP (*/src/third_party/cubemx_autogen/Drivers/*(.fini))

    EXCLUDE_FILE (*system_stm32h7xx.o) 
    */src/third_party/cubemx_autogen/src/*(.text .text* .glue_7 .glue_7t .eh_frame)

    KEEP (*/src/third_party/cubemx_autogen/src/*(.init))
    KEEP (*/src/third_party/cubemx_autogen/src/*(.fini))


    . = ALIGN(4);
    _eitcm_text = .;        /* define a global symbols at end of code */
  } >ITCMRAM AT>FLASH


  /* used by the startup to initialize data */
  _siramd1_data = LOADADDR(.ramd1_data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .ramd1_data :
  {
    . = ALIGN(4);
    _sramd1_data = .;        /* create a global symbol at data start */
    

    /* collect anything put into this region */
    *(.ramd1_data)

    /* CubeMX Source Special Cases */
    *sd_diskio.o(.data .data* .RamFunc .RamFunc*)
    *bsp_driver_sd.o(.data .data* .RamFunc .RamFunc*)
    *sdmmc.o(.data .data* .RamFunc .RamFunc*)

    /* FatFS */
    */src/third_party/cubemx_autogen/Drivers/STM32H7xx_HAL_Driver/FatFs/*(.data .data* .RamFunc .RamFunc*)

    . = ALIGN(4);
    _eramd1_data = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .ramd1_bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sramd1_bss = .;         /* define a global symbol at bss start */
    __ramd1_bss_start__ = _sramd1_bss;

    /* collect anything put into this region */
    *(.ramd1_bss)

    /* CubeMX Source Special Cases */
    *sd_diskio.o(.bss .bss* COMMON)
    *bsp_driver_sd.o(.bss .bss* COMMON)
    *sdmmc.o(.bss .bss* COMMON)

    /* FatFS */
    */src/third_party/cubemx_autogen/Drivers/STM32H7xx_HAL_Driver/FatFs/*(.bss .bss* COMMON)

    . = ALIGN(4);
    _eramd1_bss = .;         /* define a global symbol at bss end */
    __ramd1_bss_end__ = _eramd1_bss;
  } >RAM_D1


  /* The program code and other data goes into FLASH */
  .text :
  {
    _stext = .;        /* define a global symbols at start of code */
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  _sirodata = LOADADDR(.rodata);

  .rodata :
  {
    _srodata = .;        /* define a global symbols at start of rodata */
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    _erodata = .;        /* define a global symbols at end of rodata */
    . = ALIGN(4);
  } >DTCMRAM AT>FLASH

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH
  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >DTCMRAM AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >DTCMRAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* used by the startup to initialize data */
  _sisram4 = LOADADDR(.sram4);

  .sram4 :
  {
    . = ALIGN(4);
    _ssram4 = .;        /* create a global symbol at sram4 start */
    *(.sram4)
    . = ALIGN(4);
    _esram4 = .;        /* define a global symbol at sram4 end */
  } >RAM_D3 AT>FLASH

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
