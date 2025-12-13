ENTRY(Init)

MEMORY
{
    RAM (rx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__itcm_start__ = 0;
__itcm_end__ = 0;
__itcm_loadaddr__ = 0;

SECTIONS
{
    PrgCode :
    {
        . = ALIGN(4);
        KEEP(*(PrgCode))
        . = ALIGN(4);
    } > RAM

    .text :
    {
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
    } >RAM

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >RAM

    .ARM.extab : {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >RAM

    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >RAM

    PrgData :
    {
        . = ALIGN(4);
        KEEP(*(PrgData))
        . = ALIGN(4);
    } > RAM

    .preinit_array     :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >RAM

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >RAM

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >RAM

    /* used by the startup to initialize data */
    __data_load_addr__ = LOADADDR(.data);

    /* Initialized data sections goes into RAM, load LMA copy after code */
    .data : 
    {
        . = ALIGN(4);
        __data_start__ = .;        /* create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        __data_end__ = .;        /* define a global symbol at data end */
    } > RAM

    /* Uninitialized data section */
    . = ALIGN(4);
    .bss (NOLOAD) :
    {
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } > RAM
    
    DevDscr :
    {
        . = ALIGN(4);
        KEEP(*(DevDscr))
        . = ALIGN(4);
    } > RAM

    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
    
    .ARM.attributes 0 : { *(.ARM.attributes) }
}