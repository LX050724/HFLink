ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of RAM */

MEMORY
{
    ITCM (rx) :    ORIGIN = 0x00000000, LENGTH = 32K
    RAM (rwx) :    ORIGIN = 0x20000000, LENGTH = 32K
    /* EXFLASH (rx) : ORIGIN = 0x4D400000, LENGTH = 4M-16K */
}

__itcm_start__ = 0;
__itcm_end__ = 0;
__itcm_loadaddr__ = 0;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        __isr_vector_start__ = .;
        KEEP(*(.isr_vector)) /* Startup code */
        __isr_vector_end__ = .;
        . = ALIGN(4);
    } >ITCM

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
    } >ITCM

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >ITCM

    .ARM.extab : {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >ITCM

    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >ITCM

    .preinit_array     :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >ITCM

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >ITCM

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >ITCM

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
    } > RAM AT> ITCM

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
    
    /*
    .itcm :
    {
        . = ALIGN(4);
        __itcm_start__ = .;
        *(.itcm)
        *(.itcm*)
        __itcm_end__ = .;
        . = ALIGN(4);
    } >ITCM AT>EXFLASH
    __itcm_loadaddr__ = LOADADDR(.itcm);
    */
    
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
    
    .ARM.attributes 0 : { *(.ARM.attributes) }
}