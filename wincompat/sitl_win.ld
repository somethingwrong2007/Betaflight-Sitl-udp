SECTIONS {
  /* Windows PE does not have the ELF .fini_array section used by the
     official sitl.ld, so place the parameter-group registry before .data. */
  .pg_registry BLOCK( DEFINED(__section_alignment__) ? __section_alignment__ : 4 ) :   SUBALIGN(4)
  {
    PROVIDE_HIDDEN (__pg_registry_start = . );
    PROVIDE_HIDDEN (___pg_registry_start = . );
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = . );
    PROVIDE_HIDDEN (___pg_registry_end = . );

    PROVIDE_HIDDEN (__pg_resetdata_start = . );
    PROVIDE_HIDDEN (___pg_resetdata_start = . );
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = . );
    PROVIDE_HIDDEN (___pg_resetdata_end = . );
  }
}
INSERT BEFORE .data;
