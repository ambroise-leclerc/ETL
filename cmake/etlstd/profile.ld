/* Link-only profile identity. INFO keeps references out of flash/SRAM and Intel HEX output.
   KEEP preserves archive dependencies with --gc-sections, without startup code or constructors. */
SECTIONS
{
  .etl.profile 0 (INFO) : { KEEP(*(.etl.profile.*)) }
}
INSERT AFTER .comment;
