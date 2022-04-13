REM Modifying Output File to Include Safety, Application, and Configuration CRCs, then generating 'srec' and 'hex' files. %1=TOOLKIT_DIR %2=OUTPUT_FILE
%1"\bin\ielftool.exe" --fill v;0xFF;LNK_ROM_START-LNK_ROM_END --checksum Safety_CRC:4,crc32:L,0xFFFFFFFF;LNK_ROM_START-LNK_ROM_END  %2 %2
%1"\bin\ielftool.exe" --fill v;0xFF;LNK_CFG_ROM_START-LNK_CFG_ROM_END --checksum Config_CRC:4,crc32:L,0xFFFFFFFF;LNK_CFG_ROM_START-LNK_CFG_ROM_END  %2 %2
%1"\bin\ielftool.exe" --fill v;0xFF;LNK_APP_ROM_START-LNK_APP_ROM_END --checksum Application_CRC:4,crc32:L,0xFFFFFFFF;LNK_APP_ROM_START-LNK_APP_ROM_END  %2 %2
%1"\bin\ielftool.exe" --srec --verbose %2 "%~dpn2.srec"
%1"\bin\ielftool.exe" --ihex --verbose %2 "%~dpn2.hex"
set RECORD_LENGTH=-obs=16
"%~dp0"srec_cat.exe "%~dpn2.hex" -Intel -crop 0x0800E800 0x08010000 -o "%~dpn2_f30x.hex" -Intel %RECORD_LENGTH%
REM Exporting Symbols file to allow use as library
%1"\bin\isymexport.exe" %2 "%~dpn2.symbols" --edit "%~dpn2_symbols.txt"
REM pause

