doxygen Doxyfile
xcopy ".\docs\html\refman.chm" ".\docs\" /Q /Y
rmdir ".\docs\html\" /s /q
xcopy ".\images\CyLogo.pdf" ".\docs\latex\" /Q /Y
call .\docs\latex\make.bat
xcopy ".\docs\latex\refman.pdf" ".\docs\" /Q /Y
rmdir ".\docs\latex\" /s /q

