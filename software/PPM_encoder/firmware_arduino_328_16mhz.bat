@echo *********************************
@echo Visit DIYdrones.com!
@echo This Bat is for ArduPilot PPM Enconder with At328@16Mhz! Date: July 12, 2010
@echo *********************************
@echo Press enter to do some Magic!
pause
:A
@echo Programming PPM Enconder...

"C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe" -cUSB -dATmega328P -fDFFF -EFE -FDFFF -GFE -e -iffirmware.hex -pf -lCF -LCF

@echo ****************************************************************
@echo Jordi: CHECK FOR PROBLEMS ABOVE before closing this window!
@echo Press enter to do the Magic again!
@echo ****************************************************************
pause
Goto A
