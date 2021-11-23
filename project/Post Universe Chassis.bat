@echo off
D:\Keil_v5\ARM\ARMCC\bin\fromelf.exe --bin --output "Bin/pallet_check_xx_xx_xx.bin" "HC32/pallet_check_hc32f460jeua_app.axf"
CalculateFirmwareCRC 16 3976 "Bin/pallet_check_xx_xx_xx.bin"