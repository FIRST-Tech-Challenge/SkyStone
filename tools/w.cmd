rem adb kill-server
rem adb wait-for-device
adb root
adb wait-for-device

@echo off
SETLOCAL ENABLEDELAYEDEXPANSION
SET count=1
FOR /F "tokens=* USEBACKQ" %%F IN (`adb shell ifconfig wlan0 `) DO (
  SET var!count!=%%F
  SET /a count=!count!+1
)

@rem ECHO %var2%

for /F "tokens=1,3 delims=: " %%a in ("%var2%") do (
   @rem echo %%a
   echo %%b
   set var=%%b
)
adb wait-for-device
adb tcpip 5555
adb wait-for-device
ping %var%
start "wifi keepalive" ping %var% -t
adb connect %var%:5555
sleep 2
adb devices