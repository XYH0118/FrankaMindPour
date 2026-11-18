@echo off
chcp 65001 >nul
cd /d "%~dp0"
if exist "%USERPROFILE%\anaconda3\python.exe" (
    "%USERPROFILE%\anaconda3\python.exe" -X utf8 fmp_udp_keyboard.py %*
) else (
    python -X utf8 fmp_udp_keyboard.py %*
)
pause
