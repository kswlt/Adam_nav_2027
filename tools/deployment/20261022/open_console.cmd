@echo off
chcp 65001 >nul
ssh -t -p 2233 asus@192.168.1.164 "bash /home/asus/nav_2027_vehicle_config_164/runtime_20261022_fit153_console/start_console.bash"
pause
