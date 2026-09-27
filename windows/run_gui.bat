@echo off
rem Lance l'IHM du bras 6 axes (Windows, sans ROS 2).
rem   run_gui.bat --demo                  robot simule dans le programme (aucun robot necessaire)
rem   run_gui.bat --host 192.168.1.50     se connecte au Raspberry Pi (rosbridge, port 9090)
rem Le premier lancement installe automatiquement l'environnement (quelques minutes).
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo Premier lancement : installation de l'environnement...
  call "%~dp0install.bat"
  if errorlevel 1 (
    echo.
    echo L'installation a echoue ^(voir le message ci-dessus^).
    pause
    exit /b 1
  )
)

".venv\Scripts\python.exe" run_gui.py %*
if errorlevel 1 (
  echo.
  echo L'IHM s'est arretee avec une erreur ^(voir le message ci-dessus^).
  pause
)
endlocal
