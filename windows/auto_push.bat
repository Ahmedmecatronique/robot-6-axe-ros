@echo off
rem Double-cliquez : ajoute, commit et pousse les changements du projet vers GitHub (tools/auto_push.py).
rem Utilise Python + Git du systeme (pas l'environnement de l'IHM). Necessite Git installe et configure
rem (identifiants GitHub deja en place, ex. via "gh auth login").
setlocal
cd /d "%~dp0.."

where git >nul 2>nul
if errorlevel 1 (
  echo [ERREUR] Git est introuvable. Installez-le depuis https://git-scm.com/download/win
  pause
  exit /b 1
)

where python >nul 2>nul
if errorlevel 1 (
  echo [ERREUR] Python est introuvable ^(voir windows\install.bat pour l'installer^).
  pause
  exit /b 1
)

python tools\auto_push.py %*
if errorlevel 1 (
  echo.
  echo Le push a echoue ^(voir le message ci-dessus^).
)
pause
endlocal
