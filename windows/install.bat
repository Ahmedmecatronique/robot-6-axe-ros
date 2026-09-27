@echo off
rem Installe l'IHM Windows dans un environnement virtuel local (dossier windows\.venv).
rem Peut etre relance sans risque. Necessite Python 3.10 a 3.12 (python.org, cocher "Add python.exe to PATH").
setlocal
cd /d "%~dp0"

set "PY="
python --version >nul 2>nul && set "PY=python"
if not defined PY (
  py -3 --version >nul 2>nul && set "PY=py -3"
)
if not defined PY (
  echo.
  echo [ERREUR] Python est introuvable.
  echo Installez Python 3.10 a 3.12 depuis https://www.python.org/downloads/
  echo et cochez "Add python.exe to PATH" pendant l'installation, puis relancez ce fichier.
  echo ^(Si "python" ouvre le Microsoft Store, desactivez l'alias dans Parametres, Applications, Alias d'execution.^)
  exit /b 1
)
echo Python trouve : %PY%

if not exist ".venv\Scripts\python.exe" (
  echo Creation de l'environnement virtuel...
  %PY% -m venv .venv
  if errorlevel 1 (
    echo [ERREUR] Impossible de creer l'environnement virtuel.
    exit /b 1
  )
)

echo Installation des dependances ^(quelques minutes la premiere fois^)...
".venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 exit /b 1
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 (
  echo [ERREUR] Echec de l'installation des dependances. Verifiez la connexion Internet.
  exit /b 1
)

echo.
echo Installation terminee. Lancez l'IHM avec : run_gui.bat --demo   ^(ou : run_gui.bat --host IP_DU_RASPBERRY^)
endlocal
