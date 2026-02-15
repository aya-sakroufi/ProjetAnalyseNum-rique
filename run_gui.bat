@echo off
echo ========================================================
echo      INSTALLATION DES DEPENDANCES ET LANCEMENT
echo ========================================================
echo.

echo Verification de Python...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Python n'est pas detecte dans le PATH.
    echo Essayons avec le lanceur 'py'...
    py --version >nul 2>&1
    if %errorlevel% neq 0 (
        echo ERREUR: Python n'est pas installe ou pas dans le PATH.
        echo Veuillez installer Python depuis python.org.
        pause
        exit /b
    )
    set PYTHON_CMD=py
) else (
    set PYTHON_CMD=python
)

echo.
echo Installation des librairies requises (PyQt6, Matplotlib)...
%PYTHON_CMD% -m pip install -r requirements.txt

echo.
echo Lancement de l'interface graphique...
%PYTHON_CMD% beamforming_gui.py

if %errorlevel% neq 0 (
    echo.
    echo Une erreur est survenue lors de l'execution.
    pause
)
