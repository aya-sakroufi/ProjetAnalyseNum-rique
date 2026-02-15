# Projet Analyse Numérique - Beamforming GUI

Ce projet implémente une interface graphique moderne pour la simulation et la comparaison d'algorithmes de Beamforming (Formation de faisceaux).

## Fonctionnalités

- **Simulation**: Configuration intuitive des antennes, murs et objectifs de beamforming.
- **Comparaison**: Analyse comparative entre la décomposition LU et le Pivot de Gauss.
- **Visualisation**: Graphiques interactifs (Polaire, Cartésien) intégrés.

## Installation et Exécution (VM / Linux / Windows)

1.  **Cloner le dépôt**:
    ```bash
    git clone https://github.com/aya-sakroufi/ProjetAnalyseNumBeamforming.git
    cd ProjetAnalyseNumBeamforming
    ```

2.  **Installer les dépendances**:
    ```bash
    pip install -r requirements.txt
    ```

3.  **Lancer l'application**:
    ```bash
    python beamforming_gui.py
    ```

## Structure du Projet

- `beamforming_gui.py`: Code principal de l'interface graphique (PyQt6).
- `version_valide.py`: Bibliothèque contenant la logique mathématique et physique.
- `requirements.txt`: Liste des dépendances Python.
- `run_gui.bat`: Script de lancement rapide pour Windows.
