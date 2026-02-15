##### %matplotlib inline

import numpy as np
from scipy.linalg import lu_factor, lu_solve
import matplotlib.pyplot as plt
import time
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch
import matplotlib.patches as mpatches
​
# ==================================================
# CONFIGURATION GLOBALE
# ==================================================
​
C = 3e8  # Vitesse de la lumière (m/s)
FREQ = 1e9  # Fréquence (Hz)
LAMBDA = C / FREQ  # Longueur d'onde (m)
K = 2 * np.pi / LAMBDA  # Nombre d'onde
​
​
# ==================================================
# CONFIGURATION GLOBALE - MURS ET MATÉRIAUX
# ==================================================
​
# Base de données des matériaux de murs
WALL_MATERIALS = {
    'concrete': {
        'name': 'Béton',
        'alpha_1GHz': 15.0,  # Atténuation en dB/m à 1 GHz
        'color': 'gray',
        'description': 'Béton armé standard'
    },
    'brick': {
        'name': 'Brique',
        'alpha_1GHz': 10.0,
        'color': 'brown',
        'description': 'Mur en briques'
    },
    'wood': {
        'name': 'Bois',
        'alpha_1GHz': 5.0,
        'color': 'sienna',
        'description': 'Cloison en bois'
    },
    'glass': {
        'name': 'Verre',
        'alpha_1GHz': 3.0,
        'color': 'lightblue',
        'description': 'Vitre/fenêtre'
    },
    'metal': {
        'name': 'Métal',
        'alpha_1GHz': 50.0,
        'color': 'silver',
        'description': 'Plaque métallique'
    }
}
​
def calculate_attenuation(material, thickness, frequency=FREQ, incidence_angle=0):
    """
    Calcule l'atténuation d'un mur automatiquement.
    
    Paramètres:
    -----------
    material : str
        Type de matériau ('concrete', 'brick', 'wood', 'glass', 'metal')
    thickness : float
        Épaisseur du mur en mètres
    frequency : float
        Fréquence du signal en Hz (défaut: FREQ global)
    incidence_angle : float
        Angle d'incidence en radians (0 = perpendiculaire)
    
    Retourne:
    ---------
    float
        Facteur d'atténuation linéaire (0-1)
        0 = blocage total, 1 = aucune atténuation
    
    Formule:
    --------
    Atténuation (dB) = α(f) × épaisseur × sec(θ)
    où α(f) = α₀ × (f/f₀)^0.5  (dépendance fréquentielle)
    """
    if material not in WALL_MATERIALS:
        print(f"⚠️  Matériau '{material}' inconnu, utilisation de 'concrete' par défaut")
        material = 'concrete'
    
    # Coefficient d'atténuation à la fréquence de référence (1 GHz)
    alpha_ref = WALL_MATERIALS[material]['alpha_1GHz']
    
    # Ajustement pour la fréquence actuelle (loi en racine carrée)
    freq_ratio = frequency / 1e9
    alpha_freq = alpha_ref * np.sqrt(freq_ratio)
    
    # Correction pour l'angle d'incidence (trajet plus long dans le mur)
    if np.abs(incidence_angle) < np.pi/2 - 0.01:  # Éviter division par zéro
        path_factor = 1.0 / np.cos(incidence_angle)
    else:
        path_factor = 10.0  # Angle rasant = forte atténuation
    
    # Atténuation totale en dB
    attenuation_dB = alpha_freq * thickness * path_factor
    
    # Conversion en facteur linéaire (0-1)
    # attenuation_linear = 10^(-attenuation_dB/20)
    attenuation_linear = 10 ** (-attenuation_dB / 20.0)
    
    # Limiter entre 0 et 1
    attenuation_linear = np.clip(attenuation_linear, 0.0, 1.0)
    
    return attenuation_linear
​
# ==================================================
# GÉNÉRATION AUTOMATIQUE DE A ET B
# ==================================================
​
​
def generer_systeme_beamforming(n, geometry='linear', radius=2.0, spacing=0.5,
                                 directions_principales=None, directions_nulls=None,
                                 poids_nulls=100.0):
    """Génère automatiquement la matrice A et le vecteur b pour un objectif de beamforming."""
    print("\n" + "═" * 70)
    print("   GÉNÉRATION AUTOMATIQUE DU SYSTÈME BEAMFORMING")
    print("═" * 70)
    
    antenne_positions = get_antenna_positions(n, geometry, radius, spacing)
    
    print(f"\n📐 Géométrie: {geometry.upper()}")
    print(f"📡 Nombre d'antennes: {n}")
    print(f"📏 Longueur d'onde λ = {LAMBDA:.3f} m")
    print(f"↔️  Espacement = {spacing}λ = {spacing * LAMBDA:.3f} m")
    
    if directions_principales is None:
        directions_principales = [30]
    if directions_nulls is None:
        directions_nulls = [-30, 60]
    
    print(f"\n🎯 Objectifs:")
    print(f"   ✅ Lobes principaux: {directions_principales}°")
    print(f"   ❌ Nulls (zeros): {directions_nulls}°")
    
    n_contraintes = len(directions_principales) + len(directions_nulls)
    
    A_list = []
    b_list = []
    
    for angle_deg in directions_principales:
        angle_rad = np.deg2rad(angle_deg)
        steering_vector = compute_steering_vector(antenne_positions, angle_rad)
        A_list.append(steering_vector)
        b_list.append(1.0 + 0j)
    
    for angle_deg in directions_nulls:
        angle_rad = np.deg2rad(angle_deg)
        steering_vector = compute_steering_vector(antenne_positions, angle_rad)
        A_list.append(steering_vector * np.sqrt(poids_nulls))
        b_list.append(0.0 + 0j)
    
    A_full = np.array(A_list)
    b_full = np.array(b_list)
    
    if n_contraintes >= n:
        A = A_full.conj().T @ A_full
        b = A_full.conj().T @ b_full
    else:
        A = A_full.T @ A_full + 0.01 * np.eye(n)
        b = A_full.T @ b_full
    
    print(f"\n📊 Système généré: A ({n}x{n}), b ({n},)")
    print(f"🔢 Conditionnement: {np.linalg.cond(A):.2e}")
    
    return A, b, antenne_positions
​
def get_antenna_positions(n, geometry='linear', radius=2.0, spacing=0.5):
    """Retourne les positions des antennes selon la géométrie choisie."""
    positions = []
    
    if geometry == 'linear':
        start_x = -(n-1) * spacing * LAMBDA / 2
        for k in range(n):
            positions.append(np.array([start_x + k * spacing * LAMBDA, 0.0]))
    
    elif geometry == 'circular':
        for k in range(n):
            angle = 2 * np.pi * k / n
            positions.append(np.array([radius * np.cos(angle), radius * np.sin(angle)]))
    
    elif geometry == 'random':
        np.random.seed(42)
        for _ in range(n):
            r = radius * np.sqrt(np.random.random())
            theta = 2 * np.pi * np.random.random()
            positions.append(np.array([r * np.cos(theta), r * np.sin(theta)]))
    
    return positions
​
def compute_steering_vector(positions, angle_rad):
    """Calcule le vecteur de pointage pour une direction donnée."""
    n = len(positions)
    sv = np.zeros(n, dtype=complex)
    direction = np.array([np.sin(angle_rad), np.cos(angle_rad)])
    
    for i, pos in enumerate(positions):
        phase = -K * np.dot(pos, direction)
        sv[i] = np.exp(1j * phase)
    
    return sv
​
def compute_array_factor(positions, x, angles_deg):
    """Calcule le facteur de réseau pour un ensemble d'angles."""
    angles_rad = np.deg2rad(angles_deg)
    af = np.zeros(len(angles_deg), dtype=complex)
    
    for i, angle_rad in enumerate(angles_rad):
        sv = compute_steering_vector(positions, angle_rad)
        af[i] = np.dot(sv, x)
    
    return af
​
# ==================================================
# HELPERS POUR GRANDES MATRICES
# ==================================================
​
def is_large_system(n):
    """Détermine si le système est considéré comme grand."""
    return n > 20
​
def estimate_computation_time(n):
    """Estime le temps de calcul approximatif."""
    # Approximation basée sur la complexité
    field_map_ops = 120 * 120 * n  # grid_x * grid_y * n_antennas
    return field_map_ops / 1e6  # Estimation grossière en secondes
​
def get_grid_resolution(n, user_choice=None):
    """
    Détermine la résolution de la grille en fonction de n.
    
    Retourne: (grid_size, description)
    """
    if user_choice == '1' or (user_choice is None and n > 50):
        return 60, 'basse (60x60)'
    elif user_choice == '3':
        return 240, 'haute (240x240)'
    else:
        return 120, 'normale (120x120)'
​
# ==================================================
# SAISIE DES PARAMÈTRES
# ==================================================
​
def configurer_murs_antennes(n, mode='complet'):
    """
    Configure les murs pour chaque antenne.
    
    Paramètres:
    -----------
    n : int
        Nombre d'antennes
    mode : str
        'complet' = configuration détaillée par antenne
        'simple' = même mur pour toutes les antennes
        'aucun' = pas de murs
    
    Retourne:
    ---------
    list : Liste de configurations de murs (None ou dict par antenne)
    """
    walls_config = []
    
    if mode == 'aucun':
        return [None] * n
    
    elif mode == 'simple':
        print("\n🧱 Configuration d'un mur commun pour toutes les antennes:")
        has_wall = input("   Mur présent? (o/n) : ").strip().lower()
        
        if has_wall == 'o':
            wall_x = float(input("   Position X du mur (m) : ") or 2.0)
            
            print("\n   Matériaux disponibles:")
            for i, (key, mat) in enumerate(WALL_MATERIALS.items(), 1):
                print(f"   {i}. {mat['name']} ({mat['description']}) - {mat['alpha_1GHz']} dB/m")
            
            mat_choice = input(f"   Choix du matériau (1-{len(WALL_MATERIALS)}) : ").strip()
            materials_list = list(WALL_MATERIALS.keys())
            material = materials_list[int(mat_choice)-1] if mat_choice.isdigit() and 1 <= int(mat_choice) <= len(WALL_MATERIALS) else 'concrete'
            
            thickness = float(input(f"   Épaisseur du mur (m, ex: 0.2) : ") or 0.2)
            
            wall_config = {'x': wall_x, 'material': material, 'thickness': thickness}
            walls_config = [wall_config.copy() for _ in range(n)]
        else:
            walls_config = [None] * n
    
    else:  # mode == 'complet'
        print(f"\n🧱 Configuration des murs par antenne ({n} antennes):")
        print("   (Appuyez Entrée pour 'non' si pas de mur)")
        
        for k in range(n):
            has_wall = input(f"\n   Antenne {k+1}: Mur présent? (o/n) : ").strip().lower()
            
            if has_wall == 'o':
                wall_x = float(input(f"      Position X du mur (m) : ") or 2.0)
                
                print("      Matériaux: 1=Béton 2=Brique 3=Bois 4=Verre 5=Métal")
                mat_choice = input(f"      Matériau (1-5) : ").strip()
                materials_list = list(WALL_MATERIALS.keys())
                material = materials_list[int(mat_choice)-1] if mat_choice.isdigit() and 1 <= int(mat_choice) <= 5 else 'concrete'
                
                thickness = float(input(f"      Épaisseur (m, ex: 0.2) : ") or 0.2)
                
                walls_config.append({'x': wall_x, 'material': material, 'thickness': thickness})
            else:
                walls_config.append(None)
    
    return walls_config
​
​
def saisie_personnalisee():
    print("\n" + "═" * 90)
    print("          MODE PERSONNALISÉ – SAISIE MANUELLE")
    print("═" * 90)
    print("\nVous définissez vous-même tous les paramètres :")
    print("• Nombre d'antennes (n)")
    print("• Matrice A (ligne par ligne)")
    print("• Vecteur b")
    print("• Murs par antenne (position, matériau, épaisseur)")
    print("• Bruit ajouté sur b")
    print("═" * 90 + "\n")
​
    while True:
        try:
            n = int(input("Nombre d'antennes (dimension n) : "))
            if n < 1:
                print("Erreur : n doit être au moins 1")
                continue
            break
        except:
            print("Veuillez entrer un entier positif")
​
    print(f"\n📋 Entrez la matrice A ({n} × {n}) – ligne par ligne")
    ex_val = "1+0j"
    ex_row = "  ".join([ex_val] * min(n, 3)) + (" ..." if n > 3 else "")
    print(f"   Exemple : {ex_row}")
    A = np.zeros((n, n), dtype=complex)
    for i in range(n):
        while True:
            try:
                ligne = input(f"   Ligne {i+1} ({n} valeurs) : ").strip()
                valeurs = [complex(v) for v in ligne.split()]
                if len(valeurs) != n:
                    print(f"   Erreur : {len(valeurs)} valeurs au lieu de {n}")
                    continue
                A[i] = valeurs
                break
            except Exception as e:
                print(f"   Erreur format : {e}")
​
    print(f"\n📋 Entrez le vecteur b ({n} valeurs)")
    print(f"   Exemple : {ex_row}")
    while True:
        try:
            b_str = input("   Vecteur b : ").strip()
            b = np.array([complex(v) for v in b_str.split()], dtype=complex)
            if len(b) != n:
                print(f"   Erreur : {len(b)} valeurs au lieu de {n}")
                continue
            break
        except Exception as e:
            print(f"   Erreur format : {e}")
​
    # Configuration des murs
    print("\n🧱 Configuration des murs:")
    print("   1 → Mur simple (même pour toutes les antennes)")
    print("   2 → Murs différents par antenne")
    print("   3 → Aucun mur")
    wall_mode_choice = input("   Choix (1-3) : ").strip()
    
    if wall_mode_choice == '1':
        walls_config = configurer_murs_antennes(n, mode='simple')
    elif wall_mode_choice == '2':
        walls_config = configurer_murs_antennes(n, mode='complet')
    else:
        walls_config = configurer_murs_antennes(n, mode='aucun')
​
    noise_level = float(input("\n🔊 Bruit à ajouter sur b (ex: 0.05, 0 pour aucun) : ") or 0.05)
    b += noise_level * (np.random.randn(n) + 1j * np.random.randn(n))
​
    # Affichage récapitulatif
    print("\n" + "═" * 90)
    print("         📋 RÉCAPITULATIF DES PARAMÈTRES SAISIS")
    print("═" * 90)
    print(f"📡 Nombre d'antennes (n) : {n}")
    
    print("\n📐 Matrice A :")
    for i in range(n):
        ligne = "  ".join(f"{A[i,j]:.4f}" for j in range(n))
        print(f"   Ligne {i+1} : {ligne}")
    
    print("\n📊 Vecteur b (après bruit) :")
    print("   b = " + "  ".join(f"{val:.4f}" for val in b))
    
    # Affichage des murs
    print("\n🧱 Configuration des murs :")
    n_walls = sum(1 for w in walls_config if w is not None)
    if n_walls == 0:
        print("   Aucun mur configuré")
    else:
        for k, wall in enumerate(walls_config):
            if wall is not None:
                mat_name = WALL_MATERIALS[wall['material']]['name']
                print(f"   Antenne {k+1}: Mur à X={wall['x']:.2f}m, {mat_name}, ép={wall['thickness']:.2f}m")
    
    print(f"\n🔊 Bruit ajouté : {noise_level:.4f}")
    print("═" * 90 + "\n")
​
    return A, b, n, noise_level, walls_config
​
def saisie_hybride():
    """Mode hybride: saisie manuelle de A et b + configuration automatique de la géométrie."""
    print("\n" + "═" * 90)
    print("          MODE HYBRIDE – SAISIE A/b + GÉOMÉTRIE AUTOMATIQUE")
    print("═" * 90)
    print("\nCe mode combine:")
    print("• Saisie manuelle de la matrice A et du vecteur b")
    print("• Configuration automatique de la géométrie des antennes")
    print("• Toutes les fonctionnalités avancées (positions, vecteurs de pointage, etc.)")
    print("═" * 90 + "\n")
    
    # Nombre d'antennes
    while True:
        try:
            n = int(input("📡 Nombre d'antennes (n) : "))
            if n < 1:
                print("Erreur : n doit être au moins 1")
                continue
            break
        except:
            print("Veuillez entrer un entier positif")
    
    # Géométrie des antennes
    print("\n📐 Géométrie des antennes:")
    print("   1 → Linéaire (ULA)")
    print("   2 → Circulaire (UCA)")
    print("   3 → Aléatoire")
    geo_choice = input("   Choix (1-3) : ").strip()
    
    geometry = 'linear'
    radius = 2.0
    spacing = 0.5
    
    if geo_choice == '2':
        geometry = 'circular'
        radius = float(input(f"   Rayon du cercle (m, ex: 2.0) : ") or 2.0)
    elif geo_choice == '3':
        geometry = 'random'
        radius = float(input(f"   Rayon de dispersion (m, ex: 2.0) : ") or 2.0)
    else:
        spacing = float(input(f"   Espacement d/λ (ex: 0.5) : ") or 0.5)
    
    # Calcul des positions des antennes
    antenne_positions = get_antenna_positions(n, geometry, radius, spacing)
    
    print(f"\n✅ Positions des antennes calculées ({geometry.upper()})")
    print(f"   Nombre d'antennes: {n}")
    if geometry == 'linear':
        print(f"   Espacement: {spacing}λ = {spacing * LAMBDA:.3f} m")
    else:
        print(f"   Rayon: {radius:.2f} m")
    
    # Saisie manuelle de A
    print(f"\n📋 Entrez la matrice A ({n} × {n}) – ligne par ligne")
    ex_val = "1+0j"
    ex_row = "  ".join([ex_val] * min(n, 3)) + (" ..." if n > 3 else "")
    print(f"   Exemple : {ex_row}")
    A = np.zeros((n, n), dtype=complex)
    for i in range(n):
        while True:
            try:
                ligne = input(f"   Ligne {i+1} ({n} valeurs) : ").strip()
                valeurs = [complex(v) for v in ligne.split()]
                if len(valeurs) != n:
                    print(f"   Erreur : {len(valeurs)} valeurs au lieu de {n}")
                    continue
                A[i] = valeurs
                break
            except Exception as e:
                print(f"   Erreur format : {e}")
    
    # Saisie manuelle de b
    print(f"\n📋 Entrez le vecteur b ({n} valeurs)")
    print(f"   Exemple : {ex_row}")
    while True:
        try:
            b_str = input("   Vecteur b : ").strip()
            b = np.array([complex(v) for v in b_str.split()], dtype=complex)
            if len(b) != n:
                print(f"   Erreur : {len(b)} valeurs au lieu de {n}")
                continue
            break
        except Exception as e:
            print(f"   Erreur format : {e}")
    
    # Paramètres du mur et bruit
    print("\n🧱 Configuration des murs:")
    print("   1 → Mur simple (même pour toutes les antennes)")
    print("   2 → Murs différents par antenne")
    print("   3 → Aucun mur")
    wall_mode_choice = input("   Choix (1-3) : ").strip()
    
    if wall_mode_choice == '1':
        walls_config = configurer_murs_antennes(n, mode='simple')
    elif wall_mode_choice == '2':
        walls_config = configurer_murs_antennes(n, mode='complet')
    else:
        walls_config = configurer_murs_antennes(n, mode='aucun')
    
    noise_level = float(input("\n🔊 Bruit à ajouter sur b (ex: 0.05, 0 pour aucun) : ") or 0.05)
    b += noise_level * (np.random.randn(n) + 1j * np.random.randn(n))
    
    # Affichage récapitulatif détaillé
    print("\n" + "═" * 90)
    print("         📋 RÉCAPITULATIF COMPLET - MODE HYBRIDE")
    print("═" * 90)
    print(f"\n📡 Configuration des antennes:")
    print(f"   • Nombre d'antennes: {n}")
    print(f"   • Géométrie: {geometry.upper()}")
    if geometry == 'linear':
        print(f"   • Espacement: {spacing}λ = {spacing * LAMBDA:.3f} m")
    else:
        print(f"   • Rayon: {radius:.2f} m")
    
    print(f"\n📍 Positions des antennes:")
    for i, pos in enumerate(antenne_positions[:min(6, len(antenne_positions))]):
        print(f"   Antenne {i+1}: ({pos[0]:.3f}, {pos[1]:.3f}) m")
    if len(antenne_positions) > 6:
        print(f"   ... ({len(antenne_positions)-6} autres antennes)")
    
    print("\n📐 Matrice A :")
    for i in range(n):
        ligne = "  ".join(f"{A[i,j]:.4f}" for j in range(n))
        print(f"   Ligne {i+1} : {ligne}")
    
    print("\n📊 Vecteur b (après bruit) :")
    print("   b = " + "  ".join(f"{val:.4f}" for val in b))
    
    # Affichage des murs
    print("\n🧱 Configuration des murs :")
    n_walls = sum(1 for w in walls_config if w is not None)
    if n_walls == 0:
        print("   Aucun mur configuré")
    else:
        for k, wall in enumerate(walls_config):
            if wall is not None:
                mat_name = WALL_MATERIALS[wall['material']]['name']
                print(f"   Antenne {k+1}: Mur à X={wall['x']:.2f}m, {mat_name}, ép={wall['thickness']:.2f}m")
    
    print(f"\n🔊 Bruit ajouté : {noise_level:.4f}")
    print(f"🔢 Conditionnement de A : {np.linalg.cond(A):.2e}")
    print("═" * 90 + "\n")
    
    return A, b, n, noise_level, walls_config, geometry, antenne_positions
​
​
def saisie_automatique_beamforming():
    """Mode automatique: génération de A et b à partir d'objectifs."""
    print("\n" + "═" * 90)
    print("          MODE AUTOMATIQUE – GÉNÉRATION PAR OBJECTIFS")
    print("═" * 90)
    print("\nDéfinissez vos objectifs de beamforming:")
    print("• Directions des lobes principaux")
    print("• Directions des nulls (interférences à rejeter)")
    print("• La matrice A et le vecteur b seront générés automatiquement")
    print("═" * 90 + "\n")
    
    while True:
        try:
            n = int(input("📡 Nombre d'antennes (n) : "))
            if n < 2:
                print("Erreur : n doit être au moins 2")
                continue
            break
        except:
            print("Veuillez entrer un entier positif")
    
    print("\n📐 Géométrie des antennes:")
    print("   1 → Linéaire (ULA)")
    print("   2 → Circulaire (UCA)")
    print("   3 → Aléatoire")
    geo_choice = input("   Choix (1-3) : ").strip()
    
    geometry = 'linear'
    radius = 2.0
    spacing = 0.5
    
    if geo_choice == '2':
        geometry = 'circular'
        radius = float(input(f"   Rayon du cercle (m, ex: 2.0) : ") or 2.0)
    elif geo_choice == '3':
        geometry = 'random'
        radius = float(input(f"   Rayon de dispersion (m, ex: 2.0) : ") or 2.0)
    else:
        spacing = float(input(f"   Espacement d/λ (ex: 0.5) : ") or 0.5)
    
    print("\n🎯 Objectifs de beamforming:")
    
    print("\n   Directions des lobes principaux (angles en degrés)")
    print("   Exemple: 0 30 (pour lobes à 0° et 30°)")
    print("   Appuyez Entrée pour valeur par défaut: 30")
    lobes_str = input("   Angles lobes principaux : ").strip()
    if lobes_str:
        directions_principales = [float(x) for x in lobes_str.split()]
    else:
        directions_principales = [30]
    
    print("\n   Directions des nulls (angles à rejeter)")
    print("   Exemple: -30 60 (pour nulls à -30° et 60°)")
    print("   Appuyez Entrée pour valeur par défaut: -30 60")
    nulls_str = input("   Angles nulls : ").strip()
    if nulls_str:
        directions_nulls = [float(x) for x in nulls_str.split()]
    else:
        directions_nulls = [-30, 60]
    
    poids_nulls = float(input("\n⚖️  Pénalisation des nulls (ex: 100) : ") or 100.0)
    
    A, b, antenne_positions = generer_systeme_beamforming(
        n, geometry, radius, spacing, directions_principales, directions_nulls, poids_nulls
    )
    
    # Configuration des murs
    print("\n🧱 Configuration des murs:")
    print("   1 → Mur simple (même pour toutes les antennes)")
    print("   2 → Murs différents par antenne")
    print("   3 → Aucun mur")
    wall_mode_choice = input("   Choix (1-3) : ").strip()
    
    if wall_mode_choice == '1':
        walls_config = configurer_murs_antennes(n, mode='simple')
    elif wall_mode_choice == '2':
        walls_config = configurer_murs_antennes(n, mode='complet')
    else:
        walls_config = configurer_murs_antennes(n, mode='aucun')
    
    noise_level = float(input("\n🔊 Bruit à ajouter sur b (ex: 0.05, 0 pour aucun) : ") or 0.05)
    b += noise_level * (np.random.randn(n) + 1j * np.random.randn(n))
    
    print("\n" + "═" * 90)
    print("         ✅ SYSTÈME GÉNÉRÉ AVEC SUCCÈS")
    print("═" * 90)
    print(f"📡 Nombre d'antennes: {n}")
    print(f"📐 Géométrie: {geometry.upper()}")
    print(f"🎯 Lobes principaux: {directions_principales}°")
    print(f"🚫 Nulls: {directions_nulls}°")
    
    # Affichage des murs
    n_walls = sum(1 for w in walls_config if w is not None)
    if n_walls == 0:
        print("🧱 Aucun mur configuré")
    else:
        print(f"🧱 {n_walls} mur(s) configuré(s)")
        
    print(f"🔊 Bruit ajouté : {noise_level:.4f}")
    print(f"🔢 Conditionnement de A : {np.linalg.cond(A):.2e}")
    print("═" * 90 + "\n")
    
    return A, b, n, noise_level, walls_config, geometry, antenne_positions, directions_principales, directions_nulls
​
​
def charger_exemple():
    """Charge un exemple pré-configuré."""
    print("\n" + "═" * 70)
    print("   EXEMPLES PRÉ-CONFIGURÉS")
    print("═" * 70)
    print("1 → Beamforming simple (4 antennes, lobe à 30°)")
    print("2 → Rejection d'interférence (8 antennes, null à -20°)")
    print("3 → Multiples lobes (6 antennes, lobes à -45° et 45°)")
    
    choix = input("   Choix (1-3) : ").strip()
    
    if choix == '1':
        n = 4
        geometry = 'linear'
        radius = 2.0
        directions_principales = [30]
        directions_nulls = [-30]
        description = "Beamforming simple avec lobe principal à 30°"
    elif choix == '2':
        n = 8
        geometry = 'linear'
        radius = 2.0
        directions_principales = [0]
        directions_nulls = [-20, 20]
        description = "Rejection d'interférence avec nulls à ±20°"
    else:
        n = 6
        geometry = 'circular'
        radius = 1.5
        directions_principales = [-45, 45]
        directions_nulls = [0]
        description = "Multiples lobes à -45° et 45°"
    
    A, b, antenne_positions = generer_systeme_beamforming(
        n, geometry, radius if geometry == 'circular' else 2.0, 0.5,
        directions_principales, directions_nulls, 100.0
    )
    
    
    # Configuration mur par défaut pour l'exemple (Mur en béton à 2m)
    wall_config = {'x': 2.0, 'material': 'concrete', 'thickness': 0.2}
    walls_config = [wall_config.copy() for _ in range(n)]
    
    noise_level = 0.05
    b += noise_level * (np.random.randn(n) + 1j * np.random.randn(n))
    
    # Affichage détaillé de l'exemple
    print("\n" + "═" * 90)
    print("         📋 DÉTAILS DE L'EXEMPLE PRÉ-CONFIGURÉ")
    print("═" * 90)
    print(f"\n📝 Description: {description}")
    print(f"\n📡 Configuration:")
    print(f"   • Nombre d'antennes: {n}")
    print(f"   • Géométrie: {geometry.upper()}")
    print(f"   • Rayon/Espacement: {radius if geometry == 'circular' else 0.5}{'m' if geometry == 'circular' else 'λ'}")
    print(f"   • Lobes principaux: {directions_principales}°")
    print(f"   • Nulls: {directions_nulls}°")
    
    print(f"\n📍 Positions des antennes:")
    for i, pos in enumerate(antenne_positions[:min(6, len(antenne_positions))]):
        print(f"   Antenne {i+1}: ({pos[0]:.3f}, {pos[1]:.3f}) m")
    if len(antenne_positions) > 6:
        print(f"   ... ({len(antenne_positions)-6} autres antennes)")
    
    print(f"\n📐 Matrice A ({n}×{n}):")
    for i in range(min(4, n)):
        ligne = "  ".join(f"{A[i,j]:.4f}" for j in range(min(4, n)))
        if n > 4:
            ligne += "  ..."
        print(f"   Ligne {i+1}: {ligne}")
    if n > 4:
        print(f"   ... ({n-4} autres lignes)")
    
    print(f"\n📊 Vecteur b (après bruit):")
    b_str = "  ".join(f"{val:.4f}" for val in b[:min(6, len(b))])
    if len(b) > 6:
        b_str += "  ..."
    print(f"   b = {b_str}")
    
    print(f"\n🧱 Environnement:")
    print(f"   • Mur: Béton à {wall_config['x']:.2f}m (Épaisseur: {wall_config['thickness']:.2f}m)")
    print(f"   • Bruit ajouté: {noise_level}")
    print(f"\n🔢 Conditionnement de A: {np.linalg.cond(A):.2e}")
    print("═" * 90 + "\n")
    
    return A, b, n, noise_level, walls_config, geometry, antenne_positions, directions_principales, directions_nulls
​
# ==================================================
# CALCUL DU CHAMP
# ==================================================
​
def traverse_mur(antenne_pos, point_pos, wall_config):
    """
    Détecte si le trajet traverse un mur et retourne les paramètres du mur.
    
    Paramètres:
    -----------
    antenne_pos : array
        Position de l'antenne [x, y]
    point_pos : array
        Position du point d'observation [x, y]
    wall_config : dict or None
        Configuration du mur: {'x': position, 'material': str, 'thickness': float}
        ou None si pas de mur
    
    Retourne:
    ---------
    tuple : (traverse, material, thickness, angle)
        traverse : bool - True si le trajet traverse le mur
        material : str - Type de matériau
        thickness : float - Épaisseur du mur
        angle : float - Angle d'incidence en radians
    """
    if wall_config is None:
        return False, None, None, None
    
    wall_x = wall_config['x']
    x1, y1 = antenne_pos
    x2, y2 = point_pos
    
    # Vérifier si le trajet traverse le mur
    if (x1 <= wall_x and x2 <= wall_x) or (x1 >= wall_x and x2 >= wall_x):
        return False, None, None, None
    
    if min(x1, x2) <= wall_x <= max(x1, x2):
        # Calculer l'angle d'incidence
        dx = x2 - x1
        dy = y2 - y1
        
        if abs(dx) > 1e-10:
            # Angle du trajet par rapport à l'horizontale
            traj_angle = np.arctan2(dy, dx)
            # Angle d'incidence par rapport à la normale du mur (verticale)
            incidence_angle = abs(traj_angle)
        else:
            incidence_angle = np.pi / 2  # Trajet vertical
        
        return True, wall_config['material'], wall_config['thickness'], incidence_angle
    
    return False, None, None, None
​
​
def compute_field_map(x, n, walls_config=None, antenne_positions=None, grid_size=120, show_progress=False):
    """
    Calcule la carte de champ avec prise en compte des murs par antenne.
    
    Paramètres:
    -----------
    x : array
        Poids complexes des antennes
    n : int
        Nombre d'antennes
    walls_config : list of dict or None
        Liste de configurations de murs, une par antenne
        walls_config[k] = {'x': position, 'material': str, 'thickness': float} ou None
    antenne_positions : list of array or None
        Positions des antennes
    grid_size : int
        Taille de la grille (60, 120, ou 240)
    show_progress : bool
        Afficher la progression pour les grands systèmes
    
    Retourne:
    ---------
    tuple : (intensity, grid_x, grid_y, antenne_positions)
    """
    grid_x = np.linspace(-6, 6, grid_size)
    grid_y = np.linspace(0, 15, grid_size)
    nx, ny = len(grid_x), len(grid_y)
    intensity = np.zeros((ny, nx))
    
    if show_progress:
        print(f"   🔄 Calcul de la carte de champ ({grid_size}x{grid_size})...")
    
    if antenne_positions is None:
        spacing = 0.5 * LAMBDA
        start_x = -(n-1) * spacing / 2
        antenne_positions = [np.array([start_x + k * spacing, 0.0]) for k in range(n)]
    
    if walls_config is None:
        walls_config = [None] * n
    
    for ix in range(nx):
        if show_progress and ix % (nx // 10) == 0:
            progress = int(100 * ix / nx)
            print(f"      Progression: {progress}%", end='\r')
        
        for iy in range(ny):
            p = np.array([grid_x[ix], grid_y[iy]])
            field = 0j
            
            for k in range(n):
                antenne_pos = antenne_positions[k]
                dist = np.linalg.norm(p - antenne_pos)
                prop = np.exp(-1j * K * dist) / (dist + 1e-10)
                
                # Vérifier si le trajet traverse un mur pour cette antenne
                if walls_config[k] is not None:
                    traverses, material, thickness, angle = traverse_mur(
                        antenne_pos, p, walls_config[k]
                    )
                    if traverses:
                        # Calculer l'atténuation automatiquement
                        atten_factor = calculate_attenuation(material, thickness, FREQ, angle)
                        prop *= atten_factor
                
                field += x[k] * prop
            
            intensity[iy, ix] = np.abs(field)**2
    
    if show_progress:
        print("      Progression: 100% ✓")
    
    return intensity, grid_x, grid_y, antenne_positions
​
def compute_field_cut_y(x, n, walls_config=None, antenne_positions=None, y_cut=10.0, nx=200):
    """
    Calcule le champ uniquement le long d'une ligne horizontale à Y = y_cut.
    Beaucoup plus rapide que compute_field_map pour les visualisations 1D.
    
    Retourne:
    ---------
    tuple : (intensities, grid_x)
    """
    grid_x = np.linspace(-6, 6, nx)
    intensities = np.zeros(nx)
    
    if antenne_positions is None:
        spacing = 0.5 * LAMBDA
        start_x = -(n-1) * spacing / 2
        antenne_positions = [np.array([start_x + k * spacing, 0.0]) for k in range(n)]
    
    if walls_config is None:
        walls_config = [None] * n
    
    # Pré-calcul des constantes si possible pour optimiser
    # Mais ici on garde la logique simple pour supporter les murs
    
    p_y = y_cut
    
    for ix in range(nx):
        p_x = grid_x[ix]
        p = np.array([p_x, p_y])
        field = 0j
        
        for k in range(n):
            antenne_pos = antenne_positions[k]
            dist = np.linalg.norm(p - antenne_pos)
            prop = np.exp(-1j * K * dist) / (dist + 1e-10)
            
            # Vérifier murs
            if walls_config[k] is not None:
                traverses, material, thickness, angle = traverse_mur(
                    antenne_pos, p, walls_config[k]
                )
                if traverses:
                    atten_factor = calculate_attenuation(material, thickness, FREQ, angle)
                    prop *= atten_factor
            
            field += x[k] * prop
            
        intensities[ix] = np.abs(field)**2
        
    return intensities, grid_x
​
def compute_field_cut_x(x, n, walls_config=None, antenne_positions=None, x_cut=0.0, ny=200):
    """
    Calcule le champ uniquement le long d'une ligne verticale à X = x_cut.
    
    Retourne:
    ---------
    tuple : (intensities, grid_y)
    """
    grid_y = np.linspace(0, 15, ny) # De 0 à 15m (zone typique d'intérêt)
    intensities = np.zeros(ny)
    
    if antenne_positions is None:
        spacing = 0.5 * LAMBDA
        start_x = -(n-1) * spacing / 2
        antenne_positions = [np.array([start_x + k * spacing, 0.0]) for k in range(n)]
    
    if walls_config is None:
        walls_config = [None] * n
    
    p_x = x_cut
    
    for iy in range(ny):
        p_y = grid_y[iy]
        p = np.array([p_x, p_y])
        field = 0j
        
        for k in range(n):
            antenne_pos = antenne_positions[k]
            dist = np.linalg.norm(p - antenne_pos)
            prop = np.exp(-1j * K * dist) / (dist + 1e-10)
            
            # Vérifier murs
            if walls_config[k] is not None:
                traverses, material, thickness, angle = traverse_mur(
                    antenne_pos, p, walls_config[k]
                )
                if traverses:
                    atten_factor = calculate_attenuation(material, thickness, FREQ, angle)
                    prop *= atten_factor
            
            field += x[k] * prop
            
        intensities[iy] = np.abs(field)**2
        
    return intensities, grid_y
​
​
​
# ==================================================
# MÉTRIQUES
# ==================================================
​
def calculer_metriques(x, A, b, noise_level, walls_config=None, antenne_positions=None):
    residual = np.linalg.norm(A @ x - b)
    puissance = np.linalg.norm(x)**2
    
    # Optimisation: éviter le calcul complet de la carte de champ pour les métriques
    # si le système est grand. On utilise une coupe horizontale pour estimer le gain.
    n = len(b)
    if is_large_system(n):
        # Pour les grands systèmes, on estime le gain max sur une coupe
        intensities, _ = compute_field_cut_y(x, n, walls_config, antenne_positions, y_cut=10.0, nx=100)
    else:
        # Pour les petits systèmes, on peut se permettre la carte complète (plus précis)
        intensities, _, _, _ = compute_field_map(x, n, walls_config, antenne_positions, grid_size=60)
        intensities = intensities.flatten()
        
    gain_max = np.max(intensities)
    
    # Estimation du plancher de bruit (valeurs faibles)
    # On prend les 10% des valeurs les plus faibles comme référence de bruit
    threshold = np.percentile(intensities, 10)
    noise_idx = intensities < (threshold * 1.5) # Marge
    
    if np.any(noise_idx):
        noise_floor = np.mean(intensities[noise_idx]) + noise_level**2
    else:
        noise_floor = noise_level**2
        
    snr = 10 * np.log10(gain_max / (noise_floor + 1e-12)) if noise_floor > 0 else float('inf')
    
    # Calcul du RSB (rapport signal sur bruit) du système
    signal_power = np.linalg.norm(b)**2
    noise_power = noise_level**2 * len(b)
    system_snr = 10 * np.log10(signal_power / (noise_power + 1e-12)) if noise_power > 0 else float('inf')
    
    return {
        "Erreur résiduelle ||Ax-b||": f"{residual:.2e}",
        "Puissance émise totale ||x||²": f"{puissance:.6f}",
        "Gain maximal (est.)": f"{gain_max:.4f}",
        "SNR champ (dB, est.)": f"{snr:.2f}" if snr != float('inf') else "Inf",
        "SNR système (dB)": f"{system_snr:.2f}" if system_snr != float('inf') else "Inf"
    }
​
​
# ==================================================
# VISUALISATIONS AMÉLIORÉES
# ==================================================
​
​
# ==================================================
# VISUALISATIONS SIMPLIFIÉES (1D)
# ==================================================
​
def visualiser_champ_detaille(x, n, walls_config=None, antenne_positions=None,
                               titre="Analyse", methode="", save_path=None,
                               directions_principales=None, directions_nulls=None):
    """
    Affiche les graphiques essentiels (1D uniquement) :
    1. Diagramme de rayonnement (Cartésien)
    2. Diagramme Polaire
    3. Coupe Horizontale (Axe X, Y=10m)
    4. Coupe Verticale (Axe Y, X=0m)
    5. Infos et Poids
    """
    if antenne_positions is None:
        # Default fallback
        spacing = 0.5 * LAMBDA
        start_x = -(n-1) * spacing / 2
        antenne_positions = [np.array([start_x + k * spacing, 0.0]) for k in range(n)]
​
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle(f"{titre} - Méthode: {methode}", fontsize=16, fontweight='bold', y=0.98)
    
    # GridSpec pour layout personnalisé (2 lignes, 3 colonnes)
    # Ligne 1: Cartésien | Polaire | Infos
    # Ligne 2: Coupe X    | Coupe Y | (Vide)
    
    gs = fig.add_gridspec(2, 3)
    
    # 1. Diagramme de rayonnement (Cartésien)
    ax1 = fig.add_subplot(gs[0, 0])
    angles = np.linspace(-90, 90, 361)
    af = compute_array_factor(antenne_positions, x, angles)
    gain_db = 20 * np.log10(np.abs(af) + 1e-12)
    
    ax1.plot(angles, gain_db, 'b-', linewidth=2, label='Gain')
    
    if directions_principales:
        for angle in directions_principales:
            ax1.axvline(x=angle, color='green', linestyle='--', alpha=0.6, linewidth=2)
            idx = np.argmin(np.abs(angles - angle))
            ax1.text(angle, gain_db[idx] + 2, f"{angle}°", ha='center', va='bottom', color='green', fontsize=8)
            
    if directions_nulls:
        for angle in directions_nulls:
            ax1.axvline(x=angle, color='red', linestyle=':', alpha=0.6, linewidth=2)
            
    ax1.set_xlabel('Angle (degrés)')
    ax1.set_ylabel('Gain (dB)')
    ax1.set_title('Diagramme de Rayonnement', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-90, 90)
    
    # 2. Diagramme Polaire (Restauré)
    ax2 = fig.add_subplot(gs[0, 1], projection='polar')
    angles_rad = np.deg2rad(angles)
    ax2.plot(angles_rad, np.abs(af)/np.max(np.abs(af)), 'b-', linewidth=2)
    ax2.set_theta_zero_location('N')
    ax2.set_theta_direction(-1)
    ax2.set_title('Diagramme Polaire (Normalisé)', fontsize=12, fontweight='bold', pad=10)
    
    # 3. Coupe Horizontale (Axe X) à Y=10m
    target_y_cut = 10.0
    ax3 = fig.add_subplot(gs[1, 0])
    intensities_h, x_vals_h = compute_field_cut_y(x, n, walls_config, antenne_positions, y_cut=target_y_cut, nx=300)
    intensities_db_h = 10 * np.log10(intensities_h + 1e-12)
    intensities_db_h = intensities_db_h - np.max(intensities_db_h)
    
    ax3.plot(x_vals_h, intensities_db_h, 'purple', linewidth=2)
    ax3.set_title(f'Coupe horizontale à Y={target_y_cut}m', fontsize=12, fontweight='bold')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Intensité (dB)')
    ax3.grid(True, alpha=0.3)
    
    # Affichage position des murs sur coupe X
    if walls_config and any(w is not None for w in walls_config):
        first_wall = next((w for w in walls_config if w is not None), None)
        if first_wall:
             ax3.axvline(x=first_wall['x'], color='blue', linestyle='--', alpha=0.5, label='Mur')
             ax3.legend()
    
    # 4. Coupe Verticale (Axe Y) à X=0m
    target_x_cut = 0.0
    ax4 = fig.add_subplot(gs[1, 1])
    intensities_v, y_vals_v = compute_field_cut_x(x, n, walls_config, antenne_positions, x_cut=target_x_cut, ny=300)
    intensities_db_v = 10 * np.log10(intensities_v + 1e-12)
    intensities_db_v = intensities_db_v - np.max(intensities_db_v)
    
    ax4.plot(y_vals_v, intensities_db_v, 'orange', linewidth=2)
    ax4.set_title(f'Coupe verticale à X={target_x_cut}m', fontsize=12, fontweight='bold')
    ax4.set_xlabel('Y (m)')
    ax4.set_ylabel('Intensité (dB)')
    ax4.grid(True, alpha=0.3)
​
    # 5. Infos et Poids (Tableau propre) - Prend toute la colonne droite
    ax5 = fig.add_subplot(gs[:, 2])
    ax5.axis('off')
    
    # Préparation des données pour le tableau
    cell_text = []
    
    # Section Simulation
    cell_text.append(["INFORMATIONS DE SIMULATION", ""])
    cell_text.append(["Méthode", methode])
    cell_text.append(["Nombre d'antennes", str(n)])
    cell_text.append(["Fréquence", f"{FREQ/1e9:.1f} GHz"])
    cell_text.append(["Longueur d'onde λ", f"{LAMBDA:.3f} m"])
    
    # Section Mur
    cell_text.append(["", ""])
    cell_text.append(["PARAMÈTRES DU MUR", ""])
    if walls_config and any(w is not None for w in walls_config):
        w = next((w for w in walls_config if w is not None), None)
        cell_text.append(["Position X", f"{w['x']:.2f} m"])
        cell_text.append(["Matériau", f"{w['material']}"])
        cell_text.append(["Épaisseur", f"{w['thickness']:.2f} m"])
    else:
        cell_text.append(["Mur", "Aucun"])
        
    # Section Poids
    cell_text.append(["", ""])
    cell_text.append(["POIDS DES ANTENNES (module)", ""])
    norm_x = np.linalg.norm(x)
    cell_text.append(["Norme du vecteur", f"{norm_x:.4f}"])
    
    for i, val in enumerate(x[:min(12, len(x))]): # Plus de place pour afficher plus d'antennes
        cell_text.append([f"Antenne {i+1}", f"{np.abs(val):.4f}"])
    if len(x) > 12:
        cell_text.append(["...", f"({len(x)-12} autres)"])
​
    # Création du tableau
    table = ax5.table(cellText=cell_text, colLabels=None, loc='center', cellLoc='left', edges='horizontal')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.5)
    
    # Style pour les titres de section
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('lightgray')
        if (row == 0) or (cell_text[row][0] in ["PARAMÈTRES DU MUR", "POIDS DES ANTENNES (module)"]):
            cell.set_text_props(weight='bold', color='black')
            cell.set_facecolor('#e6e6fa')
        elif cell_text[row][0] == "":
            cell.set_edgecolor('white')
            
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        
    plt.show() # Bloquant, ok pour visualiser
    
    return None, None # On ne retourne plus les champs lourds
​
def visualiser_comparaison_detaillee(x_lu, x_pivot, n, A, b, noise_level,
                                      walls_config=None, antenne_positions=None,
                                      directions_principales=None, directions_nulls=None):
    """
    Visualisation comparative SIMPLIFIÉE (1D uniquement) :
    1. Comparaison Rayonnement (Cartésien)
    2. Comparaison Polaire
    3. Écart de Gain
    4. Métriques comparatives
    """
    if antenne_positions is None:
        spacing = 0.5 * LAMBDA
        start_x = -(n-1) * spacing / 2
        antenne_positions = [np.array([start_x + k * spacing, 0.0]) for k in range(n)]
​
    # Calcul des métriques
    m_lu = calculer_metriques(x_lu, A, b, noise_level, walls_config, antenne_positions)
    m_pivot = calculer_metriques(x_pivot, A, b, noise_level, walls_config, antenne_positions)
    
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('COMPARAISON RAPIDE: LU vs PIVOT DE GAUSS', fontsize=16, fontweight='bold', y=0.98)
    
    # 1. Comparaison Cartésienne
    ax1 = fig.add_subplot(2, 2, 1)
    angles = np.linspace(-90, 90, 361)
    af_lu = compute_array_factor(antenne_positions, x_lu, angles)
    af_pivot = compute_array_factor(antenne_positions, x_pivot, angles)
    gain_lu = 20 * np.log10(np.abs(af_lu) + 1e-12)
    gain_pivot = 20 * np.log10(np.abs(af_pivot) + 1e-12)
    
    ax1.plot(angles, gain_lu, 'b-', linewidth=2, label='LU')
    ax1.plot(angles, gain_pivot, 'r--', linewidth=2, label='Pivot')
    
    if directions_principales:
        for angle in directions_principales:
            ax1.axvline(x=angle, color='green', linestyle=':', alpha=0.5)
    if directions_nulls:
        for angle in directions_nulls:
            ax1.axvline(x=angle, color='red', linestyle=':', alpha=0.5)
            
    ax1.set_xlabel('Angle (degrés)')
    ax1.set_ylabel('Gain (dB)')
    ax1.set_title('Comparaison Diagrammes de Rayonnement', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-90, 90)
    
    # 2. Comparaison Polaire
    ax2 = fig.add_subplot(2, 2, 2, projection='polar')
    angles_rad = np.deg2rad(angles)
    # Normalisation par rapport au max global pour comparaison juste
    max_val = max(np.max(np.abs(af_lu)), np.max(np.abs(af_pivot))) + 1e-12
    
    ax2.plot(angles_rad, np.abs(af_lu)/max_val, 'b-', linewidth=2, label='LU')
    ax2.plot(angles_rad, np.abs(af_pivot)/max_val, 'r--', linewidth=2, label='Pivot')
    
    ax2.set_theta_zero_location('N')
    ax2.set_theta_direction(-1)
    ax2.set_title('Comparaison Polaire (Normalisée)', fontsize=12, fontweight='bold', pad=10)
    ax2.legend(loc='lower right', bbox_to_anchor=(1.1, 0))
    
    # 3. Écart de Gain
    ax3 = fig.add_subplot(2, 2, 3)
    diff_af = np.abs(gain_lu - gain_pivot)
    ax3.plot(angles, diff_af, 'g-', linewidth=2)
    ax3.set_xlabel('Angle (degrés)')
    ax3.set_ylabel('Δ Gain (dB)')
    ax3.set_title('Différence absolue de Gain (dB)', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(-90, 90)
    
    # 4. Tableau Métriques
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.axis('off')
    
    col_labels = ['Métrique', 'LU', 'Pivot']
    table_vals = []
    
    for k in m_lu:
        table_vals.append([k, m_lu[k], m_pivot[k]])
        
    # Ajouter comparaison norme des poids
    norm_lu = np.linalg.norm(x_lu)
    norm_pivot = np.linalg.norm(x_pivot)
    table_vals.append(['Norme |x|', f"{norm_lu:.4f}", f"{norm_pivot:.4f}"])
    
    # Ajouter écart max poids
    max_diff_x = np.max(np.abs(x_lu - x_pivot))
    table_vals.append(['Max Diff Sols', '-', f"{max_diff_x:.2e}"])
    
    table = ax4.table(cellText=table_vals, colLabels=col_labels, loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.8)
    ax4.set_title('Métriques Comparatives', fontsize=12, fontweight='bold')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
​
    
    # Comparaison des poids
​
​
# ==================================================
# SOLVEURS
# ==================================================
​
def solve_lu(A, b):
    print("\n🔷 Résolution par LU (scipy.linalg.lu_solve)")
    t0 = time.time()
    lu, piv = lu_factor(A)
    x = lu_solve((lu, piv), b)
    t_lu = time.time() - t0
    print(f"   ⏱️  Temps: {t_lu*1000:.3f} ms")
    return x, t_lu
​
def solve_pivot_gauss(A, b, verbose=False):
    if verbose:
        print("\n🔶 Résolution par Pivot de Gauss (implémentation maison)")
    
    t0 = time.time()
    n = len(b)
    Ab = np.hstack((A.copy().astype(complex), b.reshape(-1, 1).astype(complex)))
    
    scale = np.max(np.abs(Ab[:, :-1]), axis=1)
    scale[scale == 0] = 1
    
    if verbose:
        print(f"   📐 Dimension: {n}x{n}")
        print(f"   🔢 Conditionnement: {np.linalg.cond(A):.2e}")
    
    for k in range(n):
        ratios = np.abs(Ab[k:, k]) / scale[k:]
        max_row = np.argmax(ratios) + k
        
        if max_row != k:
            Ab[[k, max_row]] = Ab[[max_row, k]]
            scale[[k, max_row]] = scale[[max_row, k]]
        
        pivot = Ab[k, k]
        
        if np.abs(pivot) < 1e-14:
            if verbose:
                print(f"   ⚠️  Pivot quasi-nul à l'étape {k+1}")
            if np.abs(pivot) < 1e-16:
                print("   ❌ ERREUR: Matrice singulière")
                return None, 0
        
        if k < n - 1:
            factors = Ab[k+1:, k] / pivot
            Ab[k+1:, k:] -= np.outer(factors, Ab[k, k:])
    
    x = np.zeros(n, dtype=complex)
    for i in range(n-1, -1, -1):
        x[i] = Ab[i, n] - np.dot(Ab[i, i+1:n], x[i+1:n])
        x[i] /= Ab[i, i]
    
    t_pivot = time.time() - t0
    
    if verbose:
        residual = np.linalg.norm(A @ x - b)
        print(f"   ✅ Résidu: {residual:.2e}")
        print(f"   ⏱️  Temps: {t_pivot*1000:.3f} ms")
    
    return x, t_pivot
​
# ==================================================
# AFFICHAGE DES POIDS
# ==================================================
​
def afficher_poids(x, methode):
    print(f"\n📊 Poids calculés avec {methode} :")
    print("-" * 60)
    print(f"{'Antenne':<10} {'Partie réelle':<15} {'Partie imag.':<15} {'Module':<12} {'Phase (°)':<10}")
    print("-" * 60)
    for i, val in enumerate(x, 1):
        real = val.real
        imag = val.imag
        module = np.abs(val)
        phase = np.angle(val, deg=True)
        print(f"x{i:<9} {real:>14.4f}  {imag:>14.4f}  {module:>11.4f}  {phase:>9.2f}")
    print("-" * 60)
​
# ==================================================
# MENU INTERACTIF AMÉLIORÉ
# ==================================================
​
def menu_principal():
    print("\n" + "█" * 70)
    print("█" + "   BEAMFORMING - COMPARAISON LU vs PIVOT DE GAUSS".center(68) + "█")
    print("█" * 70)
    print("\n📋 MENU PRINCIPAL:")
    print("   1 → Mode Personnalisé (saisie manuelle complète)")
    print("   2 → Mode Automatique (génération par objectifs)")
    print("   3 → Exemples pré-configurés")
    print("   4 → Mode Hybride (saisie A/b + géométrie auto)")
    print("   0 → Quitter")
    print("═" * 70)
    return input("   Votre choix : ").strip()
​
def menu_post_calcul(A, b, n, noise_level, walls_config, 
                     antenne_positions, directions_principales, directions_nulls,
                     methode_utilisee, x_result):
    """Menu après calcul avec une méthode - permet de tester l'autre méthode."""
    
    autre_methode = "Pivot de Gauss" if methode_utilisee == "LU" else "LU"
    tested_other_method = False  # Flag pour savoir si l'autre méthode a été testée
    
    while True:
        print("\n" + "═" * 70)
        print(f"   ✅ Calcul effectué avec: {methode_utilisee}")
        if tested_other_method:
             print(f"   ✅ Autre méthode testée: {autre_methode}")
        print("═" * 70)
        print("\n📋 QUE SOUHAITEZ-VOUS FAIRE ?")
        
        if not tested_other_method:
            print(f"   1 → Tester l'autre méthode ({autre_methode})")
        
        print("   2 → Visualiser le résultat courant")
        print("   3 → Modifier le vecteur b")
        print("   4 → Modifier les paramètres (mur, bruit)")
        print("   0 → Retour au menu principal")
        print("═" * 70)
        
        choix = input("   Votre choix : ").strip()
        
        if choix == '1' and not tested_other_method:
            # Tester l'autre méthode
            print(f"\n🔄 Test avec {autre_methode}")
            print("═" * 70)
            print("Voulez-vous:")
            print("   1 → Garder les MÊMES paramètres (A, b, mur, bruit)")
            print("   2 → CHANGER certains paramètres avant le calcul")
            print("═" * 70)
            
            sous_choix = input("   Votre choix : ").strip()
            
            if sous_choix == '2':
                # Permettre de changer des paramètres
                print("\n📋 Paramètres actuels:")
                print(f"   • Bruit: {noise_level}")
                n_walls = sum(1 for w in walls_config if w is not None)
                print(f"   • Murs configurés: {n_walls}")
                
                changer = input("\nChanger le niveau de bruit ? (o/n) : ").lower()
                if changer == 'o':
                    new_noise = float(input("   Nouveau bruit (ex: 0.05) : ") or noise_level)
                    b = b + (new_noise - noise_level) * (np.random.randn(n) + 1j * np.random.randn(n))
                    noise_level = new_noise
                
                changer = input("Reconfigurer les murs ? (o/n) : ").lower()
                if changer == 'o':
                    print("   1 → Mur simple")
                    print("   2 → Murs différents")
                    print("   3 → Aucun mur")
                    wm = input("   Choix : ")
                    if wm == '1':
                        walls_config = configurer_murs_antennes(n, mode='simple')
                    elif wm == '2':
                        walls_config = configurer_murs_antennes(n, mode='complet')
                    else:
                        walls_config = configurer_murs_antennes(n, mode='aucun')
                
                changer = input("Changer le vecteur b ? (o/n) : ").lower()
                if changer == 'o':
                    print(f"   Nouveau b ({n} valeurs):")
                    ex_val = "1+0j"
                    ex_row = "  ".join([ex_val] * min(n, 3)) + (" ..." if n > 3 else "")
                    print(f"   Exemple : {ex_row}")
                    b_str = input("   b = ").strip()
                    try:
                        nouveau_b = np.array([complex(v) for v in b_str.split()], dtype=complex)
                        if len(nouveau_b) == n:
                            b = nouveau_b
                        else:
                            print(f"   ⚠️  Erreur: {len(nouveau_b)} valeurs au lieu de {n}")
                    except Exception as e:
                        print(f"   ⚠️  Erreur: {e}")
            
            # Exécution avec l'autre méthode
            if autre_methode == "LU":
                x_autre, t_autre = solve_lu(A, b)
            else:
                x_autre, t_autre = solve_pivot_gauss(A, b, verbose=True)
            
            if x_autre is not None:
                tested_other_method = True # Marquer comme testé
                afficher_poids(x_autre, autre_methode)
                m = calculer_metriques(x_autre, A, b, noise_level, walls_config, antenne_positions)
                print(f"\n📊 Métriques pour {autre_methode}:")
                for k, v in m.items():
                    print(f"   {k}: {v}")
                
                # Proposer visualisation
                vis = input(f"\nVisualiser le champ ({autre_methode}) ? (o/n) : ").lower()
                if vis == 'o':
                    visualiser_champ_detaille(x_autre, n, walls_config, antenne_positions,
                                              f"Champ calculé", autre_methode,
                                              directions_principales=directions_principales,
                                              directions_nulls=directions_nulls)
​
                # Proposer comparaison
                print("\n" + "-" * 70)
                comparer = input("Comparer les deux méthodes (visuellement) ? (o/n) : ").lower()
                if comparer == 'o':
                    if methode_utilisee == "LU":
                        x_lu, x_pivot = x_result, x_autre
                    else:
                        x_lu, x_pivot = x_autre, x_result
                    
                    visualiser_comparaison_detaillee(x_lu, x_pivot, n, A, b, noise_level,
                                                      walls_config, antenne_positions,
                                                      directions_principales, directions_nulls)
        
        elif choix == '2':
            visualiser_champ_detaille(x_result, n, walls_config, antenne_positions,
                                      f"Champ calculé", methode_utilisee,
                                      directions_principales=directions_principales,
                                      directions_nulls=directions_nulls)
        
        elif choix == '3':
            print(f"\n📝 Nouveau vecteur b ({n} valeurs complexes):")
            print(f"   Vecteur actuel: {' '.join(f'{v:.4f}' for v in b[:min(6, n)])}{' ...' if n > 6 else ''}")
            b_str = input("   Nouveau b = ").strip()
            try:
                nouveau_b = np.array([complex(v) for v in b_str.split()], dtype=complex)
                if len(nouveau_b) == n:
                    b = nouveau_b
                    print("   ✅ Vecteur b mis à jour.")
                    
                    # NOUVEAU: Recalcul automatique avec les DEUX méthodes
                    print("\n🔄 Recalcul automatique avec les deux méthodes...")
                    print("═" * 70)
                    
                    # Calcul avec LU
                    print("\n📊 Calcul avec LU...")
                    x_lu, t_lu = solve_lu(A, b)
                    
                    # Calcul avec Pivot de Gauss
                    print("\n📊 Calcul avec Pivot de Gauss...")
                    x_pivot, t_pivot = solve_pivot_gauss(A, b, verbose=False)
                    
                    if x_lu is not None and x_pivot is not None:
                        # Calcul des métriques pour les deux méthodes
                        m_lu = calculer_metriques(x_lu, A, b, noise_level, walls_config, antenne_positions)
                        m_pivot = calculer_metriques(x_pivot, A, b, noise_level, walls_config, antenne_positions)
                        
                        # Affichage du tableau comparatif
                        print("\n" + "╔" + "═" * 68 + "╗")
                        print("║" + " " * 15 + "COMPARAISON APRÈS MODIFICATION DE b" + " " * 17 + "║")
                        print("╠" + "═" * 68 + "╣")
                        print(f"║ {'Métrique':<30} │ {'LU':<15} │ {'Pivot Gauss':<15} ║")
                        print("╠" + "═" * 68 + "╣")
                        
                        # Afficher chaque métrique
                        for key in m_lu.keys():
                            val_lu = m_lu[key]
                            val_pivot = m_pivot[key]
                            print(f"║ {key:<30} │ {val_lu:<15} │ {val_pivot:<15} ║")
                        
                        print(f"║ {'Temps de calcul':<30} │ {f'{t_lu:.6f} s':<15} │ {f'{t_pivot:.6f} s':<15} ║")
                        print("╚" + "═" * 68 + "╝")
                        
                        # Afficher les poids pour les deux méthodes
                        print("\n📋 Poids calculés:")
                        print("\n   LU:")
                        for i, w in enumerate(x_lu[:min(6, n)]):
                            print(f"      x[{i}] = {w:.6f}")
                        if n > 6:
                            print(f"      ... ({n-6} autres)")
                        
                        print("\n   Pivot de Gauss:")
                        for i, w in enumerate(x_pivot[:min(6, n)]):
                            print(f"      x[{i}] = {w:.6f}")
                        if n > 6:
                            print(f"      ... ({n-6} autres)")
                        
                        # Proposer visualisation
                        print("\n" + "-" * 70)
                        vis = input("Visualiser la comparaison graphique ? (o/n) : ").lower()
                        if vis == 'o':
                            visualiser_comparaison_detaillee(x_lu, x_pivot, n, A, b, noise_level,
                                                              walls_config, antenne_positions,
                                                              directions_principales, directions_nulls)
                        
                        # Mettre à jour x_result avec la méthode courante
                        if methode_utilisee == "LU":
                            x_result = x_lu
                        else:
                            x_result = x_pivot
                    else:
                        print("   ⚠️  Erreur lors du recalcul")
                else:
                    print(f"   ⚠️  Erreur: {len(nouveau_b)} valeurs au lieu de {n}")
            except Exception as e:
                print(f"   ⚠️  Erreur: {e}")
        
        elif choix == '4':
            print("\n📋 Modification des paramètres:")
            n_walls = sum(1 for w in walls_config if w is not None)
            print(f"   • Murs actuels: {n_walls}")
            changer = input("   Reconfigurer les murs ? (o/n) : ").lower()
            if changer == 'o':
                print("   1 → Mur simple")
                print("   2 → Murs différents")
                print("   3 → Aucun mur")
                wm = input("   Choix : ")
                if wm == '1':
                    walls_config = configurer_murs_antennes(n, mode='simple')
                elif wm == '2':
                    walls_config = configurer_murs_antennes(n, mode='complet')
                else:
                    walls_config = configurer_murs_antennes(n, mode='aucun')
            
            new_noise = float(input(f"   Bruit à ajouter (actuel: {noise_level}) : ") or 0)
            if new_noise > 0:
                b += new_noise * (np.random.randn(n) + 1j * np.random.randn(n))
                noise_level = new_noise
            
            print(f"\n🔄 Recalcul automatique avec {methode_utilisee}...")
            
            # Recalcul
            if methode_utilisee == "LU":
                x_new, t_new = solve_lu(A, b)
            else:
                x_new, t_new = solve_pivot_gauss(A, b, verbose=False)
            
            if x_new is not None:
                x_result = x_new
                
                # Affichage résultats
                afficher_poids(x_result, methode_utilisee)
                m = calculer_metriques(x_result, A, b, noise_level, walls_config, antenne_positions)
                print(f"\n📊 Nouvelles métriques:")
                for k, v in m.items():
                    print(f"   {k}: {v}")
                
                # Proposer visualisation
                vis = input(f"\nVisualiser le nouveau champ ? (o/n) : ").lower()
                if vis == 'o':
                    visualiser_champ_detaille(x_result, n, walls_config, antenne_positions,
                                              f"Champ calculé (Paramètres modifiés)", methode_utilisee,
                                              directions_principales=directions_principales,
                                              directions_nulls=directions_nulls)
        
        elif choix == '0':
            return A, b, noise_level, walls_config
        
        else:
            print("   ❌ Choix invalide")
    
    return A, b, noise_level, walls_config
​
​
# ==================================================
# FONCTION PRINCIPALE
# ==================================================
​
​
​
​
def main():
    print("═" * 70)
    print("     SIMULATEUR DE BEAMFORMING INTELLIGENT")
    print("     Techniques: LU vs Pivot de Gauss")
    #print("     Version: 2.1 (Murs par antenne + Visualisation simplifiée)")
    print("═" * 70)
    
    while True:
        print("\n📋 MENU PRINCIPAL")
        print("   1 → Mode Personnalisé (Saisie manuelle complète)")
        print("   2 → Mode Automatique (Génération par objectifs)")
        print("   3 → Exemples pré-configurés")
        print("   4 → Mode Hybride (A/b manuels + Géométrie auto)")
        print("   0 → Quitter")
        
        choix = input("   Votre choix : ").strip()
        
        if choix in ['1', '2', '3', '4']:
            # Initialisation des variables
            A, b, n, noise_level = None, None, 0, 0
            walls_config = None
            antenne_positions = None
            directions_principales = None
            directions_nulls = None
            
            # Saisie selon le mode
            if choix == '1':
                A, b, n, noise_level, walls_config = saisie_personnalisee()
            
            elif choix == '2':
                A, b, n, noise_level, walls_config, geometry, antenne_positions, \
                directions_principales, directions_nulls = saisie_automatique_beamforming()
            
            elif choix == '3':
                A, b, n, noise_level, walls_config, geometry, antenne_positions, \
                directions_principales, directions_nulls = charger_exemple()
            
            elif choix == '4':
                A, b, n, noise_level, walls_config, geometry, antenne_positions = saisie_hybride()
            
            # Si pas de positions définies (mode 1), on met une géométrie par défaut (linéaire)
            if antenne_positions is None:
                spacing = 0.5 * LAMBDA
                start_x = -(n-1) * spacing / 2
                antenne_positions = [np.array([start_x + k * spacing, 0.0]) for k in range(n)]
            
            # Choix de la méthode
            print("\n" + "═" * 70)
            print("   Choisissez la méthode de résolution:")
            print("   1 → LU (décomposition LU avec scipy)")
            print("   2 → Pivot de Gauss (implémentation maison)")
            print("═" * 70)
            methode_choix = input("   Votre choix : ").strip()
            
            x = None
            methode_nom = ""
            
            if methode_choix == '1':
                methode_nom = "LU"
                x, t = solve_lu(A, b)
            else:
                methode_nom = "Pivot de Gauss"
                x, t = solve_pivot_gauss(A, b, verbose=True)
            
            if x is not None:
                afficher_poids(x, methode_nom)
                m = calculer_metriques(x, A, b, noise_level, walls_config, antenne_positions)
                print(f"\n📊 Métriques ({methode_nom}):")
                for k, v in m.items():
                    print(f"   {k}: {v}")
                
                # Visualisation initiale (si n pas trop grand ou si utilisateur veut)
                if not is_large_system(n) or input("\nAfficher les graphiques ? (o/n) : ").lower() == 'o':
                    visualiser_champ_detaille(x, n, walls_config, antenne_positions,
                                              f"Résultat", methode_nom,
                                              directions_principales=directions_principales,
                                              directions_nulls=directions_nulls)
                
                # Menu post-calcul
                A, b, noise_level, walls_config = menu_post_calcul(
                    A, b, n, noise_level, walls_config,
                    antenne_positions, directions_principales, directions_nulls,
                    methode_nom, x
                )
        
        elif choix == '0':
            print("\n👋 Au revoir!")
            break
​
if __name__ == "__main__":
    main()