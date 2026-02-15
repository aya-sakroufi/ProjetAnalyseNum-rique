import sys
import numpy as np
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTabWidget, QLabel, QLineEdit, QPushButton, QComboBox, 
                             QFormLayout, QGroupBox, QSpinBox, QDoubleSpinBox, QTableWidget, 
                             QTableWidgetItem, QSplitter, QMessageBox, QScrollArea, QFrame, QCheckBox)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPalette, QColor
import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

# Import logic from version_valide
# Ensure version_valide.py is in the same directory
import version_valide as bv

# Modern Color Palette
COLORS = {
    'primary': '#2c3e50',
    'secondary': '#34495e',
    'accent': '#3498db',
    'success': '#27ae60',
    'danger': '#e74c3c',
    'text': '#2c3e50',
    'background': '#ecf0f1',
    'panel': '#ffffff'
}

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.subplots() # Default single plot
        super(MplCanvas, self).__init__(self.fig)

class BeamformingApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulateur de Beamforming Intelligent - GUI")
        self.setGeometry(100, 100, 1400, 900)
        self.apply_styles()
        
        # Main Layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Tabs
        self.tabs = QTabWidget()
        self.main_layout.addWidget(self.tabs)
        
        # --- Tab 1: Simulation (Auto/Hybrid) ---
        self.tab_simulation = QWidget()
        self.setup_simulation_tab()
        self.tabs.addTab(self.tab_simulation, "📡 Simulation & Analyse")
        
        # --- Tab 2: Comparaison (LU vs Gauss) ---
        self.tab_comparison = QWidget()
        self.setup_comparison_tab()
        self.tabs.addTab(self.tab_comparison, "⚔️ Comparaison (LU vs Gauss)")
        
        # Status Bar
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Prêt")

    def apply_styles(self):
        # Global Stylesheet
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {COLORS['background']}; }}
            QTabWidget::pane {{ border: 1px solid #bdc3c7; background: white; }}
            QTabBar::tab {{ background: #bdc3c7; padding: 10px; margin: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }}
            QTabBar::tab:selected {{ background: {COLORS['accent']}; color: white; font-weight: bold; }}
            QGroupBox {{ font-weight: bold; border: 1px solid #bdc3c7; margin-top: 10px; border-radius: 5px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 3px; background-color: {COLORS['background']}; }}
            QPushButton {{ background-color: {COLORS['primary']}; color: white; border-radius: 4px; padding: 8px; font-weight: bold; }}
            QPushButton:hover {{ background-color: {COLORS['accent']}; }}
            QLabel {{ font-size: 12px; color: {COLORS['text']}; }}
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {{ padding: 6px; border: 1px solid #bdc3c7; border-radius: 3px; background-color: white; }}
            QScrollArea {{ border: none; }}
        """)

    def setup_simulation_tab(self):
        layout = QHBoxLayout(self.tab_simulation)
        
        # --- LEFT PANEL: CONTROLS ---
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMaximumWidth(450)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        content_widget = QWidget()
        self.control_layout = QVBoxLayout(content_widget)
        self.control_layout.setSpacing(15)
        
        # Title
        lbl_title = QLabel("CONFIGURATION")
        lbl_title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {COLORS['primary']}; margin-bottom: 10px;")
        self.control_layout.addWidget(lbl_title)
        
        # 1. Antenna Configuration
        group_antenna = QGroupBox("Configuration Antennes")
        form_antenna = QFormLayout()
        
        self.spin_n = QSpinBox()
        self.spin_n.setRange(2, 100)
        self.spin_n.setValue(8)
        form_antenna.addRow("Nombre d'antennes:", self.spin_n)
        
        self.combo_geo = QComboBox()
        self.combo_geo.addItems(["Linéaire (ULA)", "Circulaire (UCA)", "Aléatoire"])
        form_antenna.addRow("Géométrie:", self.combo_geo)
        
        self.spin_spacing = QDoubleSpinBox()
        self.spin_spacing.setRange(0.1, 5.0)
        self.spin_spacing.setSingleStep(0.1)
        self.spin_spacing.setValue(0.5)
        form_antenna.addRow("Espacement (λ) / Rayon (m):", self.spin_spacing)
        
        group_antenna.setLayout(form_antenna)
        self.control_layout.addWidget(group_antenna)
        
        # 2. Beamforming Goals
        group_goals = QGroupBox("Objectifs de Beamforming")
        form_goals = QFormLayout()
        
        self.input_lobes = QLineEdit("30")
        self.input_lobes.setPlaceholderText("Ex: 0, 30")
        form_goals.addRow("Lobes principaux (°):", self.input_lobes)
        
        self.input_nulls = QLineEdit("-30, 60")
        self.input_nulls.setPlaceholderText("Ex: -30, 60")
        form_goals.addRow("Nulls / Interférences (°):", self.input_nulls)
        
        self.spin_null_weight = QDoubleSpinBox()
        self.spin_null_weight.setRange(1, 1000)
        self.spin_null_weight.setValue(100)
        form_goals.addRow("Poids des Nulls:", self.spin_null_weight)
        
        group_goals.setLayout(form_goals)
        self.control_layout.addWidget(group_goals)
        
        # 3. Environment (Walls & Noise)
        group_env = QGroupBox("Environnement")
        form_env = QFormLayout()
        
        self.check_wall = QCheckBox("Activer Mur")
        self.check_wall.setChecked(True)
        form_env.addRow(self.check_wall)
        
        self.combo_wall_mat = QComboBox()
        self.combo_wall_mat.addItems(["Béton", "Brique", "Bois", "Verre", "Métal"])
        form_env.addRow("Matériau:", self.combo_wall_mat)
        
        self.spin_wall_dist = QDoubleSpinBox()
        self.spin_wall_dist.setValue(2.0)
        form_env.addRow("Distance Mur (m):", self.spin_wall_dist)
        
        self.spin_noise = QDoubleSpinBox()
        self.spin_noise.setRange(0.0, 1.0)
        self.spin_noise.setSingleStep(0.01)
        self.spin_noise.setValue(0.05)
        form_env.addRow("Bruit (niveau):", self.spin_noise)
        
        group_env.setLayout(form_env)
        self.control_layout.addWidget(group_env)
        
        # 4. Method Choice
        group_method = QGroupBox("Résolution & Exécution")
        vbox_method = QVBoxLayout()
        self.combo_method = QComboBox()
        self.combo_method.addItems(["LU (Scipy)", "Pivot de Gauss (Maison)"])
        vbox_method.addWidget(self.combo_method)
        
        self.btn_simulate = QPushButton("🚀 LANCER SIMULATION")
        self.btn_simulate.setFixedHeight(50)
        self.btn_simulate.setStyleSheet(f"background-color: {COLORS['success']}; color: white; font-size: 14px; border-radius: 5px;")
        self.btn_simulate.clicked.connect(self.run_simulation)
        vbox_method.addWidget(self.btn_simulate)
        
        group_method.setLayout(vbox_method)
        self.control_layout.addWidget(group_method)
        
        self.control_layout.addStretch()
        scroll.setWidget(content_widget)
        layout.addWidget(scroll)
        
        # --- RIGHT PANEL: VISUALIZATION ---
        self.viz_container = QWidget()
        self.viz_container.setStyleSheet("background-color: white; border-radius: 5px;")
        self.viz_layout = QVBoxLayout(self.viz_container)
        
        # Matplotlib Canvas
        self.canvas_sim = MplCanvas(self, width=10, height=8, dpi=100)
        self.toolbar_sim = NavigationToolbar(self.canvas_sim, self)
        
        self.viz_layout.addWidget(self.toolbar_sim)
        self.viz_layout.addWidget(self.canvas_sim)
        
        # Metrics Label
        self.lbl_metrics = QLabel("Résultats: Cliquez sur 'Lancer Simulation' pour voir les résultats.")
        self.lbl_metrics.setStyleSheet(f"background-color: #f8f9fa; border: 1px solid #dee2e6; padding: 15px; border-radius: 5px; font-family: monospace;")
        self.lbl_metrics.setFixedHeight(120)
        self.lbl_metrics.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.lbl_metrics.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.viz_layout.addWidget(self.lbl_metrics)
        
        layout.addWidget(self.viz_container)

    def setup_comparison_tab(self):
        layout = QVBoxLayout(self.tab_comparison)
        
        top_bar = QHBoxLayout()
        btn_compare = QPushButton("📊 Comparer LU vs Gauss avec les paramètres actuels")
        btn_compare.setFixedWidth(400)
        btn_compare.clicked.connect(self.run_comparison)
        top_bar.addWidget(btn_compare)
        top_bar.addStretch()
        
        layout.addLayout(top_bar)
        
        self.canvas_comp = MplCanvas(self, width=10, height=8, dpi=100)
        self.toolbar_comp = NavigationToolbar(self.canvas_comp, self)
        layout.addWidget(self.toolbar_comp)
        layout.addWidget(self.canvas_comp)

    def get_common_params(self):
        n = self.spin_n.value()
        geo_map = {0: 'linear', 1: 'circular', 2: 'random'}
        geometry = geo_map[self.combo_geo.currentIndex()]
        spacing = self.spin_spacing.value()
        radius = spacing # Reusing input for radius if circular/random
        
        lobes_str = self.input_lobes.text()
        try:
            lobes = [float(x) for x in lobes_str.split(',')] if lobes_str.strip() else [30]
        except ValueError:
            QMessageBox.warning(self, "Attention", "Format des lobes invalide. Utilisation de [30°].")
            lobes = [30]
        
        nulls_str = self.input_nulls.text()
        try:
            nulls = [float(x) for x in nulls_str.split(',')] if nulls_str.strip() else []
        except ValueError:
            nulls = []
        
        # Wall config
        walls_config = [None] * n
        if self.check_wall.isChecked():
            # Get material key from index
            mat_idx = self.combo_wall_mat.currentIndex()
            # Assuming WALL_MATERIALS keys are ordered: concrete, brick, wood, glass, metal
            keys = list(bv.WALL_MATERIALS.keys())
            mat = keys[mat_idx] if mat_idx < len(keys) else 'concrete'
            
            dist = self.spin_wall_dist.value()
            # Simple wall mode for GUI start - same wall for all
            wall = {'x': dist, 'material': mat, 'thickness': 0.2}
            walls_config = [wall for _ in range(n)]
            
        noise = self.spin_noise.value()
        
        return n, geometry, radius, spacing, lobes, nulls, walls_config, noise

    def run_simulation(self):
        try:
            self.status_bar.showMessage("Calcul en cours...")
            QApplication.processEvents()
            
            n, geometry, radius, spacing, lobes, nulls, walls_config, noise = self.get_common_params()
            
            # Generate System
            # Note: generer_systeme_beamforming expected kwargs/args order might need check
            # def generer_systeme_beamforming(n, geometry='linear', radius=2.0, spacing=0.5, ...
            
            A, b, antenne_positions = bv.generer_systeme_beamforming(
                n, geometry, radius, spacing, lobes, nulls, self.spin_null_weight.value()
            )
            
            # Add Noise
            b += noise * (np.random.randn(n) + 1j * np.random.randn(n))
            
            # Solve
            method_idx = self.combo_method.currentIndex()
            t_start = time.time()
            if method_idx == 0: # LU
                x, t = bv.solve_lu(A, b)
                method_name = "LU"
            else:
                x, t = bv.solve_pivot_gauss(A, b)
                method_name = "Pivot Gauss"
            
            if x is None:
                QMessageBox.critical(self, "Erreur", "La résolution a échoué (Matrice singulière ?)")
                self.status_bar.showMessage("Erreur de calcul")
                return

            # Metrics
            metrics = bv.calculer_metriques(x, A, b, noise, walls_config, antenne_positions)
            
            metrics_html = f"<b>Résultats ({method_name})</b> &nbsp;&nbsp; ⏱️ {t*1000:.2f} ms<br>"
            metrics_html += "<table style='margin-top:5px;'>"
            for k, v in metrics.items():
                metrics_html += f"<tr><td style='padding-right:15px; color:#555;'>{k}</td><td><b>{v}</b></td></tr>"
            metrics_html += "</table>"
            
            self.lbl_metrics.setText(metrics_html)
            
            # Plotting
            self.plot_simulation(x, n, walls_config, antenne_positions, lobes, nulls, method_name)
            self.status_bar.showMessage("Simulation terminée avec succès")
            
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Une erreur est survenue:\n{str(e)}")
            import traceback
            traceback.print_exc()
            self.status_bar.showMessage("Erreur")

    def plot_simulation(self, x, n, walls_config, antenne_positions, lobes, nulls, method_name):
        self.canvas_sim.fig.clear()
        
        # 2x2 Grid
        gs = self.canvas_sim.fig.add_gridspec(2, 2, height_ratios=[1, 1])
        
        # 1. Geometry (Top Left)
        ax_geo = self.canvas_sim.fig.add_subplot(gs[0, 0])
        poss = np.array(antenne_positions)
        ax_geo.scatter(poss[:,0], poss[:,1], c='red', marker='^', s=50, label='Antennes', zorder=3)
        
        # Draw wall line if exists
        if walls_config[0]:
            wall_x = walls_config[0]['x']
            ax_geo.axvline(x=wall_x, color='grey', linestyle='--', linewidth=2, label=f"Mur ({walls_config[0]['material']})")
            # Fill area behind wall to visualize it
            ax_geo.axvspan(wall_x, wall_x+0.5, color='grey', alpha=0.2)
            
        ax_geo.set_title("Disposition des Antennes")
        ax_geo.set_xlabel("X (m)")
        ax_geo.set_ylabel("Y (m)")
        ax_geo.grid(True, alpha=0.3)
        ax_geo.legend(loc='best')
        ax_geo.axis('equal')
        
        # 2. Polar Plot (Top Right)
        ax_pol = self.canvas_sim.fig.add_subplot(gs[0, 1], projection='polar')
        angles = np.linspace(-90, 90, 361)
        af = bv.compute_array_factor(antenne_positions, x, angles)
        angles_rad = np.deg2rad(angles)
        norm_af = np.abs(af)/np.max(np.abs(af))
        ax_pol.plot(angles_rad, norm_af, 'b-', linewidth=2)
        ax_pol.fill(angles_rad, norm_af, 'b', alpha=0.1)
        ax_pol.set_theta_zero_location('N')
        ax_pol.set_theta_direction(-1)
        ax_pol.set_title("Diagramme Polaire (Normalisé)")
        ax_pol.set_xticks(np.deg2rad([-90, -60, -30, 0, 30, 60, 90]))
        ax_pol.set_thetamin(-90)
        ax_pol.set_thetamax(90)
        
        # 3. Cartesian Gain (Bottom Full Width)
        ax_cart = self.canvas_sim.fig.add_subplot(gs[1, :])
        gain_db = 20 * np.log10(np.abs(af) + 1e-12)
        ax_cart.plot(angles, gain_db, 'b-', linewidth=2, label='Gain (dB)')
        
        # Markers for lobes/nulls
        for l in lobes:
            ax_cart.axvline(x=l, color='green', linestyle='--', alpha=0.8, linewidth=1.5)
            ax_cart.text(l, max(gain_db), f"Lobe {l}°", color='green', ha='center', va='bottom', fontsize=8)
        for nu in nulls:
            ax_cart.axvline(x=nu, color='red', linestyle=':', alpha=0.8, linewidth=1.5)
            ax_cart.text(nu, min(gain_db)+10, f"Null {nu}°", color='red', ha='center', va='top', fontsize=8)
            
        ax_cart.set_title("Diagramme de Rayonnement (Cartésien)")
        ax_cart.set_xlabel("Angle (°)")
        ax_cart.set_ylabel("Gain (dB)")
        ax_cart.grid(True, alpha=0.3)
        ax_cart.set_xlim(-90, 90)
        ax_cart.set_ylim(bottom=max(np.min(gain_db), -60))
        
        self.canvas_sim.fig.tight_layout()
        self.canvas_sim.draw()

    def run_comparison(self):
        try:
            self.status_bar.showMessage("Calcul de comparaison...")
            n, geometry, radius, spacing, lobes, nulls, walls_config, noise = self.get_common_params()
            
            A, b, antenne_positions = bv.generer_systeme_beamforming(
                n, geometry, radius, spacing, lobes, nulls, self.spin_null_weight.value()
            )
            b += noise * (np.random.randn(n) + 1j * np.random.randn(n))
            
            # Solve both
            t0 = time.time()
            x_lu, t_lu = bv.solve_lu(A, b)
            
            t1 = time.time()
            x_pivot, t_pivot = bv.solve_pivot_gauss(A, b)
            
            # Metrics
            m_lu = bv.calculer_metriques(x_lu, A, b, noise, walls_config, antenne_positions)
            m_pivot = bv.calculer_metriques(x_pivot, A, b, noise, walls_config, antenne_positions)
            
            # Plot Comparison
            self.canvas_comp.fig.clear()
            gs = self.canvas_comp.fig.add_gridspec(2, 2)
            
            # 1. Cartesian Comparison
            ax1 = self.canvas_comp.fig.add_subplot(gs[0, :])
            angles = np.linspace(-90, 90, 361)
            af_lu = bv.compute_array_factor(antenne_positions, x_lu, angles)
            af_pivot = bv.compute_array_factor(antenne_positions, x_pivot, angles)
            
            gain_lu = 20*np.log10(np.abs(af_lu)+1e-12)
            gain_pivot = 20*np.log10(np.abs(af_pivot)+1e-12)
            
            ax1.plot(angles, gain_lu, 'b-', linewidth=2, label=f'LU (Scipy)')
            ax1.plot(angles, gain_pivot, 'r--', linewidth=2, label=f'Pivot Gauss')
            ax1.set_title("Comparaison des Diagrammes de Rayonnement")
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            ax1.set_xlim(-90, 90)
            ax1.set_ylabel("Gain (dB)")
            
            # 2. Difference
            ax2 = self.canvas_comp.fig.add_subplot(gs[1, 0])
            diff = np.abs(gain_lu - gain_pivot)
            ax2.plot(angles, diff, 'g-', linewidth=1.5)
            ax2.set_title("Différence Absolue de Gain (dB)")
            ax2.set_xlim(-90, 90)
            ax2.set_xlabel("Angle (°)")
            ax2.grid(True, alpha=0.3)
            
            # 3. Metrics Table
            ax3 = self.canvas_comp.fig.add_subplot(gs[1, 1])
            ax3.axis('off')
            ax3.set_title("Comparaison Métriques")
            
            keys = ["Erreur résiduelle ||Ax-b||", "Puissance émise totale ||x||²", "Gain maximal (est.)", "SNR champ (dB, est.)"]
            # Filter keys that exist
            keys = [k for k in keys if k in m_lu]
            
            cell_text = []
            for k in keys:
                cell_text.append([k, m_lu[k], m_pivot[k]])
            
            # Add times
            cell_text.append(["Temps de calcul", f"{t_lu*1000:.3f} ms", f"{t_pivot*1000:.3f} ms"])
            
            table = ax3.table(cellText=cell_text, colLabels=["Métrique", "LU", "Pivot"], loc='center', cellLoc='center')
            table.auto_set_font_size(False)
            table.set_fontsize(9)
            table.scale(1, 1.8)
            
            self.canvas_comp.fig.tight_layout()
            self.canvas_comp.draw()
            self.status_bar.showMessage("Comparaison terminée")
            
        except Exception as e:
             QMessageBox.critical(self, "Erreur", f"Erreur de comparaison:\n{str(e)}")

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # Enable High DPI display
    if hasattr(Qt.ApplicationAttribute, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.ApplicationAttribute.AA_EnableHighDpiScaling, True)
    if hasattr(Qt.ApplicationAttribute, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.ApplicationAttribute.AA_UseHighDpiPixmaps, True)

    window = BeamformingApp()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
