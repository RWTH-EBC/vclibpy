import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import pandas as pd
import numpy as np

# =============================================================================
# 1. KONFIGURATION
# =============================================================================

FIXED_MASS_FLOW_KOND = 0.342  # kg/s
EXPORT_FILENAME = 'betriebspunkte_final_weniger.xlsx'

# Transformation
OFFSET_AIR = 8.0
PINCH_COND = 3.0
ESTIMATED_SPREAD = 5.0

# =============================================================================
# 2. DEFINITION GEOMETRIE
# =============================================================================

# Original Eckpunkte (Sättigung)
corners_sat = [(-35, 65), (-10, 80), (15, 80), (25, 70), (25, 25), (10, 10), (-35, 65)]
# Umrechnung in Real (T_amb, T_vl)
corners_real = [(x + OFFSET_AIR, y - PINCH_COND) for x, y in corners_sat]
poly_path = mplPath.Path(corners_real)

# =============================================================================
# 3. GENERIERUNG (RASTER-METHODE)
# =============================================================================

candidates = []

# A. Das GROBE RASTER (weniger Punkte)
# Luft: Start bei -7, Schritte von 8 Grad (statt 4)
amb_range = np.arange(-7, 40, 8)
# Wasser: Start bei 20, Schritte von 10 Grad (statt 6)
vl_range = np.arange(20, 85, 10)

for t_a in amb_range:
    for t_v in vl_range:
        candidates.append({'ta': t_a, 'tv': t_v, 'type': 'Standard', 'info': 'Raster'})

# B. Die ECKPUNKTE (Wichtig für die Grenzen)
# Schnittpunkte bei -7°C
candidates.append({'ta': -7, 'tv': 73, 'type': 'Grenze', 'info': 'Ecke Oben-Links'})
candidates.append({'ta': -7, 'tv': 38, 'type': 'Grenze', 'info': 'Ecke Unten-Links'})

# Echte Envelope Ecken (sofern > -7°C)
real_corners = [
    {'ta': -2, 'tv': 77, 'type': 'Grenze', 'info': 'Peak Oben'},
    {'ta': 23, 'tv': 77, 'type': 'Grenze', 'info': 'Ecke Oben-Rechts'},
    {'ta': 33, 'tv': 67, 'type': 'Grenze', 'info': 'Ecke Rechts'},
    {'ta': 33, 'tv': 22, 'type': 'Grenze', 'info': 'Ecke Rechts-Unten'},
]
candidates.extend(real_corners)

# =============================================================================
# 4. FILTERUNG
# =============================================================================

rows_list = []

for p in candidates:
    ta = float(p['ta'])
    tv = float(p['tv'])

    # FILTER 1: Muss im Verdichter-Envelope liegen
    is_inside = poly_path.contains_point((ta, tv)) or p['type'] == 'Grenze'

    if is_inside:
        # FILTER 2: Muss >= -7°C sein
        if ta >= -7.0:

            # FILTER 3: Vorlauf muss deutlich GRÖSSER als Luft sein (mind 3K)
            if tv > (ta + 3.0):
                # Rückrechnung Sättigung
                tsat_ev = ta - OFFSET_AIR
                tsat_co = tv + PINCH_COND
                t_ruecklauf = tv - ESTIMATED_SPREAD

                row = {
                    'T_eva_in': ta,
                    'phi': 0.8,
                    'm_flow_eva': 0.6,
                    'T_kond_in': t_ruecklauf,
                    'm_flow_kond': FIXED_MASS_FLOW_KOND,
                    'T_ambient': ta,
                    'type': p['type'],
                    'info_ext': p['info'],
                    'T_vorlauf': tv,
                    'ev': tsat_ev,
                    'co': tsat_co
                }
                rows_list.append(row)

df = pd.DataFrame(rows_list)
# Duplikate entfernen (falls Raster genau auf Ecke trifft)
df = df.drop_duplicates(subset=['T_ambient', 'T_vorlauf'])
df = df.sort_values(by=['T_ambient', 'T_vorlauf']).reset_index(drop=True)
df.index.name = 'ID'

cols = ['T_eva_in', 'phi', 'm_flow_eva', 'T_kond_in', 'm_flow_kond', 'T_ambient', 'type', 'info_ext', 'T_vorlauf']
df[cols].to_excel(EXPORT_FILENAME)
print(f"Datei '{EXPORT_FILENAME}' erstellt. ({len(df)} Punkte)")

# =============================================================================
# 5. PLOT
# =============================================================================
plt.figure(figsize=(10, 8))

# Envelope
px, py = zip(*corners_real)
plt.fill(px, py, 'lightgray', alpha=0.3, label='Verdichter Envelope')
plt.plot(px, py, 'gray', linestyle='--')

# Grenzen
plt.axvline(x=-7, color='red', linewidth=2, label='Grenze -7°C')
# Diagonale T_vl = T_amb
x_d = np.linspace(-10, 45, 10)
plt.plot(x_d, x_d, 'k-', linewidth=2, label='Physik-Grenze (T_vl > T_amb)')

# Punkte Plotten
# Normale Raster-Punkte
mask_std = df['type'] == 'Standard'
plt.scatter(df[mask_std]['T_ambient'], df[mask_std]['T_vorlauf'],
            c='dodgerblue', s=60, edgecolors='white', label='Raster-Punkte', zorder=5)

# Grenz-Punkte
mask_border = df['type'] == 'Grenze'
plt.scatter(df[mask_border]['T_ambient'], df[mask_border]['T_vorlauf'],
            c='orange', s=100, marker='D', edgecolors='black', label='Eckpunkte', zorder=10)

plt.title(f'Reduzierte Simulations-Matrix\nFilter: Innerhalb Envelope | >= -7°C | T_vl > T_amb')
plt.xlabel('Außenlufttemperatur [°C]')
plt.ylabel('Heizwasser Vorlauftemperatur [°C]')
plt.legend(loc='upper left')
plt.grid(True, linestyle=':', alpha=0.5)

plt.xlim(-15, 40)
plt.ylim(0, 90)
plt.tight_layout()
plt.show()