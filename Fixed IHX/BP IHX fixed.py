import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import pandas as pd
import numpy as np

# =============================================================================
# 1. KONFIGURATION
# =============================================================================

FIXED_MASS_FLOW_KOND = 0.342  # kg/s
EXPORT_FILENAME = 'betriebspunkte_final_bivalent_seriell.xlsx'

# Transformation (Sat -> Real)
# T_amb = T_sat_evap + 8 | T_vl = T_sat_cond - 3
OFFSET_AIR = 8.0
PINCH_COND = 3.0
ESTIMATED_SPREAD = 5.0

# =============================================================================
# 2. DEFINITION GEOMETRIE (ENVELOPE)
# =============================================================================

# Original Eckpunkte des Verdichters (Sättigungsebene)
corners_sat = [(-35, 65), (-10, 80), (15, 80), (25, 70), (25, 25), (10, 10), (-35, 45)]

# Umrechnung in Real-Koordinaten (Luft / Wasser) für den Check
# Damit haben wir die echten physikalischen Grenzen
corners_real = [(x + OFFSET_AIR, y - PINCH_COND) for x, y in corners_sat]
poly_path = mplPath.Path(corners_real)

# =============================================================================
# 3. GENERIERUNG (RASTER-METHODE)
# =============================================================================

candidates = []

# A. Das RASTER (Erweitert nach links!)
# Luft: Start jetzt bei -30°C (statt -7), damit wir den linken Bereich füllen
amb_range = np.arange(-30, 40, 8)
# Wasser: Start bei 15, Schritte von 8 (etwas feiner für gute Abdeckung)
vl_range = np.arange(15, 85, 8)

for t_a in amb_range:
    for t_v in vl_range:
        candidates.append({'ta': t_a, 'tv': t_v, 'type': 'Standard', 'info': 'Raster'})

# B. Die ECKPUNKTE (Die echten physikalischen Ecken)
# Wir nehmen die oben berechneten Real-Corners und fügen sie als Punkte hinzu.
# Das garantiert, dass die Simulation die absoluten Limits testet.
for (cx, cy) in corners_real:
    candidates.append({'ta': cx, 'tv': cy, 'type': 'Grenze', 'info': 'Verdichter-Limit'})

# C. DESIGN PUNKTE BEI -7°C (Nur zur Info / Referenz)
# Wir fügen explizit Punkte auf der Bivalenzlinie hinzu, weil die wichtig sind.
#candidates.append({'ta': -7, 'tv': 55, 'type': 'Design', 'info': 'Bivalenz-Punkt Hoch'})
#candidates.append({'ta': -7, 'tv': 35, 'type': 'Design', 'info': 'Bivalenz-Punkt Tief'})

# =============================================================================
# 4. FILTERUNG
# =============================================================================

rows_list = []

for p in candidates:
    ta = float(p['ta'])
    tv = float(p['tv'])

    # FILTER 1: Muss im Verdichter-Envelope liegen
    # (Wir akzeptieren Punkte auf der Grenze oder knapp drinnen)
    is_inside = poly_path.contains_point((ta, tv), radius=0.1) or p['type'] == 'Grenze'

    if is_inside:
        # FILTER 2: Vorlauf muss deutlich GRÖSSER als Luft sein (mind 3K)
        # Physikalische Notwendigkeit, sonst keine Wärmeabgabe
        if tv > (ta + 3.0):

            # Rückrechnung Sättigung (für Simulation Input)
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
# Sortieren: Erst nach Ambient (kalt nach warm), dann nach Vorlauf
df = df.sort_values(by=['T_ambient', 'T_vorlauf']).reset_index(drop=True)
df.index.name = 'ID'

cols = ['T_eva_in', 'phi', 'm_flow_eva', 'T_kond_in', 'm_flow_kond', 'T_ambient', 'type', 'info_ext', 'T_vorlauf']
df[cols].to_excel(EXPORT_FILENAME)
print(f"Datei '{EXPORT_FILENAME}' erstellt. ({len(df)} Punkte)")

# =============================================================================
# 5. PLOT
# =============================================================================
plt.figure(figsize=(10, 8))

# Envelope (Grau hinterlegt)
px, py = zip(*corners_real)
# Polygon schließen für fill
px_poly = px + (px[0],)
py_poly = py + (py[0],)
plt.fill(px_poly, py_poly, 'lightgray', alpha=0.3, label='Verdichter Envelope')
plt.plot(px_poly, py_poly, 'gray', linestyle='--')

# Die Bivalenz-Linie (Nur Visuell!)
plt.axvline(x=-7, color='red', linewidth=2, linestyle=':', label='Bivalenzpunkt (-7°C)')

# Diagonale T_vl = T_amb
x_d = np.linspace(-30, 45, 10)
plt.plot(x_d, x_d, 'k-', linewidth=1, label='Grenze T_vl > T_amb')

# Punkte Plotten
# 1. Normale Raster-Punkte
mask_std = df['type'] == 'Standard'
plt.scatter(df[mask_std]['T_ambient'], df[mask_std]['T_vorlauf'],
            c='dodgerblue', s=60, edgecolors='white', label='Betriebspunkte', zorder=5)

# 2. Grenz-Punkte (Echte Ecken)
mask_border = df['type'] == 'Grenze'
plt.scatter(df[mask_border]['T_ambient'], df[mask_border]['T_vorlauf'],
            c='orange', s=80, marker='D', edgecolors='black', label='Envelope-Ecken', zorder=6)

# 3. Design-Punkte (-7 Grad Referenz)
#mask_design = df['type'] == 'Design'
#plt.scatter(df[mask_design]['T_ambient'], df[mask_design]['T_vorlauf'],
            #c='red', s=100, marker='X', edgecolors='black', label='Design Referenz', zorder=7)

plt.title(f'Simulations-Matrix (Bivalent Seriell)\nVolle Abdeckung Envelope | m_dot={FIXED_MASS_FLOW_KOND} kg/s')
plt.xlabel('Außenlufttemperatur [°C]')
plt.ylabel('Heizwasser Vorlauftemperatur [°C]')
plt.legend(loc='upper left')
plt.grid(True, linestyle=':', alpha=0.5)

plt.xlim(-30, 40) # Bereich erweitert für tiefe Temperaturen
plt.ylim(0, 90)
plt.tight_layout()
plt.show()