import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# --- 1. KONFIGURATION GEBÄUDE & PHYSIK ---

# Parameter für die Massenstrom-Berechnung
HOUSE_AREA = 150.0  # m²
SPECIFIC_LOAD = 70.0  # W/m²
T_DESIGN_OUT = -15.0  # °C
T_HEIZGRENZE = 15.0  # °C
CP_WATER = 4180.0  # J/(kg*K)
TARGET_SPREAD = 7.0  # K (Ziel-Spreizung)

# Pinch-Point (Verlust im Kondensator)
# T_Vorlauf ist immer etwas kälter als die Kondensationstemperatur
PINCH_COND = 3.0  # K

# Offset Luft (Verdampfer)
# T_Ambient ist wärmer als T_Verdampfung
OFFSET_AIR = 8.0  # K

MAX_HEATING_LOAD_KW = (HOUSE_AREA * SPECIFIC_LOAD) / 1000.0

# Dateiname
EXPORT_FILENAME = 'betriebspunkte.xlsx'


# --- Berechnungs-Funktionen ---

def calculate_variable_mass_flow(t_ambient):
    """Berechnet Massenstrom basierend auf Wärmebedarf und Zielspreizung."""
    if t_ambient < T_HEIZGRENZE:
        load_factor = (T_HEIZGRENZE - t_ambient) / (T_HEIZGRENZE - T_DESIGN_OUT)
        load_factor = min(load_factor, 1.2)
        current_load_kw = MAX_HEATING_LOAD_KW * load_factor
    else:
        current_load_kw = MAX_HEATING_LOAD_KW * 0.2  # Warmwasser Sommer

    current_load_kw = max(current_load_kw, 2.0)  # Minimum

    q_watts = current_load_kw * 1000.0
    m_flow = q_watts / (CP_WATER * TARGET_SPREAD)
    return round(m_flow, 3)


def map_to_temperatures(row):
    """
    Wandelt Kennfeld-Koordinaten (Sättigung) in System-Temperaturen um.
    """
    evap_sat = row['ev']
    cond_sat = row['co']

    # 1. X-Achse: Außenluft
    t_ambient = evap_sat + OFFSET_AIR

    # 2. Y-Achse: VORLAUF-Temperatur (Das Ziel für das Haus)
    # Wir nehmen an, der Vorlauf ist um den Pinch kühler als die Kondensation
    t_vorlauf = cond_sat - PINCH_COND

    # 3. Simulations-Input: RÜCKLAUF-Temperatur
    # Der Rücklauf ist um die Spreizung kühler als der Vorlauf
    t_ruecklauf = t_vorlauf - TARGET_SPREAD

    return pd.Series([t_ambient, t_vorlauf, t_ruecklauf])


# --- 2. DEFINITION DER PUNKTE (Unverändert) ---

skeleton_points = [
    # OBEN
    {'ev': -10, 'co': 80, 'type': 'Grenze', 'info': 'P6 (Oben Links)'},
    {'ev': 5, 'co': 80, 'type': 'Grenze', 'info': 'Mitte Oben'},
    {'ev': 15, 'co': 80, 'type': 'Grenze', 'info': 'P8 (Oben Rechts)'},
    # LINKS / SCHNITT
    {'ev': -25, 'co': 71, 'type': 'Grenze', 'info': 'Flanke Links'},
    {'ev': -35, 'co': 65, 'type': 'Grenze', 'info': 'P5 (Links)'},
    {'ev': -12.5, 'co': 37.5, 'type': 'Grenze', 'info': 'Schnitt Mitte (P3-P5)'},
    # RECHTS
    {'ev': 25, 'co': 70, 'type': 'Grenze', 'info': 'P1 (Rechts)'},
    {'ev': 25, 'co': 48, 'type': 'Grenze', 'info': 'Flanke Rechts'},
    # UNTEN
    {'ev': 25, 'co': 25, 'type': 'Grenze', 'info': 'P2 (Ecke Rechts Unten)'},
    {'ev': 17.5, 'co': 17.5, 'type': 'Grenze', 'info': 'Mitte Unten (P2-P3)'},
    {'ev': 10, 'co': 10, 'type': 'Grenze', 'info': 'P3 (Ecke Unten)'}
]

cloud_points = [
    {'ev': -10, 'co': 68, 'type': 'Innen', 'info': 'Feld Oben L'},
    {'ev': 5, 'co': 65, 'type': 'Innen', 'info': 'Feld Oben M'},
    {'ev': -18, 'co': 58, 'type': 'Innen', 'info': 'Feld Mitte L'},
    {'ev': 0, 'co': 55, 'type': 'Innen', 'info': 'Feld Mitte Z'},
    {'ev': 15, 'co': 52, 'type': 'Innen', 'info': 'Feld Mitte R'},
    {'ev': -20, 'co': 48, 'type': 'Innen', 'info': 'Feld Schnitt'},
    {'ev': 10, 'co': 40, 'type': 'Innen', 'info': 'Feld Unten M'},
    {'ev': -5, 'co': 45, 'type': 'Innen', 'info': 'Feld Unten L'},
    {'ev': 5, 'co': 28, 'type': 'Innen', 'info': 'Feld Tief'}
]

final_data = skeleton_points + cloud_points

# --- 3. DATENVERARBEITUNG ---

df = pd.DataFrame(final_data)

# Temperaturen berechnen (Ambient, Vorlauf, Rücklauf)
df[['T_ambient', 'T_vorlauf', 'T_ruecklauf']] = df.apply(map_to_temperatures, axis=1)

# Mapping für Simulation (Simulation braucht Rücklauf als 'T_kond_in')
df['T_eva_in'] = df['T_ambient']
df['T_kond_in'] = df['T_ruecklauf']

# Massenströme
df['m_flow_eva'] = 0.6  # Konstant Luft
df['m_flow_kond'] = df['T_ambient'].apply(calculate_variable_mass_flow)

# Sortierung (nach Vorlauf, sieht schöner aus)
df = df.sort_values(by=['T_vorlauf', 'T_ambient'], ascending=[False, True]).reset_index(drop=True)
df.index = df.index + 1
df.index.name = 'Nr'

# Export
export_cols = [
    'T_eva_in', 'm_flow_eva',
    'T_kond_in', 'm_flow_kond',
    'T_ambient',
    'type', 'info', 'T_vorlauf'
]

try:
    df[export_cols].to_excel(EXPORT_FILENAME)
    print(f"Erfolg: Datei '{EXPORT_FILENAME}' erstellt.")
    print(f"Logik: T_Vorlauf = T_Sat - {PINCH_COND}K | T_Rücklauf = T_Vorlauf - {TARGET_SPREAD}K")
except Exception as e:
    print(f"Fehler beim Speichern: {e}")

# --- 4. VISUALISIERUNG ---

plt.figure(figsize=(12, 9))

# A. Grenzen Transformieren (Wir wollen die Grenzen im Vorlauf-Diagramm sehen)
# Grenze_Vorlauf = Grenze_Sat - Pinch
corners_poly = [(-35, 65), (-10, 80), (15, 80), (25, 70), (25, 25), (10, 10), (-35, 65)]
vl_poly = [(x + OFFSET_AIR, y - PINCH_COND) for x, y in corners_poly]  # Y-Achse ist jetzt Vorlauf!
px, py = zip(*vl_poly)

plt.plot(px, py, 'k-', lw=1, alpha=0.3, label='Grenzen (Vorlauftemperatur)')
plt.fill(px, py, 'gray', alpha=0.05)

# Schnittlinie auch transformieren
p3 = (10 + OFFSET_AIR, 10 - PINCH_COND)
p5 = (-35 + OFFSET_AIR, 65 - PINCH_COND)
plt.plot([p3[0], p5[0]], [p3[1], p5[1]], 'r--', lw=2, alpha=0.5, label='Schnittlinie')

# B. Punkte Plotten (Y-Achse = Vorlauf!)
sizes = df['m_flow_kond'] * 500

for idx, row in df.iterrows():
    x = row['T_ambient']
    y = row['T_vorlauf']  # <--- Wir plotten den Vorlauf

    if row['type'] == 'Grenze':
        c, m = 'darkblue', 's'
    else:
        c, m = 'dodgerblue', 'o'

    s = row['m_flow_kond'] * 500

    plt.scatter(x, y, c=c, marker=m, s=s, edgecolors='white', zorder=5, alpha=0.8)
    plt.text(x, y - 2.5, str(idx), ha='center', va='top', fontsize=9, fontweight='bold')

plt.title(f'Betriebspunkte (Y-Achse = Heizungs-Vorlauf)\nSimulation erhält Rücklauf (VL - {TARGET_SPREAD}K)')
plt.xlabel('Außenlufttemperatur [°C]')
plt.ylabel('Heizwasser Vorlauftemperatur [°C]')
plt.legend(loc='upper left')
plt.grid(True, ls=':', alpha=0.6)
plt.xlim(-35, 40)
plt.ylim(0, 85)
plt.tight_layout()
plt.show()

print("\nDatenvorschau (Auszug):")
print(df[['T_ambient', 'T_vorlauf', 'T_kond_in', 'm_flow_kond']].head().to_string())