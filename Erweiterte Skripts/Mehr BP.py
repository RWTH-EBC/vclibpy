import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# =============================================================================
# 1. KONFIGURATION GEBÄUDE & PHYSIK
# =============================================================================

# Parameter für die Massenstrom-Berechnung
HOUSE_AREA = 150.0  # m²
SPECIFIC_LOAD = 70.0  # W/m²
T_DESIGN_OUT = -15.0  # °C
T_HEIZGRENZE = 15.0  # °C
CP_WATER = 4180.0  # J/(kg*K)
TARGET_SPREAD = 7.0  # K (Ziel-Spreizung)

# Pinch-Point & Offsets
PINCH_COND = 3.0  # K
OFFSET_AIR = 8.0  # K

MAX_HEATING_LOAD_KW = (HOUSE_AREA * SPECIFIC_LOAD) / 1000.0

# --- NEU: FEUCHTE-LOGIK ---
# Grenze für Frostbildung (nach Zhu et al.)
T_FROST_LIMIT = 7.0  # °C

# Feuchte-Schritte für den Frostbereich (< 7°C)
HUMIDITY_STEPS = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

# Feste Feuchte für den warmen Bereich (>= 7°C)
# Dient nur als Platzhalter, da hier kein Frost entsteht
HUMIDITY_FIXED = 0.5

# Dateiname
EXPORT_FILENAME = 'betriebspunkte_mit_feuchte.xlsx'


# =============================================================================
# BERECHNUNGS-FUNKTIONEN
# =============================================================================

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
    """Wandelt Kennfeld-Koordinaten in System-Temperaturen um."""
    evap_sat = row['ev']
    cond_sat = row['co']

    t_ambient = evap_sat + OFFSET_AIR
    t_vorlauf = cond_sat - PINCH_COND
    t_ruecklauf = t_vorlauf - TARGET_SPREAD

    return pd.Series([t_ambient, t_vorlauf, t_ruecklauf])


# =============================================================================
# 2. DEFINITION DER PUNKTE (Basis-Geometrie)
# =============================================================================

skeleton_points = [
    {'ev': -10, 'co': 80, 'type': 'Grenze', 'info': 'P6 (Oben Links)'},
    {'ev': 5, 'co': 80, 'type': 'Grenze', 'info': 'Mitte Oben'},
    {'ev': 15, 'co': 80, 'type': 'Grenze', 'info': 'P8 (Oben Rechts)'},
    {'ev': -25, 'co': 71, 'type': 'Grenze', 'info': 'Flanke Links'},
    {'ev': -35, 'co': 65, 'type': 'Grenze', 'info': 'P5 (Links)'},
    {'ev': -12.5, 'co': 37.5, 'type': 'Grenze', 'info': 'Schnitt Mitte (P3-P5)'},
    {'ev': 25, 'co': 70, 'type': 'Grenze', 'info': 'P1 (Rechts)'},
    {'ev': 25, 'co': 48, 'type': 'Grenze', 'info': 'Flanke Rechts'},
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

base_data = skeleton_points + cloud_points

# =============================================================================
# 3. DATENVERARBEITUNG & FEUCHTE-EXPANSION
# =============================================================================

# Schritt A: Basis-Berechnungen (Temperaturen & Massenströme)
df_base = pd.DataFrame(base_data)
df_base[['T_ambient', 'T_vorlauf', 'T_ruecklauf']] = df_base.apply(map_to_temperatures, axis=1)
df_base['T_eva_in'] = df_base['T_ambient']
df_base['T_kond_in'] = df_base['T_ruecklauf']
df_base['m_flow_eva'] = 0.6
df_base['m_flow_kond'] = df_base['T_ambient'].apply(calculate_variable_mass_flow)

# Schritt B: Zeilen expandieren basierend auf Frostgrenze
expanded_rows = []

print("Generiere Betriebspunkte mit Feuchte-Variation...")

for _, row in df_base.iterrows():
    t_amb = row['T_ambient']

    if t_amb < T_FROST_LIMIT:
        # Kalt: Wir brauchen Simulationspunkte für alle Feuchten
        for phi in HUMIDITY_STEPS:
            new_row = row.copy()
            new_row['phi'] = phi
            # Info erweitern, damit man es später zuordnen kann
            new_row['info_ext'] = f"{row['info']} (phi={phi})"
            expanded_rows.append(new_row)
    else:
        # Warm: Reifbildung unwahrscheinlich -> Nur 1 Punkt mit Standardfeuchte
        new_row = row.copy()
        new_row['phi'] = HUMIDITY_FIXED
        new_row['info_ext'] = f"{row['info']} (warm)"
        expanded_rows.append(new_row)

# Das neue, große DataFrame erstellen
df_final = pd.DataFrame(expanded_rows)

# Sortierung und Index
df_final = df_final.sort_values(by=['T_vorlauf', 'T_ambient', 'phi'], ascending=[False, True, True]).reset_index(
    drop=True)
df_final.index = df_final.index + 1
df_final.index.name = 'Nr'

# =============================================================================
# 4. EXPORT
# =============================================================================

export_cols = [
    'T_eva_in',
    'phi',  # <--- NEUE SPALTE
    'm_flow_eva',
    'T_kond_in',
    'm_flow_kond',
    'T_ambient',
    'type',
    'info_ext',  # Erweiterte Info
    'T_vorlauf'
]

try:
    df_final[export_cols].to_excel(EXPORT_FILENAME)
    print(f"Erfolg: '{EXPORT_FILENAME}' erstellt.")
    print(f" -> Basis-Punkte: {len(df_base)}")
    print(f" -> Expandierte Simulations-Punkte: {len(df_final)}")
    print(f" -> Logik: Unter {T_FROST_LIMIT}°C werden {len(HUMIDITY_STEPS)} Feuchte-Varianten erstellt.")
except Exception as e:
    print(f"Fehler beim Speichern: {e}")

# =============================================================================
# 5. VISUALISIERUNG
# =============================================================================

plt.figure(figsize=(12, 9))

# Grenzen Transformieren
corners_poly = [(-35, 65), (-10, 80), (15, 80), (25, 70), (25, 25), (10, 10), (-35, 65)]
vl_poly = [(x + OFFSET_AIR, y - PINCH_COND) for x, y in corners_poly]
px, py = zip(*vl_poly)

plt.plot(px, py, 'k-', lw=1, alpha=0.3, label='Grenzen')
plt.fill(px, py, 'gray', alpha=0.05)

# Frostgrenze einzeichnen
plt.axvline(x=T_FROST_LIMIT, color='purple', linestyle='--', alpha=0.5, label='Frostgrenze (7°C)')

# Punkte Plotten (Wir plotten df_base, um Überlappungen zu vermeiden,
# aber wir nutzen df_final für die Statistik im Titel)
# Die "Bubbles" zeigen die Basis-Positionen.
for idx, row in df_base.iterrows():
    x = row['T_ambient']
    y = row['T_vorlauf']

    if row['type'] == 'Grenze':
        c, m = 'darkblue', 's'
    else:
        c, m = 'dodgerblue', 'o'

    # Markiere Punkte, die expandiert werden (links der Linie)
    if x < T_FROST_LIMIT:
        edge = 'red'  # Roter Rand = Wird expandiert
        lw = 2
    else:
        edge = 'white'
        lw = 1

    s = row['m_flow_kond'] * 500

    plt.scatter(x, y, c=c, marker=m, s=s, edgecolors=edge, linewidths=lw, zorder=5, alpha=0.8)

# Dummy Artists für Legende
plt.scatter([], [], c='dodgerblue', edgecolors='red', linewidths=2, label=f'Expandiert (x{len(HUMIDITY_STEPS)})')
plt.scatter([], [], c='dodgerblue', edgecolors='white', linewidths=1, label='Einzelpunkt')

plt.title(
    f'Betriebspunkte Generator\nGesamtanzahl Simulationen: {len(df_final)} (statt {len(df_base) * len(HUMIDITY_STEPS)})')
plt.xlabel('Außenlufttemperatur [°C]')
plt.ylabel('Heizwasser Vorlauftemperatur [°C]')
plt.legend(loc='upper left')
plt.grid(True, ls=':', alpha=0.6)
plt.xlim(-35, 40)
plt.ylim(0, 85)
plt.tight_layout()
plt.show()

print("\nDatenvorschau (Auszug mit Feuchte):")
print(df_final[['T_ambient', 'phi', 'info_ext']].head(10).to_string())