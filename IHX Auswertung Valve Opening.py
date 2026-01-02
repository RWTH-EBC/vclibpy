import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
import sys
import logging

# Konfiguration für Logging (damit dein Block funktioniert)
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# =============================================================================
# 1. DATEN LADEN (Dein Block)
# =============================================================================

# Automatisch die neueste passende Datei finden
excel_filename = 'IHX_Propane_Valve_Check.xlsx'

if not os.path.exists(excel_filename):
    logging.error(f"Datei '{excel_filename}' nicht gefunden. Bitte erst Betriebspunkte generieren.")
    # Fallback für Testzwecke, falls User den Namen geändert hat
    sys.exit()

print(f"Lade Daten aus: {excel_filename} ...")

df_raw = pd.read_excel(excel_filename)
df_raw.columns = df_raw.columns.str.strip()  # Leerzeichen entfernen

# =============================================================================
# 2. DATEN AUFBEREITEN & MAPPING
# =============================================================================

# Wir erstellen ein sauberes DataFrame nur mit den benötigten Daten
df = pd.DataFrame()

# Mapping basierend auf deiner Spalten-Liste
# Wir nutzen .get(), um Abstürze zu vermeiden, falls eine Spalte fehlt
try:
    df['Valve_Opening'] = df_raw['opening in - (Opening High-Side EV)']
    df['COP'] = df_raw['COP in - (Coefficient of Performance)']
    df['Q_con_W'] = df_raw['Q_con in W (Condenser refrigerant heat flow rate)']
    df['T_Amb_K'] = df_raw['T_ambient in K (Ambient temperature)']

    # Optional: Zwischendruck für physikalische Erklärung (falls vorhanden)
    if 'p_4 in Pa (Pressure in state 4)' in df_raw.columns:
        df['p_Zwischen_Pa'] = df_raw['p_4 in Pa (Pressure in state 4)']
    else:
        df['p_Zwischen_Pa'] = np.nan

    # Optional: Drehzahl zur Filterung
    if 'n in Hz (Compressor speed)' in df_raw.columns:
        df['n_Hz'] = df_raw['n in Hz (Compressor speed)']
    elif 'n in Hz (None)' in df_raw.columns:
        df['n_Hz'] = df_raw['n in Hz (None)']
    else:
        df['n_Hz'] = 0

except KeyError as e:
    logging.error(f"Spalte nicht gefunden: {e}")
    logging.error("Bitte prüfe, ob die Excel-Datei wirklich die genannten Spalten enthält.")
    sys.exit()

# Einheiten umrechnen
df['T_Amb_C'] = (df['T_Amb_K'] - 273.15).round(1)
df['Q_con_kW'] = df['Q_con_W'] / 1000.0
df['p_Zwischen_bar'] = df['p_Zwischen_Pa'] / 100000.0

# Filtern: Nur gültige physikalische Punkte
df = df[df['COP'] > 0].copy()

print(f"Daten geladen: {len(df)} gültige Simulationspunkte.")

# =============================================================================
# 3. VISUALISIERUNG (TRENDS)
# =============================================================================

sns.set_theme(style="whitegrid")
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

fig.suptitle(f'Analyse: Lohnt sich Expansion vor dem IHX? ({excel_filename})', fontsize=16)

# --- PLOT 1: COP vs. Valve Opening ---
# Wir nutzen lineplot mit 'hue', um verschiedene Außentemperaturen zu unterscheiden.
# Seaborn berechnet automatisch den Mittelwert über alle Drehzahlen (mit Konfidenzintervall).
sns.lineplot(
    data=df, x='Valve_Opening', y='COP',
    hue='T_Amb_C', palette='viridis', marker='o',
    ax=axes[0]
)
axes[0].set_title('Effizienz (COP)')
axes[0].set_xlabel('Ventilöffnung (0=Zu, 1=Offen)')
axes[0].set_ylabel('COP [-]')
axes[0].legend(title='T_Amb [°C]')

# --- PLOT 2: Heizleistung vs. Valve Opening ---
sns.lineplot(
    data=df, x='Valve_Opening', y='Q_con_kW',
    hue='T_Amb_C', palette='magma', marker='s',
    ax=axes[1]
)
axes[1].set_title('Heizleistung (Q_con)')
axes[1].set_xlabel('Ventilöffnung')
axes[1].set_ylabel('Leistung [kW]')
axes[1].get_legend().remove()  # Legende redundant

# --- PLOT 3: Zwischendruck (Physik-Check) ---
if df['p_Zwischen_bar'].notna().any():
    sns.lineplot(
        data=df, x='Valve_Opening', y='p_Zwischen_bar',
        hue='T_Amb_C', palette='coolwarm', marker='^',
        ax=axes[2]
    )
    axes[2].set_title('Zwischendruck (vor IHX)')
    axes[2].set_xlabel('Ventilöffnung')
    axes[2].set_ylabel('Druck [bar]')
    axes[2].get_legend().remove()
else:
    axes[2].text(0.5, 0.5, "Keine Druckdaten (p_4) vorhanden", ha='center')

plt.tight_layout()
output_img = "IHX_Valve_Analysis.png"
plt.savefig(output_img, dpi=150)
print(f"Grafik gespeichert als: {output_img}")

# =============================================================================
# 4. TEXT-ANALYSE & FAZIT
# =============================================================================

print("\n" + "=" * 60)
print("FAZIT: SOLLTE MAN VOR DEM IHX ENTSPANNEN?")
print("=" * 60)

# Wir berechnen den Durchschnitts-COP für jede Ventilöffnung
summary = df.groupby('Valve_Opening')[['COP', 'Q_con_kW']].mean().reset_index()

# Finde das Optimum
best_idx = summary['COP'].idxmax()
best_opening = summary.loc[best_idx, 'Valve_Opening']
worst_opening = summary.loc[summary['COP'].idxmin(), 'Valve_Opening']

print("\nDurchschnittswerte (über alle Temperaturen/Drehzahlen):")
print(summary.to_string(index=False, float_format="%.3f"))

print("\nINTERPRETATION:")
if best_opening >= 0.9:
    print(f"[X] NEIN. Bester COP bei Ventilöffnung {best_opening}.")
    print("    Trend: Je weiter das Ventil offen ist, desto besser.")
    print("    -> Empfehlung: Keine Vorentspannung. Ventil weglassen oder voll offen halten.")
    print("       Der IHX arbeitet am effizientesten mit hohem Druck (Flüssigkeit).")

elif best_opening <= 0.4:
    print(f"[!] JA. Bester COP bei Ventilöffnung {best_opening}.")
    print("    Trend: Starke Drosselung verbessert die Effizienz.")
    print("    -> Empfehlung: Vorentspannung ist sinnvoll.")

else:
    print(f"[~] TEILS/TEILS. Bester COP bei Ventilöffnung {best_opening}.")
    print("    Es gibt ein Optimum im mittleren Bereich.")
    print("    -> Ein regelbares Ventil könnte sinnvoll sein.")

# Spezial-Check für kalte Temperaturen (wo IHX oft wichtig ist)
df_cold = df[df['T_Amb_C'] <= -5]
if not df_cold.empty:
    best_cold = df_cold.groupby('Valve_Opening')['COP'].mean().idxmax()
    print(f"\nHinweis für tiefe Temperaturen (<= -5°C): Optimum bei Öffnung {best_cold}")

# Grafik anzeigen
plt.show()