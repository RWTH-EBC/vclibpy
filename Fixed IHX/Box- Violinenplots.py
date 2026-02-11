import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
import os

# --- 1. KONFIGURATION ---
file_ihx = "IHX_Propane_Auslegung.xlsx"
file_std = "SC_Propane_Auslegung.xlsx"
output_folder = "Box- & Violinenplots_3"

#Verdichter-Grenzen (Filter)
LIMITS = {
    'MAX_T_DISCHARGE': 110.0,  # °C
    'MAX_P_CON_BAR': 27.0,  # bar
    'MAX_P_EL_KW': 5.0,  # kW
    'MAX_RPM': 6600  # 1/min
}

# Ordner erstellen
os.makedirs(output_folder, exist_ok=True)
print(f"Speichere Ergebnisse in: {os.path.abspath(output_folder)}")


# --- 2. HILFSFUNKTIONEN ---

def shorten_names(df):
    """Kürzt lange Spaltennamen für die x-Achse."""
    new_names = {}
    for col in df.columns:
        short_name = str(col).split(' ')[0]
        # Spezielle Kürzung für T_1, p_1 etc.
        if len(short_name) == 3 and short_name[1] == '_':
            if short_name.startswith('T') or short_name.startswith('p'):
                short_name = short_name.replace('_', '')
        new_names[col] = short_name
    return df.rename(columns=new_names)


def load_process_and_filter(filepath, system_name):
    print(f"\nVerarbeite {system_name} ({filepath})...")
    if not os.path.exists(filepath):
        print(f"FEHLER: Datei '{filepath}' nicht gefunden.")
        return None

    try:
        df = pd.read_excel(filepath)
    except Exception as e:
        print(f"Fehler beim Lesen: {e}")
        return None

    # --- A. Einheiten korrigieren ---

    # Druck (Pascal -> Bar)
    p_cols = [c for c in df.columns if c.strip().startswith('p_') or c.strip().startswith('dp_')]
    for col in p_cols:
        if df[col].mean() > 100:
            df[col] = df[col] / 100000

    # Temperatur (Kelvin -> °C)
    t_cols = [c for c in df.columns if c.strip().startswith('T_')]
    for col in t_cols:
        if df[col].mean() > 200:
            df[col] = df[col] - 273.15

    # Leistung (W -> kW)
    p_el_col = next((c for c in df.columns if c.strip().startswith('P_el')), None)
    if p_el_col: df['P_el_kW'] = df[p_el_col] / 1000

    q_eva_col = next((c for c in df.columns if c.strip().startswith('Q_eva_outer')), None)
    if q_eva_col: df['Q_eva_kW'] = df[q_eva_col] / 1000

    # Drehzahl (Hz -> RPM)
    n_col = next((c for c in df.columns if c.strip().startswith('n ')), None)
    if n_col: df['n_rpm'] = df[n_col] * 60

    # --- B. Filtern nach LIMITS ---
    initial_count = len(df)
    mask = pd.Series([True] * len(df))

    t2_col = next((c for c in df.columns if c.startswith('T_2')), None)
    if t2_col: mask = mask & (df[t2_col] <= LIMITS['MAX_T_DISCHARGE'])

    pcon_col = next((c for c in df.columns if c.startswith('p_con')), None)
    if pcon_col: mask = mask & (df[pcon_col] <= LIMITS['MAX_P_CON_BAR'])

    if 'P_el_kW' in df.columns: mask = mask & (df['P_el_kW'] <= LIMITS['MAX_P_EL_KW'])
    if 'n_rpm' in df.columns: mask = mask & (df['n_rpm'] <= LIMITS['MAX_RPM'])

    df_filtered = df[mask].copy()
    print(f" -> {initial_count - len(df_filtered)} Punkte wegen Limits entfernt.")

    if len(df_filtered) == 0:
        print("WARNUNG: Alle Daten gefiltert!")
        return None

    # --- C. Namen kürzen ---
    df_final = shorten_names(df_filtered)
    df_final['System'] = system_name

    return df_final


# --- 3.Maxima ---

def add_max_markers(ax, df_melted, x_col, y_col, order):
    """
    Fügt rote Sterne und Text-Label für den maximalen Wert jeder Kategorie hinzu.
    """
    # Berechne Maxima pro Gruppe
    max_vals = df_melted.groupby(x_col)[y_col].max()

    # Gehe die x-Achsen-Kategorien in der exakten Plot-Reihenfolge durch
    for i, category in enumerate(order):
        if category in max_vals:
            y_max = max_vals[category]

            # Zeichne roten Stern
            ax.plot(i, y_max, marker='*', color='red', markersize=8, markeredgecolor='black', zorder=10)

            # Schreibe Wert daneben (leicht versetzt)
            ax.text(i, y_max, f'{y_max:.1f}',
                    ha='center', va='bottom',
                    fontsize=9, color='darkred', fontweight='bold')


# --- 4. PLOTTING FUNKTION ---

def create_plots(df, system_name):
    if df is None: return

    c_p = 'skyblue'
    c_t = 'salmon'

    # Sortier-Hilfe
    def sort_key(name):
        if 'eva' in name: return 0
        if 'con' in name and 'p' in name: return 100
        try:
            return int(''.join(filter(str.isdigit, name)))
        except:
            return 50

    # A) DRUCK
    p_cols = sorted([c for c in df.columns if c.startswith('p') or c.startswith('dp')], key=sort_key)
    if p_cols:
        df_p = df.melt(value_vars=p_cols, var_name='Pos', value_name='Bar')
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        # WICHTIG: Order festlegen, damit Marker passen
        order_p = p_cols

        # Boxplot mit Max-Marker
        sns.boxplot(data=df_p, x='Pos', y='Bar', ax=axes[0], color=c_p, order=order_p)
        add_max_markers(axes[0], df_p, 'Pos', 'Bar', order_p)  # <--- NEU
        axes[0].set_title(f'Druck ({system_name}) - Stern = Max. Wert')
        axes[0].grid(True, linestyle='--', alpha=0.5)

        sns.violinplot(data=df_p, x='Pos', y='Bar', ax=axes[1], color=c_p, inner="quartile", order=order_p)
        axes[1].set_title(f'Druck Verteilung ({system_name})')
        axes[1].grid(True, linestyle='--', alpha=0.5)

        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, f"{system_name}_Pressure.png"), dpi=300)
        plt.close()

    # B) TEMPERATUR
    t_cols = sorted([c for c in df.columns if c.startswith('T')], key=sort_key)
    if t_cols:
        df_t = df.melt(value_vars=t_cols, var_name='Pos', value_name='Grad_C')
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        order_t = t_cols

        # Boxplot mit Max-Marker
        sns.boxplot(data=df_t, x='Pos', y='Grad_C', ax=axes[0], color=c_t, order=order_t)
        add_max_markers(axes[0], df_t, 'Pos', 'Grad_C', order_t)  # <--- NEU

        axes[0].set_title(f'Temperatur ({system_name}) - Stern = Max. Wert')
        axes[0].grid(True, linestyle='--', alpha=0.5)
        axes[0].axhline(y=LIMITS['MAX_T_DISCHARGE'], color='red', linestyle='--', alpha=0.5, label='Limit')

        sns.violinplot(data=df_t, x='Pos', y='Grad_C', ax=axes[1], color=c_t, inner="quartile", order=order_t)
        axes[1].set_title(f'Temperatur Verteilung ({system_name})')
        axes[1].grid(True, linestyle='--', alpha=0.5)

        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, f"{system_name}_Temperature.png"), dpi=300)
        plt.close()

    # C) KOMPONENTEN
    has_m = 'm_flow_ref' in df.columns
    has_q = 'Q_eva_kW' in df.columns

    if has_m or has_q:
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        if has_m:
            sns.boxplot(y=df['m_flow_ref'], ax=axes[0], color='lightgreen')
            # Hier markieren wir Max manuell, da kein x-Axis Array
            max_m = df['m_flow_ref'].max()
            axes[0].plot(0, max_m, marker='*', color='red', markersize=10)
            axes[0].text(0, max_m, f'{max_m:.3f}', ha='center', va='bottom', color='darkred', fontweight='bold')

            axes[0].set_title(f'Massenstrom ({system_name})')
            axes[0].set_ylabel('kg/s')
            axes[0].grid(True, linestyle='--')

        if has_q:
            sns.boxplot(y=df['Q_eva_kW'], ax=axes[1], color='gold')
            max_q = df['Q_eva_kW'].max()
            axes[1].plot(0, max_q, marker='*', color='red', markersize=10)
            axes[1].text(0, max_q, f'{max_q:.1f}', ha='center', va='bottom', color='darkred', fontweight='bold')

            axes[1].set_title(f'Verdampfer-Leistung ({system_name})')
            axes[1].set_ylabel('kW')
            axes[1].grid(True, linestyle='--')

        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, f"{system_name}_Components.png"), dpi=300)
        plt.close()


# --- 5. MAIN EXECUTION ---

# IHX
df_ihx = load_process_and_filter(file_ihx, "IHX")
create_plots(df_ihx, "IHX_Cycle")

# Standard
df_std = load_process_and_filter(file_std, "Standard")
create_plots(df_std, "Standard_Cycle")

# Vergleich
if df_ihx is not None and df_std is not None:
    print("\nErstelle Vergleichs-Plot...")
    df_compare = pd.concat([df_ihx, df_std], ignore_index=True)

    if 'm_flow_ref' in df_compare.columns:
        plt.figure(figsize=(8, 6))
        ax = sns.boxplot(data=df_compare, x='System', y='m_flow_ref', palette="Set2")

        # Auch im Vergleichsplot Maxima markieren
        add_max_markers(ax, df_compare, 'System', 'm_flow_ref', ['IHX', 'Standard'])

        plt.title('Vergleich: Massenstrom (Sensor Auslegung)')
        plt.ylabel('Massenstrom [kg/s]')
        plt.grid(True, linestyle='--')
        plt.savefig(os.path.join(output_folder, "Vergleich_Massenstrom.png"), dpi=300)
        plt.close()

print(f"\nFertig! Alle Bilder liegen in '{output_folder}'.")