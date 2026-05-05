import pandas as pd
import numpy as np
import os


def finde_eev_randpunkte(excel_ihx, excel_sc):
    """
    Analysiert die Simulationsergebnisse und findet die extremen
    Betriebspunkte für die EEV-Auslegung, getrennt für IHX und SC.
    """

    def lade_und_bereite_auf(filepath, is_ihx):
        if not os.path.exists(filepath):
            print(f"FEHLER: Datei '{filepath}' nicht gefunden.")
            return None

        print(f"Lese Daten aus: {os.path.basename(filepath)}")
        df = pd.read_excel(filepath)

        def get_col(keyword):
            cols = [c for c in df.columns if pd.notna(c) and keyword.lower() in str(c).lower()]
            return cols[0] if cols else None

        # Relevante Basis-Spalten suchen
        col_m_flow = get_col('m_flow_ref')
        col_p_con = get_col('p_con')
        col_p_eva = get_col('p_eva')
        col_Q_con = get_col('Q_con in W')
        col_cop = get_col('COP in')
        col_sh = get_col('dT_eva_superheating')
        col_pel = get_col('P_el in W')
        col_conv = get_col('converged')

        # Dynamische Zustandsfindung vor dem Ventil
        # IHX = Zustand 4, SC = Zustand 3
        if is_ihx:
            col_rho_in = get_col('rho_5')
            col_T_in = get_col('T_5')
        else:
            col_rho_in = get_col('rho_3')
            col_T_in = get_col('T_3')

        # Checken ob essenzielle Spalten fehlen
        essentials = {
            'm_flow_ref': col_m_flow, 'p_con': col_p_con, 'p_eva': col_p_eva,
            'rho_in': col_rho_in, 'T_in': col_T_in, 'Q_con': col_Q_con,
            'COP': col_cop, 'superheating': col_sh, 'P_el': col_pel
        }

        missing = [name for name, val in essentials.items() if val is None]
        if missing:
            print(f"-> FEHLER in {os.path.basename(filepath)}: Konnte folgende Spalten nicht finden: {missing}")
            return None

        # 1. FILTERN
        mask_valid = (df[col_cop] > 0) & (df[col_sh] > 0)

        if col_conv is not None:
            mask_valid = mask_valid & (df[col_conv] == 1)

        df_valid = df[mask_valid].copy()

        if df_valid.empty:
            print(f"-> WARNUNG: Keine gültigen Betriebspunkte in {os.path.basename(filepath)}!")
            return None

        # 2. EINHEITEN UMRECHNEN (Pascal zu Bar, falls nötig)
        p_con_bar = df_valid[col_p_con] / 1e5 if 'pa' in col_p_con.lower() else df_valid[col_p_con]
        p_eva_bar = df_valid[col_p_eva] / 1e5 if 'pa' in col_p_eva.lower() else df_valid[col_p_eva]

        # 3. HILFSWERTE BERECHNEN
        m_flow_kg_h = df_valid[col_m_flow] * 3600
        dp = p_con_bar - p_eva_bar

        df_valid['Kv_proxy'] = m_flow_kg_h / np.sqrt(df_valid[col_rho_in] * dp)
        df_valid['m_flow_kg_h'] = m_flow_kg_h
        df_valid['dp_bar'] = dp
        df_valid['p_con_bar'] = p_con_bar
        df_valid['p_eva_bar'] = p_eva_bar
        df_valid['Q_eva_kW'] = (df_valid[col_Q_con] - df_valid[col_pel]) / 1000.0
        df_valid['T_vor_Ventil_C'] = df_valid[col_T_in] - 273.15

        # Leere Werte (NaN) sicherheitshalber droppen, falls Berechnung fehlgeschlagen ist
        df_valid = df_valid.dropna(subset=['Kv_proxy'])

        return df_valid

    # --- OUTPUT FUNKTION ---
    def print_coolselector_inputs(name, bp, df_original_cols):
        def get_val(keyword):
            col = [c for c in df_original_cols if pd.notna(c) and keyword.lower() in str(c).lower()][0]
            return bp[col]

        sh_K = get_val('dT_eva_superheating')

        print(f"\n{'-' * 60}")
        print(f" {name.upper()} ")
        print(f"{'-' * 60}")
        print(f"Filter-Indikator (Kv_proxy):  {bp['Kv_proxy']:.4f}")
        print(f"Massenstrom:                  {bp['m_flow_kg_h']:.1f} kg/h")
        print(f"Druckdifferenz (Brutto):      {bp['dp_bar']:.2f} bar")
        print(">>> TRAGE DIESE WERTE IN COOLSELECTOR EIN <<<")
        print(f"1. Cooling Capacity (Q_0):    {bp['Q_eva_kW']:.2f} kW")
        print(f"2. Evaporation Pressure:      {bp['p_eva_bar']:.2f} bar")
        print(f"3. Condensation Pressure:     {bp['p_con_bar']:.2f} bar")
        print(f"4. Useful Superheat:          {sh_K:.1f} K")
        print(f"5. Liquid Temp vor Ventil:    {bp['T_vor_Ventil_C']:.1f} °C")

    # --- DATEN VERARBEITEN ---
    print("\n" + "=" * 60)
    print(" AUSWERTUNG SC-KREIS (OHNE IHX)")
    print("=" * 60)
    df_sc = lade_und_bereite_auf(excel_sc, is_ihx=False)

    if df_sc is not None:
        idx_max_sc = df_sc['Kv_proxy'].idxmax()
        bp_max_sc = df_sc.loc[idx_max_sc]
        idx_min_sc = df_sc['Kv_proxy'].idxmin()
        bp_min_sc = df_sc.loc[idx_min_sc]

        print_coolselector_inputs("SC: Maximaler Oeffnungsgrad (Groesster Massenstrom/Kleinstes dp)", bp_max_sc,
                                  df_sc.columns)
        print_coolselector_inputs("SC: Minimaler Oeffnungsgrad (Kleinster Massenstrom/Groesstes dp)", bp_min_sc,
                                  df_sc.columns)

    print("\n\n" + "=" * 60)
    print(" AUSWERTUNG IHX-KREIS (MIT IHX)")
    print("=" * 60)
    df_ihx = lade_und_bereite_auf(excel_ihx, is_ihx=True)

    if df_ihx is not None:
        idx_max_ihx = df_ihx['Kv_proxy'].idxmax()
        bp_max_ihx = df_ihx.loc[idx_max_ihx]
        idx_min_ihx = df_ihx['Kv_proxy'].idxmin()
        bp_min_ihx = df_ihx.loc[idx_min_ihx]

        print_coolselector_inputs("IHX: Maximaler Oeffnungsgrad (Groesster Massenstrom/Kleinstes dp)", bp_max_ihx,
                                  df_ihx.columns)
        print_coolselector_inputs("IHX: Minimaler Oeffnungsgrad (Kleinster Massenstrom/Groesstes dp)", bp_min_ihx,
                                  df_ihx.columns)
    print("=" * 60 + "\n")