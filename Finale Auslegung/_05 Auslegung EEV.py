import pandas as pd
import numpy as np
import os


def finde_eev_randpunkte(excel_ihx, excel_sc):
    """
    Analysiert die Simulationsergebnisse und findet die extremen Betriebspunkte
    für die Auslegung des elektronischen Expansionsventils (EEV) in Coolselector2.
    Dabei werden die Zustände für einen Kreisprozess mit IHX und für einen
    Standard-Kreisprozess (SC) separat untersucht.

    Args:
        excel_ihx (str): Pfad zu den Ergebnissen mit internem Wärmeübertrager.
        excel_sc (str): Pfad zu den Ergebnissen ohne IHX (Subcooling).
    """

    def lade_und_bereite_auf(filepath, is_ihx):
        """Hilfsfunktion zum Laden und Aufbereiten der spezifischen Spalten."""
        if not os.path.exists(filepath):
            print(f"FEHLER: Datei '{filepath}' nicht gefunden.")
            return None

        print(f"Lese Daten aus: {os.path.basename(filepath)}")
        df = pd.read_excel(filepath)

        def get_col(keyword):
            cols = [c for c in df.columns if pd.notna(c) and keyword.lower() in str(c).lower()]
            return cols[0] if cols else None

        col_m_flow = get_col('m_flow_ref')
        col_p_con = get_col('p_con')
        col_p_eva = get_col('p_eva')
        col_Q_con = get_col('Q_con in W')
        col_cop = get_col('COP in')
        col_sh = get_col('dT_eva_superheating')
        col_pel = get_col('P_el in W')
        col_conv = get_col('converged')

        # Dynamische Zustandsfindung vor dem Ventil:
        # Mit IHX ist das Ventil hinter dem IHX (Zustand 4/5), ohne IHX direkt nach Kondensator (Zustand 3)
        if is_ihx:
            col_rho_in = get_col('rho_5')
            col_T_in = get_col('T_5')
        else:
            col_rho_in = get_col('rho_3')
            col_T_in = get_col('T_3')

        essentials = {
            'm_flow_ref': col_m_flow, 'p_con': col_p_con, 'p_eva': col_p_eva,
            'rho_in': col_rho_in, 'T_in': col_T_in, 'Q_con': col_Q_con,
            'COP': col_cop, 'superheating': col_sh, 'P_el': col_pel
        }

        missing = [name for name, val in essentials.items() if val is None]
        if missing:
            print(f"-> FEHLER in {os.path.basename(filepath)}: Konnte folgende Spalten nicht finden: {missing}")
            return None

        # 1. FILTERN: Nur physikalisch sinnvolle & konvergierte Ergebnisse
        mask_valid = (df[col_cop] > 0) & (df[col_sh] > 0)
        if col_conv is not None:
            mask_valid = mask_valid & (df[col_conv] == 1)

        df_valid = df[mask_valid].copy()
        if df_valid.empty: return None

        # 2. EINHEITEN UMRECHNEN (Pascal zu Bar)
        p_con_bar = df_valid[col_p_con] / 1e5 if 'pa' in col_p_con.lower() else df_valid[col_p_con]
        p_eva_bar = df_valid[col_p_eva] / 1e5 if 'pa' in col_p_eva.lower() else df_valid[col_p_eva]

        # 3. K_v-PROXY BERECHNEN: proportional zu m_dot / sqrt(rho * dp)
        m_flow_kg_h = df_valid[col_m_flow] * 3600
        dp = p_con_bar - p_eva_bar

        df_valid['Kv_proxy'] = m_flow_kg_h / np.sqrt(df_valid[col_rho_in] * dp)
        df_valid['m_flow_kg_h'] = m_flow_kg_h
        df_valid['dp_bar'] = dp
        df_valid['p_con_bar'] = p_con_bar
        df_valid['p_eva_bar'] = p_eva_bar
        df_valid['Q_eva_kW'] = (df_valid[col_Q_con] - df_valid[col_pel]) / 1000.0
        df_valid['T_vor_Ventil_C'] = df_valid[col_T_in] - 273.15

        return df_valid.dropna(subset=['Kv_proxy'])

    # --- OUTPUT FUNKTION FÜR COOLSELECTOR ---
    def print_coolselector_inputs(name, bp, df_original_cols):
        def get_val(keyword):
            col = [c for c in df_original_cols if pd.notna(c) and keyword.lower() in str(c).lower()][0]
            return bp[col]

        sh_K = get_val('dT_eva_superheating')
        print(f"\n{'-' * 60}\n {name.upper()} \n{'-' * 60}")
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
    print("\n" + "=" * 60 + "\n AUSWERTUNG SC-KREIS (OHNE IHX)\n" + "=" * 60)
    df_sc = lade_und_bereite_auf(excel_sc, is_ihx=False)
    if df_sc is not None:
        print_coolselector_inputs("SC: Maximaler Oeffnungsgrad (Groesster Massenstrom/Kleinstes dp)",
                                  df_sc.loc[df_sc['Kv_proxy'].idxmax()], df_sc.columns)
        print_coolselector_inputs("SC: Minimaler Oeffnungsgrad (Kleinster Massenstrom/Groesstes dp)",
                                  df_sc.loc[df_sc['Kv_proxy'].idxmin()], df_sc.columns)

    print("\n\n" + "=" * 60 + "\n AUSWERTUNG IHX-KREIS (MIT IHX)\n" + "=" * 60)
    df_ihx = lade_und_bereite_auf(excel_ihx, is_ihx=True)
    if df_ihx is not None:
        print_coolselector_inputs("IHX: Maximaler Oeffnungsgrad (Groesster Massenstrom/Kleinstes dp)",
                                  df_ihx.loc[df_ihx['Kv_proxy'].idxmax()], df_ihx.columns)
        print_coolselector_inputs("IHX: Minimaler Oeffnungsgrad (Kleinster Massenstrom/Groesstes dp)",
                                  df_ihx.loc[df_ihx['Kv_proxy'].idxmin()], df_ihx.columns)
    print("=" * 60 + "\n")