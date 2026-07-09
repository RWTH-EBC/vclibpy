from gum_ebc.Measurement import Measurement
from gum_ebc.Sensors import *
from gum_ebc.Terminals import *
from gum_ebc.configuration import *


def show(name, value):
    """
    Gibt Wert, Unsicherheit und relative Unsicherheit aus.
    Achtung:
    - Vor expand_uncertainty ist value.u normalerweise die Standardunsicherheit u.
    - Nach expand_uncertainty ist value.u die erweiterte Unsicherheit U.
    """
    print("\n" + "=" * 80)
    print(name)
    print(value)

    try:
        print("x =", value.x)
        print("u/U =", value.u)
        print("relative u/U =", abs(value.u / value.x) * 100, "%")
    except Exception as e:
        print("Keine x/u Ausgabe möglich:", e)


def print_interval(name, value, unit=""):
    """
    Gibt das Unsicherheitsintervall aus.

    Wenn value vorher mit expand_uncertainty(..., k=2) erweitert wurde,
    dann ist value.u hier die erweiterte Unsicherheit U.
    """
    lower = value.x - value.u
    upper = value.x + value.u

    print("\n" + "=" * 80)
    print(name)
    print(f"x = {value.x:.6g} {unit}")
    print(f"U = ±{value.u:.6g} {unit}")
    print(f"Intervall = [{lower:.6g}; {upper:.6g}] {unit}")
    print(f"relative erweiterte Unsicherheit = ±{abs(value.u / value.x) * 100:.3f} %")


def contribution_analysis_power(m_ref, h1, h2, k=2):
    """
    Beitragsanalyse für:

        P = m_ref * (h2 - h1) / 1000

    Einheit:
        m_ref in kg/s
        h1, h2 in J/kg
        P in kW

    Diese Funktion untersucht, welche der drei Größen
    m_ref, h1 oder h2 am stärksten zur Unsicherheit von P beiträgt.

    Annahmen:
        - m_ref, h1 und h2 sind unabhängig.
        - h1 und h2 werden hier als fertige Eingangsgrößen betrachtet.
        - Die Funktion zerlegt h1 und h2 NICHT weiter in T und p.
        - Für die Beitragsanalyse werden Standardunsicherheiten verwendet.
        - k wird nur am Ende zur erweiterten Unsicherheit verwendet.
    """

    # -------------------------------------------------------------------------
    # 1) Nominale Werte
    # -------------------------------------------------------------------------
    delta_h = h2.x - h1.x
    P = m_ref.x * delta_h / 1000

    # -------------------------------------------------------------------------
    # 2) Sensitivitätskoeffizienten
    # -------------------------------------------------------------------------
    # P = m_ref * (h2 - h1) / 1000
    #
    # Ableitungen:
    # dP/dm_ref = (h2 - h1) / 1000
    # dP/dh1    = -m_ref / 1000
    # dP/dh2    = +m_ref / 1000

    c_m = abs(delta_h / 1000)
    c_h1 = abs(m_ref.x / 1000)
    c_h2 = abs(m_ref.x / 1000)

    # -------------------------------------------------------------------------
    # 3) Einzelbeiträge zur Standardunsicherheit von P
    # -------------------------------------------------------------------------
    # u_i(P) = |c_i| * u(x_i)

    u_m = c_m * m_ref.u
    u_h1 = c_h1 * h1.u
    u_h2 = c_h2 * h2.u

    # -------------------------------------------------------------------------
    # 4) Kombinierte Standardunsicherheit
    # -------------------------------------------------------------------------
    # Unabhängige Unsicherheiten werden quadratisch addiert.

    u_total = (u_m**2 + u_h1**2 + u_h2**2) ** 0.5

    # Erweiterte Unsicherheit
    U_total = k * u_total

    # -------------------------------------------------------------------------
    # 5) Prozentuale Varianzanteile
    # -------------------------------------------------------------------------
    # Wichtig:
    # Die Anteile werden über die Quadrate berechnet.
    # Also u_i² / u_total².

    if u_total != 0:
        share_m = u_m**2 / u_total**2 * 100
        share_h1 = u_h1**2 / u_total**2 * 100
        share_h2 = u_h2**2 / u_total**2 * 100
    else:
        share_m = share_h1 = share_h2 = 0

    # -------------------------------------------------------------------------
    # 6) Ausgabe
    # -------------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("Beitragsanalyse für P = m_ref * (h2 - h1) / 1000")

    print("\nNominale Werte:")
    print(f"m_ref   = {m_ref.x:.8g} kg/s")
    print(f"h1      = {h1.x:.8g} J/kg")
    print(f"h2      = {h2.x:.8g} J/kg")
    print(f"delta_h = {delta_h:.8g} J/kg")
    print(f"P       = {P:.8g} kW")

    print("\nStandardunsicherheiten der Eingangsgrößen:")
    print(f"u(m_ref) = {m_ref.u:.8g} kg/s")
    print(f"u(h1)    = {h1.u:.8g} J/kg")
    print(f"u(h2)    = {h2.u:.8g} J/kg")

    print("\nSensitivitätskoeffizienten:")
    print(f"|dP/dm_ref| = {c_m:.8g} kW / (kg/s)")
    print(f"|dP/dh1|    = {c_h1:.8g} kW / (J/kg)")
    print(f"|dP/dh2|    = {c_h2:.8g} kW / (J/kg)")

    print("\nEinzelbeiträge zur Standardunsicherheit von P:")
    print(f"u_m_ref(P) = {u_m:.8g} kW")
    print(f"u_h1(P)    = {u_h1:.8g} kW")
    print(f"u_h2(P)    = {u_h2:.8g} kW")

    print("\nKombinierte Unsicherheit:")
    print(f"u_total      = {u_total:.8g} kW")
    print(f"U_total k={k} = {U_total:.8g} kW")
    print(f"relative U   = {abs(U_total / P) * 100:.4f} %")

    print("\nAnteile an der Gesamtvarianz:")
    print(f"m_ref: {share_m:.1f} %")
    print(f"h1:    {share_h1:.1f} %")
    print(f"h2:    {share_h2:.1f} %")

    print("\nInterpretation:")
    shares = {
        "m_ref": share_m,
        "h1": share_h1,
        "h2": share_h2,
    }
    dominant = max(shares, key=shares.get)

    if dominant == "m_ref":
        print("Dominanter Beitrag: Massenstrommessung.")
        print("Die Leistung ist direkt proportional zum Massenstrom.")
    elif dominant == "h1":
        print("Dominanter Beitrag: Enthalpie h1.")
        print("Die Unsicherheit aus Zustand 1 wirkt stark auf die Enthalpiedifferenz.")
    elif dominant == "h2":
        print("Dominanter Beitrag: Enthalpie h2.")
        print("Die Unsicherheit aus Zustand 2 wirkt stark auf die Enthalpiedifferenz.")


def first_kältekreis_example():
    """
    Erste Beispielrechnung für einen simulierten Betriebspunkt.

    WICHTIG:
    - Werte sind Platzhalter bzw. Beispielwerte.
    - Später durch deine eSimulation-Werte ersetzen.
    - EL3154 und ELM3244 sind offenbar nicht im Package hinterlegt.
      Daher wird hier vorläufig EL3182 bzw. EL3202_0010 als Näherung genutzt.
    """

    # -------------------------------------------------------------------------
    # 1) Simulierter Betriebspunkt
    # -------------------------------------------------------------------------
    # Beispielwerte bitte später durch echte eSimulation-Werte ersetzen.

    # Zustand 1: Verdichtereintritt / Saugleitung
    T1_values = [5 + 273.15]          # K
    p1_values = [5.0 * 1e5]           # Pa

    # Zustand 2: Verdichteraustritt / Heißgas
    T2_values = [80 + 273.15]         # K
    p2_values = [20.0 * 1e5]          # Pa

    # Kältemittelmassenstrom
    m_ref_values = [35 / 1000]        # kg/s

    # Messbereiche
    # Drucksensor: hier 25 bar FSO
    # Falls ihr 40 bar nutzt: 40 * 1e5 und pressure_type auf p40 setzen.
    pressure_full_scale = 25 * 1e5    # Pa
    pressure_type = "ICS_Schneider_IMP331-p25"

    # Massenstrom-Ausgangsbereich
    massflow_full_scale = 40 / 1000   # kg/s, Beispiel: 40 g/s

    # -------------------------------------------------------------------------
    # 2) Temperaturmessungen
    # -------------------------------------------------------------------------
    # Falls eure Mantelwiderstandsthermometer Klasse A sind: PT100A
    # Falls 1/10 DIN: PT100B1by10 verwenden.

    T1 = Measurement(
        values=T1_values,
        sensor=Temperature(type="PT100A", values=T1_values),
        terminal=EL3202_0010(type="PT100")
    )()

    T2 = Measurement(
        values=T2_values,
        sensor=Temperature(type="PT100A", values=T2_values),
        terminal=EL3202_0010(type="PT100")
    )()

    # -------------------------------------------------------------------------
    # 3) Druckmessungen
    # -------------------------------------------------------------------------
    # Hinweis:
    # Prüfen, ob das Package-Modell ICS_Schneider_IMP331-p25 wirklich die von euch
    # gewählte 0,1-%-FSO-Option abbildet. Es könnte auch die Standardvariante sein.

    p1 = Measurement(
        values=p1_values,
        sensor=Pressure(type=pressure_type, full_scale=pressure_full_scale),
        terminal=EL3182(full_scale=pressure_full_scale)
    )()

    p2 = Measurement(
        values=p2_values,
        sensor=Pressure(type=pressure_type, full_scale=pressure_full_scale),
        terminal=EL3182(full_scale=pressure_full_scale)
    )()

    # -------------------------------------------------------------------------
    # 4) Massenstrommessung
    # -------------------------------------------------------------------------
    # Package kennt laut deiner Suche 6400c_DN08 und 6400c.
    # Für die erste Rechnung DN08 verwenden, wenn das eurem Sensor entspricht.

    m_ref = Measurement(
        values=m_ref_values,
        sensor=Krohne_OptiMass(
            type="6400c_DN08",
            values=m_ref_values,
            pressure=p1.x,
            temperature=T1.x
        ),
        terminal=EL3182(full_scale=massflow_full_scale)
    )()

    # -------------------------------------------------------------------------
    # 5) Enthalpien aus T und p berechnen
    # -------------------------------------------------------------------------
    # Das entspricht deinem vorhandenen Pyfluids-Beispiel.

    h1 = PyfluidsEnthalpy(temperature=T1, pressure=p1, fluid="Propane")()
    h2 = PyfluidsEnthalpy(temperature=T2, pressure=p2, fluid="Propane")()

    # -------------------------------------------------------------------------
    # 6) Kältemittelseitige Leistung
    # -------------------------------------------------------------------------
    # Vorzeichen hängt von deiner Bilanzrichtung ab.
    # Hier: Enthalpieerhöhung von Zustand 1 nach Zustand 2.
    #
    # Wichtig:
    # P_therm wird hier mit k=2 erweitert.
    # Danach ist P_therm.u die erweiterte Unsicherheit U.

    P_therm = expand_uncertainty((m_ref * (h2 - h1)) / 1000, k=2)  # kW

    # -------------------------------------------------------------------------
    # 7) Ausgabe Einzelgrößen
    # -------------------------------------------------------------------------

    show("T1", T1)
    show("T2", T2)
    show("p1", p1)
    show("p2", p2)
    show("m_ref", m_ref)
    show("h1", h1)
    show("h2", h2)
    show("P_therm = m_ref * (h2 - h1)", P_therm)

    # -------------------------------------------------------------------------
    # 8) Intervall für Endergebnis
    # -------------------------------------------------------------------------

    print_interval("P_therm = m_ref * (h2 - h1)", P_therm, "kW")

    # -------------------------------------------------------------------------
    # 9) Beitragsanalyse
    # -------------------------------------------------------------------------
    # Hier wird analysiert, ob m_ref, h1 oder h2 dominant ist.

    contribution_analysis_power(m_ref, h1, h2, k=2)


def fallback_without_pyfluids():
    """
    Fallback, falls Pyfluids nicht läuft.

    Dann wird eine angenommene Enthalpiedifferenz verwendet.
    Diese Rechnung ist nicht final, aber gut, um Sensor + Klemme + Massenstrom
    zu testen.
    """

    T_ref_values = [5 + 273.15]
    p_ref_values = [5.0 * 1e5]
    m_ref_values = [35 / 1000]

    pressure_full_scale = 25 * 1e5
    massflow_full_scale = 40 / 1000

    T_ref = Measurement(
        values=T_ref_values,
        sensor=Temperature(type="PT100A", values=T_ref_values),
        terminal=EL3202_0010(type="PT100")
    )()

    p_ref = Measurement(
        values=p_ref_values,
        sensor=Pressure(type="ICS_Schneider_IMP331-p25", full_scale=pressure_full_scale),
        terminal=EL3182(full_scale=pressure_full_scale)
    )()

    m_ref = Measurement(
        values=m_ref_values,
        sensor=Krohne_OptiMass(
            type="6400c_DN08",
            values=m_ref_values,
            pressure=p_ref.x,
            temperature=T_ref.x
        ),
        terminal=EL3182(full_scale=massflow_full_scale)
    )()

    # Angenommene Enthalpiedifferenz, z. B. aus Simulation:
    delta_h = 180_000  # J/kg

    Q_dot = expand_uncertainty((m_ref * delta_h) / 1000, k=2)  # kW

    show("T_ref", T_ref)
    show("p_ref", p_ref)
    show("m_ref", m_ref)
    show("Q_dot = m_ref * delta_h", Q_dot)
    print_interval("Q_dot = m_ref * delta_h", Q_dot, "kW")

    print("\nHinweis:")
    print("Diese Fallback-Rechnung enthält nur die Unsicherheit des Massenstroms.")
    print("Die Unsicherheit von delta_h ist hier noch nicht enthalten.")


if __name__ == "__main__":
    try:
        first_kältekreis_example()
    except Exception as e:
        print("\nFEHLER IN DER PYFLUIDS-/ENTHALPIE-RECHNUNG:")
        print(type(e).__name__, e)
        print("\nStarte Fallback-Rechnung ohne Pyfluids...\n")
        fallback_without_pyfluids()