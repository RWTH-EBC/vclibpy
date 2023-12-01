from typing import List
import csv
import pandas as pd

from vclibpy.components.compressors import TenCoefficientCompressor
from vclibpy import media, Inputs

try:
    from sklearn.linear_model import LinearRegression
    from sklearn.preprocessing import PolynomialFeatures
    from xlsxwriter.workbook import Workbook
except ImportError as err:
    raise ImportError("You have to install xlsxwriter and "
                      "sklearn to use this regression tool")


def create_regression_data(
        variables: List[str],
        T_con: List[float], T_eva: List[float], n: List[float],
        T_sh: float, T_sc: float, n_max: int, V_h: float, fluid: str, datasheet: str,
        capacity_definition: str, assumed_eta_mech: int,
        folder_path: str):
    """
    Performs multidimensional linear regression to create compressor learning data.

    Args:
        variables: (List[str]):
            Variable names to create regressions for.
            Options are: eta_s, lambda_h, and eta_mech
        T_con (List[float]): Condensing temperatures in K
        T_eva (List[float]): Evaporation temperatures in K
        n (List[float]): Compressor speeds from 0 to 1
        T_sh (float): Superheat temperature.
        T_sc (float): Subcooling temperature.
        n_max (int): Maximum compressor speed.
        V_h (float): Compressor volume.
        fluid (str): Type of refrigerant.
        datasheet (str): Path to the modified datasheet.
        capacity_definition (str): Definition of compressor capacity (e.g., "cooling").
        assumed_eta_mech (int): Assumed mechanical efficiency.
        folder_path (str): Path to the folder where the newly created table will be saved.

    Returns:
        List[float]: A list containing the regression parameters [P0, P1, ..., P9].

    Raises:
        ValueError: If any specified variable column is not present in the DataFrame.

    Example:
        >>> create_regression_data(11.1, 8.3, 120, 20.9e-6, "PROPANE", "path/to/datasheet.xlsx",
        ...                        "cooling", 1, 6, 5, 5, 5, 303.15, 243.15, "path/to/save", 3)
        [intercept, P1, P2, P3, P4, P5, P6, P7, P8, P9]
    """
    # create RefProp, fluid & compressor instance
    med_prop = media.CoolProp(fluid)

    compressor = TenCoefficientCompressor(
        N_max=n_max, V_h=V_h, T_sc=T_sc, T_sh=T_sh,
        capacity_definition=capacity_definition,
        assumed_eta_mech=assumed_eta_mech,
        datasheet=datasheet
    )

    compressor.med_prop = med_prop
    keywords = {
        "eta_s": "Isentropic Efficiency(-)",
        "lambda_h": "Volumentric Efficiency(-)",
        "eta_mech": "Mechanical Efficiency(-)"
    }

    tuples_for_cols = [("", "n")]
    for _variable in variables:
        for _n in n:
            tuples_for_cols.append((keywords[_variable], compressor.get_n_absolute(_n)))
    # tuples_for_cols:
    #     eta_s   eta_s   eta_s   lambda_h    lambda_h    lambda_h    eta_mech    ...
    # n   30      60      90      30          60          90          30          ...
    cols = pd.MultiIndex.from_tuples(tuples_for_cols)
    final_df = pd.DataFrame(
        data={cols[0]: ["P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10"]},
        columns=cols
    )
    # final_df: column names are tuples (tuples_for_cols).
    # First column is filled with P1, P2, ...

    # for-loop for multiple types(eta_s, eta_mech, etc)
    for m, _variable in enumerate(variables):
        for k, _n in enumerate(n):  # for-loop for multiple rotation speeds
            T_eva_list = []
            T_con_list = []
            result_list = []
            # for-loop for multiple evaporating temperatures
            for i in range(len(T_eva)):
                # for-loop for multiple condensing temperatures
                for j in range(len(T_con)):
                    if T_eva[i] < T_con[j]:
                        p1 = med_prop.calc_state("TQ", T_eva[i], 1).p
                        state_1 = med_prop.calc_state("PT", p1, (T_eva[i] + T_sh))
                        compressor.state_inlet = state_1
                        p2 = med_prop.calc_state("TQ", T_con[j], 1).p
                        # The enthalpy and entropy of the outlet
                        # state do not matter, only the pressure:
                        # TODO: Enable calculation of get_lambda_h etc. with p2 only
                        state_2 = med_prop.calc_state("PS", p2, state_1.s)
                        compressor.state_outlet = state_2
                        T_eva_list.append(T_eva[i])
                        T_con_list.append(T_con[j])
                        inputs = Inputs(n=_n)

                        if _variable == "eta_s":
                            result_list.append(compressor.get_eta_isentropic(
                                p_outlet=p2, inputs=inputs)
                            )
                        elif _variable == "lambda_h":
                            result_list.append(compressor.get_lambda_h(inputs=inputs))
                        elif _variable == "eta_mech":
                            result_list.append(compressor.get_eta_mech(inputs=inputs))

            df = pd.DataFrame(
                data={"T_eva": T_eva_list,
                      "T_con": T_con_list,
                      _variable: result_list}
            )

            final_df[cols[m * len(n) + k + 1]] = create_regression_parameters(df, _variable)

    # dataframes with a double column header can't be saved as a
    # .xlsx yet, if index = False (NotImplementedError).
    # .xlsx format is necessary, because TenCoefficientCompressor.get_parameter()
    # expects a .xlsx as an input
    # --> workaround: save the dataframe as a .csv, read it again and save it as a .xlsx
    # TODO: Revert once this feature is in pandas.
    final_df.to_csv(folder_path + r"\regressions.csv", index=False)

    workbook = Workbook(folder_path + r"\regressions.xlsx")
    worksheet = workbook.add_worksheet()
    with open(folder_path + r"\regressions.csv", 'rt', encoding='utf8') as f:
        reader = csv.reader(f)
        for r, row in enumerate(reader):
            for c, col in enumerate(row):
                worksheet.write(r, c, col)
    workbook.close()


def create_regression_parameters(df: pd.DataFrame, variable: str):
    """
    Performs multidimensional linear regression to calculate
    ten coefficient regression parameters.

    Args:
        df (pd.DataFrame): The input DataFrame containing the necessary columns.
        variable (str): The column name for the dependent variable.

    Returns:
        List[float]: A list containing the ten regression parameters.

    Raises:
        ValueError: If the specified variable column is not present in the DataFrame.

    Example:
        >>> df = pd.DataFrame({'T_eva': [1, 2, 3], 'T_con': [4, 5, 6], 'X': [7, 8, 9]})
        >>> create_regression_parameters(df, 'X')
        [intercept, P1, P2, P3, P4, P5, P6, P7, P8, P9]
    """
    # extract the columns x, y und z
    x = df['T_eva'].values
    y = df['T_con'].values
    z = df[variable].values

    # define the features (x, y, x^2, xy, y^2, x^3, x^2y, xy^2, y^3)
    features = PolynomialFeatures(degree=3, include_bias=False)
    X = features.fit_transform(pd.concat([pd.DataFrame(x), pd.DataFrame(y)], axis=1))

    # Execute the multidimensional linear regression
    model = LinearRegression().fit(X, z)

    output = [model.intercept_, model.coef_[0], model.coef_[1], model.coef_[2],
              model.coef_[3], model.coef_[4],
              model.coef_[5], model.coef_[6], model.coef_[7], model.coef_[8]]
    # output = P1-P10
    return output
