from typing import List, Union
import csv
import pandas as pd
from pathlib import Path

from vclibpy.components.compressors import TenCoefficientCompressor
from vclibpy import media, Inputs

try:
    from sklearn.linear_model import LinearRegression
    from sklearn.preprocessing import PolynomialFeatures
except ImportError as err:
    raise ImportError("You have to install sklearn to use this regression tool")


def create_regression_data(
        T_con: List[float],
        T_eva: List[float],
        n: List[float],
        compressor: TenCoefficientCompressor,
        save_path: Union[Path, str],
        fluid: str = None,
):
    """
    Performs multidimensional linear regression to create compressor learning data.

    Args:
        T_con (List[float]): Condensing temperatures in K
        T_eva (List[float]): Evaporation temperatures in K
        n (List[float]): Compressor speeds from 0 to 1
        save_path (str): Path to the folder where the newly created table will be saved.
        compressor (TenCoefficientCompressor):
            Instance to create regression for.
            If med_prop is not set, the given fluid will be used.
        fluid (str): Type of refrigerant. Only used if not already specified in compressor.

    Returns:
        List[float]: A list containing the regression parameters [P0, P1, ..., P9].

    Raises:
        ValueError: If any specified variable column is not present in the DataFrame.
    """
    if compressor.med_prop is None:
        med_prop_class, med_prop_kwargs = media.get_global_med_prop_and_kwargs()
        med_prop = med_prop_class(fluid_name=fluid, **med_prop_kwargs)
        compressor.med_prop = med_prop
    else:
        med_prop = compressor.med_prop

    keywords = {
        "eta_is": "Isentropic Efficiency(-)",
        "lambda_h": "Volumentric Efficiency(-)",
        "eta_mech": "Mechanical Efficiency(-)"
    }

    variables = ["eta_is", "eta_mech", "lambda_h"]

    tuples_for_cols = [("", "n")]
    for _variable in variables:
        for _n in n:
            tuples_for_cols.append((keywords[_variable], compressor.get_n_absolute(_n)))
    # tuples_for_cols:
    #     eta_is   eta_is   eta_is   lambda_h    lambda_h    lambda_h    eta_mech    ...
    # n   30      60      90      30          60          90          30          ...
    cols = pd.MultiIndex.from_tuples(tuples_for_cols)
    final_df = pd.DataFrame(
        data={cols[0]: ["P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10"]},
        columns=cols
    )
    # final_df: column names are tuples (tuples_for_cols).
    # First column is filled with P1, P2, ...

    # for-loop for multiple types(eta_is, eta_mech, etc)
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
                        state_1 = med_prop.calc_state("PT", p1, (T_eva[i] + compressor.T_sh))
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

                        if _variable == "eta_is":
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
    df_new = final_df.copy()
    df_new.columns = df_new.columns.get_level_values(0)  # Use only first level
    df_new.loc[-1] = final_df.columns.get_level_values(1)
    final_df = df_new.sort_index().reset_index(drop=True)
    final_df.to_csv(save_path, index=False)


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
