import logging
from typing import Union
from pathlib import Path

import numpy as np
import sdf

try:
    import plotly.graph_objects as go
    import dash
    from dash import dcc, html
    from dash.dependencies import Input, Output
except ImportError as err:
    raise ImportError(
        "Could not import optional requirement, make sure "
        "to install plotly and dash before using this module"
    ) from err

logger = logging.getLogger(__name__)


def load_data(filepath: Union[Path, str]):
    """
    Small helper function to load data into a format
    required for the dash-app
    """
    dataset = sdf.load(str(filepath))
    for flowsheet in dataset.groups:
        for fluid in flowsheet.groups:
            datasets_sdf = fluid.datasets
            scales_sdf = fluid.datasets[0].scales
            break
    data = {}
    scales = {}
    units = {}
    for scale_sdf in scales_sdf:
        scales[scale_sdf.name] = scale_sdf.data
        units[scale_sdf.name] = scale_sdf.unit
    for data_sdf in datasets_sdf:
        data[data_sdf.name] = data_sdf.data
        units[data_sdf.name] = data_sdf.unit
    return data, scales, units


def create_app(filepath: Union[Path, str]):
    """
    This function creates the app which enables interactive visualization
    """
    data, scales, units = load_data(filepath)

    app = dash.Dash(__name__)

    app.layout = html.Div([
        html.Div([
            html.Div([
                html.H3("Plot Type"),
                dcc.RadioItems(
                    id='plot-type',
                    options=[
                        {'label': '2D Plot', 'value': '2d'},
                        {'label': '3D Plot', 'value': '3d'}
                    ],
                    value='2d'
                ),
            ], style={'width': '30%', 'display': 'inline-block'}),
            html.Div([
                html.H3("Variables Selection"),
                html.Label("Variable to Plot:"),
                dcc.Dropdown(
                    id='plot-variable',
                    options=[{'label': col, 'value': col} for col in data],
                    value=list(data.keys())[0],
                    placeholder="Select variable to plot"
                ),
                html.Label("X Axis:"),
                dcc.Dropdown(
                    id='x-axis',
                    options=[{'label': col, 'value': col} for col in scales],
                    value=list(scales.keys())[0],
                    placeholder="Select X axis"
                ),
                html.Label("Y Axis:"),
                dcc.Dropdown(
                    id='y-axis',
                    options=[{'label': col, 'value': col} for col in scales],
                    value=list(scales.keys())[1],
                    placeholder="Select Y axis"
                )
            ], style={'width': '30%', 'display': 'inline-block'}),

            html.Div([
                html.H3("Fixed Variables"),
                *[html.Div([
                    html.Label(f"{scale}:"),
                    dcc.Dropdown(
                        id=f'constant-{scale}',
                        value=scale_data.min(),
                        options=[{"label": value, "value": value} for value in scale_data]
                    )
                ]) for scale, scale_data in scales.items()],
            ], style={'width': '30%', 'display': 'inline-block'}),
        ]),
        dcc.Graph(id='sdf-plot', style={'height': '600px', 'width': '100%'})
    ])

    @app.callback(
        Output('y-axis', 'style'),
        Input('plot-type', 'value')
    )
    def toggle_z_axis(plot_type):
        if plot_type == '3d':
            return {'display': 'block'}
        return {'display': 'none'}

    @app.callback(
        Output('sdf-plot', 'figure'),
        [Input('plot-type', 'value'),
         Input('plot-variable', 'value'),
         Input('x-axis', 'value'),
         Input('y-axis', 'value')] +
        [Input(f'constant-{scale}', 'value') for scale in scales]
    )
    def update_graph(plot_type, plot_variable, x_axis, y_axis, *slider_values):
        # Create a dictionary of fixed values
        scale_names = list(scales.keys())
        fixed_values = dict(zip(scale_names, slider_values))
        masks = []
        axes_to_consider = [x_axis, y_axis] if plot_type == "3d" else [x_axis]
        # Filter data based on fixed values
        for scale, value in fixed_values.items():
            if scale not in axes_to_consider:
                idx = np.where(scales[scale] == value)[0][0]
            else:
                idx = slice(None)  # All true
            masks.append(idx)

        data_to_plot = data[plot_variable][tuple(masks)]

        if plot_type == '2d':
            logger.info("Updating %s in %s with x-axis='%s' and constant masks %s. Data min %s, max %s",
                        plot_variable, plot_type, x_axis, masks, data_to_plot.min(), data_to_plot.max())

            fig = go.Figure()

            # Simple 2D line plot: variable vs scale
            fig.add_trace(go.Scatter(
                x=scales[x_axis],
                y=data_to_plot,
                mode='lines+markers',
                name='Variable vs Scale'
            ))

            fig.update_layout(
                title='2D View: Variable vs Scale',
                xaxis_title=f'{x_axis} in {units[x_axis]}',
                yaxis_title=f'{plot_variable} in {units[plot_variable]}'
            )

        else:  # 3D plot
            logger.info("Updating %s in %s with x-axis='%s' and y-axis='%s' and constant masks %s. Data min %s, max %s",
                        plot_variable, plot_type, x_axis, y_axis, masks, data_to_plot.min(), data_to_plot.max())
            # Create meshgrid for x and y scales
            X, Y = np.meshgrid(scales[x_axis], scales[y_axis])
            if scale_names.index(y_axis) > scale_names.index(x_axis):
                data_to_plot = data_to_plot.transpose()
            fig = go.Figure()
            logger.info(f"X shape: {X.shape}, Y shape: {Y.shape}, data shape: {data_to_plot.shape}")
            # Optionally add surface plot
            fig.add_trace(go.Surface(
                x=X,
                y=Y,
                z=data_to_plot,
                colorscale='Viridis'
            ))

            fig.update_layout(
                title='3D View: Variable vs Scales',
                scene=dict(
                    xaxis_title=f'{x_axis} in {units[x_axis]}',
                    yaxis_title=f'{y_axis} in {units[y_axis]}',
                    zaxis_title=f'{plot_variable} in {units[plot_variable]}'
                )
            )

        return fig

    return app


if __name__ == '__main__':
    logging.basicConfig(level="INFO")
    FILE_PATH = "map_to_plot.sdf"
    APP = create_app(FILE_PATH)
    APP.run_server(debug=True)
