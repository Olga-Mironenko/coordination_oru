import os
import warnings
from typing import Any

import autogluon.tabular
import numpy as np
from IPython.display import display, HTML
import matplotlib.pyplot as plt
import pandas as pd
import sklearn.linear_model
import sklearn.metrics
import sklearn.model_selection
import tqdm

import events2missions

pd.options.display.max_columns = 100
pd.options.display.max_colwidth = 200

warnings.filterwarnings("ignore", message="Can't initialize NVML")

RUNDIRS = '../logs/rundirs'

# RUNNAME = '20250209_170442'
# RUNNAME = '20250214_172108_halfway'
RUNNAME = '20250214_172108'

RUNDIR = f'{RUNDIRS}/{RUNNAME}'
# DIRECTORY_DATA = f'data/{RUNNAME}'
# os.makedirs(DIRECTORY_DATA, exist_ok=True)


# Dictionary to map Map IDs to the number of OPs
MAP_TO_OPS = {
    1: 2, 6: 2, 10: 2,  # Maps with 2 OPs
    2: 1, 3: 1, 4: 1, 5: 1, 7: 1, 8: 1, 9: 1,  # Maps with 1 OP
}


def add_derived_columns(df_orig, *, columns_params = None, columns_configuration = None):
    df_id_pre = df_orig['Scenario ID'].str.extract(r'^(?P<filename>.*?)(?P<params>, .*)$', expand=True)

    df_params = pd.concat([
        df_id_pre['params'].str.extract(r', passhum (?P<passhum>0|1)\b', expand=True).astype(int).astype(bool),
        df_id_pre['params'].str.extract(r', slowness (?P<slowness>[^,]+)\b', expand=True),
        df_id_pre['params'].str.extract(r', forcing (?P<forcing>[^,]+)\b', expand=True),
    ], axis=1)
    if columns_params is not None:
        assert list(df_params.columns) == columns_params  # TODO: set `COLUMNS_PARAMS`

    df_id = pd.concat([
        df_id_pre[['filename']],
        df_id_pre['filename'].str.extract(r'(?P<dir_map>[^/]+)/(?P<basename_scenario>[^/]+)[.]json$', expand=True),
        df_id_pre['filename'].str.extract(r'/scenario(?P<i_map>\d+)-(?P<i_locations>\d+)[.]json$', expand=True).astype(
            int),
        df_params,
    ], axis=1).rename(columns={'i_locations': 'position'})
    df_id['No. of OPs'] = df_id['i_map'].map(MAP_TO_OPS)
    # IPython.display.display(df_id)

    df_id['slowness'] = df_id['slowness'].apply(lambda s: 'baseline' if s == 'no' else s)
    df_id['forcing'] = df_id['forcing'].apply(lambda s: 'baseline' if s == 'no' else s)
    df_id['filename_screenshot'] = "../map-generator/generated-maps/" + df_id['dir_map'] + '/screenshots/' + df_id[
        'basename_scenario'] + '.png'
    df_id['are_bridges'] = df_id['dir_map'].str.contains('with_bridges')
    if columns_configuration is not None:
        df_id['configuration'] = df_id[
            columns_configuration
        ].agg(
            lambda r: f'map {r['i_map']}, {"with" if r['are_bridges'] else "without"} bridges, pos.var. {r['position']}',
            axis=1
        )

    cols_reroutings = [col
                       for col in df_orig.columns
                       if col.endswith(('rerouting at parked / slow', 'reroutings at parked / slow'))]
    # print(f'{cols_reroutings=}')
    df = pd.concat(
        [
            df_id,
            df_orig['isCanPassFirstActive'].str.extract(
                r'^hum=(?P<isCanPassFirstHum>false|true), aut=(?P<isCanPassFirstAut>false|true)$', expand=True
            ).apply(lambda col: col == 'true'),
            *[
                df_orig[col].astype(str).str.extract(
                    r'^(?P<reroutingsAtParked>-|\d+) / (?P<reroutingsAtSlow>-|\d+)$', expand=True
                ).apply(lambda col: col.map(lambda x: np.nan if pd.isna(x) or x == '-' else int(x))).astype('Int64')
                for col in cols_reroutings
            ],
            df_orig,
        ],
        axis=1
    )
    return df


def prepare_missions(filename_events_tsv, filename_missions_csv):
    filename_script = './events2missions.py'
    if (
            not os.path.exists(filename_missions_csv) or
            os.path.getmtime(filename_missions_csv) < os.path.getmtime(filename_events_tsv) or
            os.path.getmtime(filename_missions_csv) < os.path.getmtime(filename_script)
    ):
        # print(filename_events_tsv)
        events2missions.convert(filename_events_tsv, filename_missions_csv)
    return pd.read_csv(filename_missions_csv)


def prepare_missions_all():
    dfs = []
    for name in tqdm.tqdm(sorted(os.listdir(RUNDIR))):
        dir_scenario = f'{RUNDIR}/{name}'
        if os.path.isdir(dir_scenario):
            df = prepare_missions(f'{dir_scenario}/events.tsv',
                                  f'{dir_scenario}/missions.csv')
            dfs.append(df)

    df_all = pd.concat(dfs)
    df_all = df_all.loc[:, ['Scenario ID'] + [col for col in df_all.columns if col != 'Scenario ID']]
    return df_all


def add_vcurr_columns(df: pd.DataFrame) -> pd.DataFrame:
    sep = ': '

    # Get all unique metric suffixes (e.g., 'speed', 'power')
    suffixes = {col.split(sep, maxsplit=1)[1]: True for col in df.columns if sep in col}

    # Build a list of dictionaries where each dict holds the Vcurr values for that row.
    new_data_list = []
    for idx, row in df.iterrows():
        robot_id = row['robotID']
        new_row = {}
        for suffix in suffixes:
            # Construct the source column name for this row
            source_col = f'V{robot_id}{sep}{suffix}'
            new_row[f'V{sep}{suffix}'] = row[source_col]
        new_data_list.append(new_row)

    # Convert the list of dictionaries into a DataFrame, ensuring the index matches df.
    new_df = pd.DataFrame(new_data_list, index=df.index)

    # Concatenate the new Vcurr columns with the original DataFrame.
    df = pd.concat([df, new_df], axis=1)

    return df


def prepare_forcing_reaction_started(df: pd.DataFrame) -> pd.DataFrame:
    df = df[df['event_type'] == 'ForcingReactionStarted']
    df = add_vcurr_columns(df)
    df = add_derived_columns(df)
    return df


def series2values(series: pd.Series) -> np.ndarray:
    dtype = series.dtype
    if dtype == 'bool':
        return series.astype('int').values
    if dtype in ('int64', 'float64'):
        return series.values
    if dtype == 'object':
        if series.name == 'V: v_current':
            return series.replace("~0.0", 0.0).astype(float).values
        return series.astype('category').cat.codes
    raise TypeError(f'{dtype} is not supported')


def select_columns_input_output(df: pd.DataFrame) -> pd.DataFrame:
    columns_input = [
        'event_isStop',
        'i_map',
        'No. of OPs',
        'V: v_current',
        'V0: v_current',
        'V: POD',
        'V0: POD',
        'event_distanceToCS',
    ]
    columns_output = [
        'MajorCollisionFromMinor before forcing ends',
    ]

    return pd.DataFrame({
        **{
            f'(in) {col}': series2values(df[col])
            for col in columns_input
        },
        **{
            f'(out) {col}': series2values(df[col])
            for col in columns_output
        },
    })


def show(obj, title=None):
    if title is not None:
        display(HTML(f"<h3>{title}</h3>"))
    display(obj)


def evaluate_and_plot_column(df_test, df_predictions, column):
    y_test_column = df_test[column]
    predictions_column = df_predictions[column]

    r2 = sklearn.metrics.r2_score(y_test_column, predictions_column)
    print(f"{column}:")
    print(f"- R^2 Score: {r2}")

    # Plot results for each output column
    fig = plt.figure(figsize=(10, 6))
    plt.scatter(y_test_column, predictions_column, color='blue', alpha=0.5)
    plt.plot([y_test_column.min(), y_test_column.max()], [y_test_column.min(), y_test_column.max()], 'k--', lw=2)
    plt.xlabel('Actual Values')
    plt.ylabel('Predicted Values')
    plt.title(f'Actual vs Predicted Values for {column}')
    plt.grid(True)
    plt.show()
    # save_and_show(fig, f'Actual_vs_Predicted_Values_{name}')


def convert_missions_all_into_dfs_model(df_all: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    df = prepare_forcing_reaction_started(df_all)
    show(df[pd.isna(df['V: v_current'])], 'bad')
    df = select_columns_input_output(df)

    df_train, df_test = sklearn.model_selection.train_test_split(df, test_size=0.2, random_state=1)
    show(df_train, 'df_train')
    show(df_test, 'df_test')

    return df_train, df_test


def split_df_to_X_y(df):
    columns_input_df = [col for col in df.columns if col.startswith('(in) ')]
    columns_output_df = [col for col in df.columns if col.startswith('(out) ')]
    assert set(columns_input_df) | set(columns_output_df) == set(df.columns)

    X = df[columns_input_df]
    y = df[columns_output_df]
    return X, y


def run_regression(df_train, df_test):
    X_train, y_train = split_df_to_X_y(df_train)
    X_test, y_test = split_df_to_X_y(df_test)

    model = sklearn.linear_model.LinearRegression()
    model.fit(X_train, y_train)
    ndarray_predictions = model.predict(X_test)
    df_predictions = pd.DataFrame(ndarray_predictions, columns=y_test.columns)
    return df_predictions


def run_autogluon(df_train: pd.DataFrame, df_test: pd.DataFrame) -> tuple[list[Any], pd.DataFrame]:
    X_train, y_train = split_df_to_X_y(df_train)
    X_test, y_test = split_df_to_X_y(df_test)

    # Train AutoGluon models
    predictors = []
    df_predictions = pd.DataFrame()
    columns_output = [col for col in df_train.columns if col.startswith('(out) ')]
    for column in columns_output:
        print(f'{column=}:')
        df_train_predictor = pd.concat([X_train, y_train[[column]]], axis=1)

        predictor = autogluon.tabular.TabularPredictor(
            label=column,
            eval_metric='r2',
            problem_type='regression',
        ).fit(
            df_train_predictor,
            presets='medium',
            hyperparameters={
                'GBM': {},  # LightGBM (TODO: something like `GBMLarge`)
                'XGB': {},  # XGBoost
                'RF': {},  # Random Forest
                'XT': {},  # Extra Trees
                # 'CAT': {},      # CatBoost, omitted if slow
                # 'NN': {},       # Neural net, if you want it
                # 'LR': {},       # Linear model
                # 'KNN': {},      # K-Nearest Neighbors
            },
        )
        predictors.append(predictor)

        df_predictions[column] = predictor.predict(X_test)

        # Leaderboard - Display a table of different models and their performance
        df_test_predictor = pd.concat([X_test, y_test[[column]]], axis=1)
        leaderboard = predictor.leaderboard(df_test_predictor, silent=True)
        show(leaderboard, f'Leaderboard for {column}')

    return predictors, df_predictions


def run_models(df_all):
    df_train, df_test = convert_missions_all_into_dfs_model(df_all)
    df_predictions_regression = run_regression(df_train, df_test)
    show(df_predictions_regression, 'df_predictions_regression')
    evaluate_and_plot_column(df_test, df_predictions_regression, '(out) MajorCollisionFromMinor before forcing ends')

    predictors, df_predictions_autogluon = run_autogluon(df_train, df_test)
    return df_predictions_autogluon


def main():
    prepare_missions_all()


if __name__ == '__main__':
    main()