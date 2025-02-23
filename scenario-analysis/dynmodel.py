import ast
import os
import subprocess
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
# RUNNAME = '20250214_172108'
# RUNNAME = '20250215_120817'
# RUNNAME = '20250217_230154_halfway'
# RUNNAME = '20250218_115818_halfway'
# RUNNAME = '20250218_115818'
# RUNNAME = '20250219_095235_halfway'
# RUNNAME = '20250219_192637_halfway'
# RUNNAME = '20250220_094622_halfway'
# RUNNAME = '20250220_094622'
RUNNAME = '20250221_172706'

RUNDIR = f'{RUNDIRS}/{RUNNAME}'
# DIRECTORY_DATA = f'data/{RUNNAME}'
# os.makedirs(DIRECTORY_DATA, exist_ok=True)

FILENAME_MISSIONS_ALL = f'{RUNDIR}/missions_all.csv'


# Dictionary to map Map IDs to the number of OPs
MAP_TO_OPS = {
    1: 2, 6: 2, 10: 2,  # Maps with 2 OPs
    2: 1, 3: 1, 4: 1, 5: 1, 7: 1, 8: 1, 9: 1,  # Maps with 1 OP
}

PARAMSETS_POD_NEXT = [
    {'pod_prefix': pod_prefix,
     'next_n': next_n}
    for pod_prefix in ("POD C", "POD Df")
    for next_n in (20, 50, 100, 200, None)
]


def add_derived_columns(df_orig, *, columns_params = None, columns_configuration = None):
    df_id_pre = df_orig['Scenario ID'].str.extract(r'^(?P<filename>.*?)(?P<params>, .*)$', expand=True)

    df_params = pd.concat([
        df_id_pre['params'].str.extract(r', passhum (?P<passhum>0|1)(?:$|(?=,))', expand=True).astype(int).astype(bool),
        df_id_pre['params'].str.extract(r', slowness (?P<slowness>[^,]+)', expand=True),
        df_id_pre['params'].str.extract(r', forcing (?P<forcing>[^,]+)', expand=True),
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
    is_try = True

    dirs = []
    for name in os.listdir(RUNDIR):
        dir_scenario = f'{RUNDIR}/{name}'
        if os.path.isdir(dir_scenario):
            dirs.append(dir_scenario)

    dfs = []
    for dir_scenario in tqdm.tqdm(dirs):
        prepare = lambda: prepare_missions(f'{dir_scenario}/events.tsv',
                                           f'{dir_scenario}/missions.csv')
        if not is_try:
            df = prepare()
        else:
            try:
                df = prepare()
            except Exception as e:
                print(dir_scenario, e)
                subprocess.run(['sed', '-n', '/^Exception in /,/^> Task :run/p', f'{dir_scenario}/scenario.log'],
                               check=True)
                continue

        dfs.append(df)

    df_all = pd.concat(dfs)
    df_all = df_all.loc[:, ['Scenario ID'] + [col for col in df_all.columns if col != 'Scenario ID']]
    return df_all


def save_missions_all(df_all: pd.DataFrame) -> None:
    df_all.to_csv(FILENAME_MISSIONS_ALL, index=False)


def load_missions_all() -> pd.DataFrame:
    return pd.read_csv(FILENAME_MISSIONS_ALL, low_memory=False)


def load_or_prepare_missions_all() -> pd.DataFrame:
    if os.path.exists(FILENAME_MISSIONS_ALL):
        return load_missions_all()

    df_all = prepare_missions_all()
    save_missions_all(df_all)
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


def make_name_column_pod_next(pod_prefix: str, next_n: int | None) -> str:
    return f"{pod_prefix} " + (f'next {next_n}' if next_n is not None else 'CS segment with human')


def add_column_pod_next(df: pd.DataFrame, pod_prefix: str, next_n: int | None) -> None:
    if pod_prefix.startswith("POD C"):
        source_col = "event_linearizationC"
    elif pod_prefix.startswith("POD Df"):
        source_col = "event_linearizationDf"
    else:
        raise ValueError("pod_prefix must start with 'POD C' or 'POD Df'.")

    new_col = make_name_column_pod_next(pod_prefix, next_n)

    def process_row(row: pd.Series) -> float:
        pod_list = row[source_col]

        i = row["V: path index"]
        if pd.isna(i):
            i = 0
        else:
            i = int(i)

        total_poses = int(row["V: no. poses"])

        # Pad pod_list with zeros if necessary.
        if len(pod_list) < total_poses:
            pod_list = pod_list + [0] * (total_poses - len(pod_list))

        # Check that the index i exists.
        if i >= len(pod_list):
            raise IndexError(
                f"Index {i} does not exist in {source_col} (length {len(pod_list)})."
            )

        if next_n is not None:
            i_start = i + 1
            i_end = i + next_n
        else:
            i_start = i + int(row['event_indicesToCS'])
            i_end = i + int(row['event_indicesToCSEnd'])

        next_slice = pod_list[i_start : i_end + 1]
        return 0 if not next_slice else sum(next_slice) / len(next_slice)

    # Apply the computation row-wise.
    df[new_col] = df.apply(process_row, axis=1)


def add_indices_human_to_cs(df: pd.DataFrame, is_to_end: bool) -> None:
    def process_row(row: pd.Series) -> float:
        i = row["V0: path index"]
        if pd.isna(i):
            i = 0
        else:
            i = int(i)

        if is_to_end:
            i_end = int(row["event_te1End"])
            di = i_end - i
            assert di >= 0
        else:
            i_start = int(row['event_te1Start'])
            di = max(0, i_start - i)

        return di

    # Apply the computation row-wise.
    df['indicesHumanToCS' + ('End' if is_to_end else '')] = df.apply(process_row, axis=1)


def add_columns_pod_next(df: pd.DataFrame) -> None:
    for col in df.columns:
        if col.startswith('event_linearization'):
            df.loc[:, col] = df[col].apply(lambda x: ast.literal_eval(x) if pd.notna(x) else x)

    for paramset in PARAMSETS_POD_NEXT:
        add_column_pod_next(df, **paramset)


def prepare_forcing_reaction_started(df: pd.DataFrame) -> pd.DataFrame:
    df = df[df['event_type'] == 'ForcingReactionStarted']

    df = add_derived_columns(df)
    df = add_vcurr_columns(df)
    add_columns_pod_next(df)
    for is_to_end in False, True:
        add_indices_human_to_cs(df, is_to_end=is_to_end)

    show(df[pd.isna(df['V: v_current'])], 'without v_current')
    return df


def series2values(series: pd.Series) -> np.ndarray:
    col = series.name
    assert isinstance(col, str)
    if col.endswith(': v_current'):
        return series.replace("~0.0", 0.0).astype(float).values
    if col.endswith(': distance ToCP, m'):
        return series.fillna(-1).astype(float).values

    dtype = series.dtype
    if dtype == 'bool':
        return series.astype('int').values
    if dtype in ('int64', 'float64'):
        return series.values
    if dtype == 'object':
        return series.astype('category').cat.codes
    raise TypeError(f'{dtype} is not supported')


def select_columns_input_output(df: pd.DataFrame) -> pd.DataFrame:
    columns_input = [
        'event_isStop',

        # Map features:
        'No. of OPs',
        'are_bridges',

        # Scenario parameters:
        'slowness',

        # Current values:

        'V: v_current',
        'V0: v_current',

        'V: POD',
        'V0: POD',

        'event_distanceToCS',
        'event_distanceToCSEnd',

        'event_distanceHumanToCP',  # Note: 'V0: distance ToCP, m' is computed BEFORE the forcing.
        'indicesHumanToCS',
        'indicesHumanToCSEnd',
    ]

    for paramset in PARAMSETS_POD_NEXT:
        columns_input.append(make_name_column_pod_next(**paramset))

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


def convert_df_started_to_train_test(df_started: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    df = select_columns_input_output(df_started)

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


def run_models(df_started: pd.DataFrame) -> pd.DataFrame:
    df_train, df_test = convert_df_started_to_train_test(df_started)
    df_predictions_regression = run_regression(df_train, df_test)
    show(df_predictions_regression, 'df_predictions_regression')
    evaluate_and_plot_column(df_test, df_predictions_regression, '(out) MajorCollisionFromMinor before forcing ends')

    predictors, df_predictions_autogluon = run_autogluon(df_train, df_test)
    return df_predictions_autogluon


def main():
    df_all = load_or_prepare_missions_all()
    df_started = prepare_forcing_reaction_started(df_all)
    run_models(df_started)


if __name__ == '__main__':
    main()