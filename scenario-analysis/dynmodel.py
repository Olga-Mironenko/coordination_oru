import os
import warnings

import autogluon.tabular
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

RUNNAME = '20250209_170442'

RUNDIR = f'{RUNDIRS}/{RUNNAME}'
DIRECTORY_DATA = f'data/{RUNNAME}'
os.makedirs(DIRECTORY_DATA, exist_ok=True)


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
    for name in tqdm.tqdm(sorted(os.listdir(RUNDIR))):
        dir_scenario = f'{RUNDIR}/{name}'
        if os.path.isdir(dir_scenario):
            prepare_missions(f'{dir_scenario}/events.tsv',
                             f'{dir_scenario}/missions.csv')


def main():
    prepare_missions_all()


if __name__ == '__main__':
    main()