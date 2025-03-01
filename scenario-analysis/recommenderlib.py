import json
from typing import Any

import autogluon.tabular
import pandas as pd

import dynmodel
import events2missions

# for Jep (https://github.com/ninia/jep/issues/260#issuecomment-643023150)
RECOMMENDER_FORCING_REACTION = None


def make_greeting():
    return 'hello from recommenderlib'


def convert_map_event_to_record_input_without_is_stop(map_event: dict[str, str]) -> dict[str, Any]:
    df_missions = events2missions.convert_map_event_to_df_missions(map_event)
    df_started = dynmodel.prepare_forcing_reaction_started(df_missions)
    df_inout = dynmodel.select_columns_input_output(df_started, are_columns_required=False)

    record_input_without_is_stop, = df_inout.to_dict('records')
    return record_input_without_is_stop


def test_convert_map_event_to_record_input_without_is_stop() -> None:
    with open('data/map_event.json') as file:
        map_event = json.load(file)
    record = convert_map_event_to_record_input_without_is_stop(map_event)

    df_test_input = dynmodel.load_df_test_input()
    reference = {col: True for col in df_test_input.columns if col.startswith('(in) ')}
    del reference['(in) event_isStop']

    cols_common = [col for col in reference if col in record]
    cols_missing = [col for col in reference if col not in record]
    cols_redundant = [col for col in record if col not in reference]

    assert not cols_missing, cols_missing
    assert not cols_redundant, cols_redundant

    print(record)


class ForcingReactionRecommender:
    def __init__(self, path_collision: str) -> None:
        self.predictor_collision = autogluon.tabular.TabularPredictor.load(path_collision)

    def calculate_score(self, prediction_collisions):
        return -prediction_collisions

    def is_stop_recommended(self, record_input_without_is_stop: dict[str, Any]) -> bool:
        col_is_stop = '(in) event_isStop'
        assert col_is_stop not in record_input_without_is_stop, record_input_without_is_stop[col_is_stop]

        df = pd.DataFrame.from_records([
            {col_is_stop: x, **record_input_without_is_stop}
            for x in (0, 1)
        ])
        predictions_collisions = self.predictor_collision.predict(df)

        score_0, score_1 = [self.calculate_score(prediction_collisions)
                            for prediction_collisions in predictions_collisions]
        print(f'{score_0=}, {score_1=}')

        return score_1 > score_0


def initialize_recommender_forcing_reaction():
    global RECOMMENDER_FORCING_REACTION
    RECOMMENDER_FORCING_REACTION = ForcingReactionRecommender(
        path_collision='scenario-analysis/AutogluonModels/ag-20250227_141815'
    )


def use_recommender_forcing_reaction(map_event: dict[str, str]) -> bool:
    assert isinstance(RECOMMENDER_FORCING_REACTION, ForcingReactionRecommender), RECOMMENDER_FORCING_REACTION
    record = convert_map_event_to_record_input_without_is_stop(map_event)
    return RECOMMENDER_FORCING_REACTION.is_stop_recommended(record)


if __name__ == '__main__':
    test_convert_map_event_to_record_input_without_is_stop()