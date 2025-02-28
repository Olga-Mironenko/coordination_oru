from typing import Any

import autogluon.tabular
import pandas as pd


def make_greeting():
    return 'hello from recommenderlib'


class ForcingReactionRecommender:
    def __init__(self, path_collision: str) -> None:
        self.predictor_collision = autogluon.tabular.TabularPredictor.load(path_collision)

    def calculate_score(self, prediction_collisions):
        return -prediction_collisions

    def is_stop_recommended(self, record_input_without_is_stop: dict[str, Any]) -> bool:
        df = pd.DataFrame.from_records([
            {'(in) event_isStop': x, **record_input_without_is_stop}
            for x in (0, 1)
        ])
        predictions_collisions = self.predictor_collision.predict(df)

        score_0, score_1 = [self.calculate_score(prediction_collisions)
                            for prediction_collisions in predictions_collisions]
        print(f'{score_0=}, {score_1=}')

        return score_1 > score_0