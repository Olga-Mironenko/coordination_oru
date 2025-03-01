#!/usr/bin/env python3
"""
This module reads a TSV file containing events and converts it into a
dictionary mapping robotID to missions. Each mission is defined as the
sequence of rows (from the TSV) starting with an event of type
'MissionStarted' and ending with an event of type 'MissionFinished'.
In the output, the 'event' field is a dictionary rather than a JSON string.
"""
import io
import json
import shutil
import sys
from typing import Any, Dict, List

import pandas as pd

# Type aliases for clarity.
Row = Dict[str, Any]
Mission = List[Row]
MissionsDict = Dict[str, List[Mission]]

ID_HUMAN = 0
WIDTH_CONSOLE = shutil.get_terminal_size().columns


def sort_df(df: pd.DataFrame):
    # --- Step 1. Custom sort for events with the same robotID, missionID and secondsVirtual ---
    # Define a helper function to assign an order value to each event type.
    def event_order(event: Dict[str, Any]) -> int:
        # Any event not explicitly listed will get order 0.
        mapping = {
            et: i_et
            for i_et, et in enumerate(
                (
                    "ForcingReactionFinished",
                    "ForcingFinished",
                    "MissionFinished",

                    "MissionStarted",
                    "ForcingStarted",
                    "ForcingReactionStarted",
                ),
                1
            )
        }
        et = event['type']
        return mapping.get(et, 0)

    # Create an auxiliary column for sorting.
    df["event_order"] = df["event"].apply(event_order)

    # Sort by robotID, missionID, secondsVirtual, then by event_order.
    df.sort_values(
        by=["secondsVirtual", "event_order"],
        inplace=True
    )


def convert_df_events_to_missions(
        df: pd.DataFrame,
        *,
        is_concise: bool = False,
        is_single_row: bool = False,
) -> MissionsDict:
    """
    Convert the dataframe to a dictionary mapping robotID to missions.

    Each mission is a list of rows (dictionaries) from the TSV file.
    A mission starts when an event with type "MissionStarted" is found and
    ends when an event with type "MissionFinished" is encountered.
    The 'event' key in each row is converted from a JSON string into a
    dictionary.

    Returns:
        A dictionary where each key is a robotID and each value is a list
        of missions. Each mission is a list of rows (dictionaries) from the file.
    """
    df['event'] = df['event'].apply(json.loads)

    if is_single_row:
        assert len(df) == 1
    else:
        sort_df(df)
    rows: List[Row] = df.to_dict(orient="records")

    robot_id_to_missions: MissionsDict = {}
    active_missions: Dict[str, Mission] = {}

    for _i_row, row in enumerate(rows):
        event_data: Dict[str, Any] = row["event"].copy()

        seconds: str = row["secondsVirtual"]
        event_type: str = event_data.pop("type")
        robot_id: str = event_data.pop("robotID")

        if is_concise:
            row_out = f"{seconds:6.1f}: {event_type}"
            if event_data:
                row_out += ": " + ", ".join(f"{key}={value!r}" for key, value in event_data.items())
        else:
            row_out = row

        if is_single_row or event_type == "MissionStarted":
            # Start a new mission for this robot.
            assert robot_id not in active_missions, (
                f"Robot {robot_id} already has an active mission."
            )
            active_missions[robot_id] = [row_out]
            robot_id_to_missions.setdefault(robot_id, []).append(
                active_missions[robot_id]
            )
        elif event_type == "MissionFinished":
            # End the mission for this robot.
            assert robot_id in active_missions, (
                f"No active mission for robot {robot_id} to finish."
            )
            active_missions[robot_id].append(row_out)
            del active_missions[robot_id]
        else:
            # Append the row to an active mission.
            assert robot_id in active_missions, (
                f"No active mission for robot {robot_id} to append to."
            )
            active_missions[robot_id].append(row_out)

    return dict(sorted(robot_id_to_missions.items()))


def convert_missions_to_df_missions(robot_id_to_missions: MissionsDict) -> pd.DataFrame:
    """
    Convert a MissionsDict into a Pandas DataFrame with the following columns:
    - robotID: the robot identifier (from the dictionary key)
    - missionID: a sequential mission number for that robot (starting with 1)
    - secondsVirtual: the original column from the TSV row
    - event_type: the event type (taken from the event dict's 'type' key)
    - additional columns for each other field in the event (excluding 'type' and 'robotID')
    - any other columns present in the original row (outside of the nested 'event')

    Args:
        robot_id_to_missions: A dictionary mapping robot IDs to a list of missions. Each mission
                  is a list of rows (dictionaries) as produced by `parse_tsv_file`
                  (with is_concise=False).

    Returns:
        A Pandas DataFrame containing the flattened mission data.
    """
    records: List[Dict[str, Any]] = []

    for robot_id, missions in robot_id_to_missions.items():
        # Enumerate missions per robot (mission numbering starts at 1).
        for mission_idx, mission in enumerate(missions, start=1):
            for row in mission:
                record: Dict[str, Any] = {"robotID": robot_id, "missionID": mission_idx}

                for key, value in row.items():
                    if key != "event":
                        assert key not in record
                        record[key] = value
                    else:
                        for key_event, value_event in value.items():
                            if key_event != "robotID":
                                key_record = f"event_{key_event}"
                                assert key_record not in record
                                record[key_record] = value_event

                records.append(record)

    df = pd.DataFrame(records)
    columns = list(df.columns)
    index_time = columns.index('secondsVirtual')
    columns_event = [col for col in columns if col.startswith('event_')]
    df = df[
            columns[:index_time + 1] +
            columns_event +
            [col for col in columns[index_time + 1:] if not col.startswith('event_')]
    ]
    return df


import pandas as pd


def add_related_event_counts(
    df: pd.DataFrame, related_event_type: str
) -> pd.DataFrame:
    """
    For each row in the dataframe with event_type 'ForcingStarted',
    add counts of subsequent events of type `related_event_type` in four scopes:

      - "{related_event_type} before forcing ends": Count of events of type
        `related_event_type` after the ForcingStarted event until the next
        ForcingFinished event (in the same mission). If no ForcingFinished is
        found, count until mission end.

      - "{related_event_type} before mission ends": Count of events of type
        `related_event_type` after the ForcingStarted event until the mission ends.

      - "{related_event_type} before scenario ends": Count of events of type
        `related_event_type` after the ForcingStarted event until the end of the
        dataframe (for that robot).

      - "{related_event_type} before next forcing starts": Count of events of type
        `related_event_type` after the ForcingStarted event until the next
        ForcingStarted event (in the same mission). If no further ForcingStarted is
        found, count until mission end.

    For rows that are not ForcingStarted, these new columns will contain NaN.

    The dataframe is assumed to contain at least the following columns:
        - robotID
        - missionID
        - secondsVirtual
        - event_type

    Additionally, for events with the same robotID, missionID, and secondsVirtual,
    the ordering is defined as:
        any other event, ForcingFinished, MissionFinished, MissionStarted, ForcingStarted

    Args:
        df: Original dataframe.
        related_event_type: The event type to count (for example, "MinorCollision").

    Returns:
        A new DataFrame (sorted by robotID, missionID, secondsVirtual, and event order)
        with the four additional columns.
    """
    # Work on a copy.
    df = df.copy()

    et_started = "ForcingReactionStarted"
    et_finished = "ForcingReactionFinished"

    # (Step 1: extracted into `sort_df`.)

    # --- Step 2. Create helper columns for counting related events ---
    # Mark 1 for rows where event_type equals the provided related_event_type.
    if 'event_otherID' not in df:
        df['event_otherID'] = pd.NA
    df["is_related_event"] = ((df["event_type"] == related_event_type) & (df["event_otherID"] == ID_HUMAN)).astype(int)

    # Cumulative counts within each mission and for each robot.
    cum_mission_col = f"cum_{related_event_type}_mission"
    cum_scenario_col = f"cum_{related_event_type}_scenario"
    df[cum_mission_col] = df.groupby(["robotID", "missionID"])["is_related_event"].cumsum()
    df[cum_scenario_col] = df.groupby("robotID")["is_related_event"].cumsum()

    # For scenario counts, record the final cumulative count for each robot.
    final_scenario_col = f"final_{related_event_type}_scenario"
    df[final_scenario_col] = df.groupby("robotID")[cum_scenario_col].transform("last")

    # --- Step 3. Initialize new count columns (only filled for ForcingStarted rows) ---
    col_before_forcing = f"{related_event_type} before forcing ends"
    col_before_next_forcing = f"{related_event_type} before next forcing starts"
    col_before_mission = f"{related_event_type} before mission ends"
    col_before_scenario = f"{related_event_type} before scenario ends"

    for col in [
        col_before_forcing,
        col_before_next_forcing,
        col_before_mission,
        col_before_scenario,
    ]:
        df[col] = pd.NA

    # --- Step 4. Process each mission group ---
    # Group by robotID and missionID so that counts are computed per mission.
    for (robot, mission), group in df.groupby(["robotID", "missionID"]):
        # Reset the group's index to iterate in sorted order; preserve original indices.
        g = group.reset_index()  # 'index' column holds the original index.
        # Get the last cumulative count within the mission.
        last_cum_mission = g[cum_mission_col].iloc[-1]
        for i, row in g.iterrows():
            if row["event_type"] == et_started:
                cum_at_forcing = row[cum_mission_col]
                # Count of related events from forcing start until mission end.
                mission_count = last_cum_mission - cum_at_forcing
                df.at[row["index"], col_before_mission] = mission_count

                # --- Count until the next ForcingFinished event ---
                subsequent = g.iloc[i + 1 :]
                forcing_finished = subsequent[subsequent["event_type"] == et_finished]
                if not forcing_finished.empty:
                    cum_at_finished = forcing_finished.iloc[0][cum_mission_col]
                    forcing_count = cum_at_finished - cum_at_forcing
                else:
                    forcing_count = mission_count
                df.at[row["index"], col_before_forcing] = forcing_count

                # --- Count until the next ForcingStarted event ---
                next_forcing = subsequent[subsequent["event_type"] == et_started]
                if not next_forcing.empty:
                    cum_at_next_forcing = next_forcing.iloc[0][cum_mission_col]
                    next_forcing_count = cum_at_next_forcing - cum_at_forcing
                else:
                    next_forcing_count = mission_count
                df.at[row["index"], col_before_next_forcing] = next_forcing_count

                # --- Count until scenario (all events for this robot) ends ---
                scenario_count = row[final_scenario_col] - row[cum_scenario_col]
                df.at[row["index"], col_before_scenario] = scenario_count

    # Optionally, drop helper columns if no longer needed.
    df.drop(
        columns=[
            "is_related_event", cum_mission_col, cum_scenario_col, final_scenario_col
        ],
        inplace=True
    )

    return df


def check_no_moments_common(filename_missions_csv, df_missions, robot_id_to_missions):
    for robot_id in robot_id_to_missions:
        df_robot = df_missions[df_missions["robotID"] == robot_id]
        assert not df_robot.empty
        moments_mission_started = df_robot[df_robot['event_type'] == 'MissionStarted']['secondsVirtual']
        moments_forcing_reaction_started = df_robot[df_robot['event_type'] == 'ForcingReactionStarted']['secondsVirtual']
        moments_common = sorted(set(moments_mission_started) & set(moments_forcing_reaction_started))
        if moments_common:
            print(f'{filename_missions_csv}, {robot_id=}: ' + ' ,'.join(map(str, moments_common)))
        assert not moments_common


def convert_map_event_to_df_missions(map_event: dict[str, str]) -> pd.DataFrame:
    df_strings = pd.DataFrame.from_records([map_event])

    stream = io.StringIO()
    df_strings.to_csv(stream, index=False)
    stream.seek(0)
    df_events = pd.read_csv(stream)
    stream.close()

    return convert_df_events_to_df_missions(df_events, is_single_row=True)


def convert_df_events_to_df_missions(
        df_events: pd.DataFrame,
        filename_missions_csv: str | None = None,
        *,
        is_single_row: bool = False
) -> pd.DataFrame:
    robot_id_to_missions = convert_df_events_to_missions(df_events, is_concise=False, is_single_row=is_single_row)
    df_missions = convert_missions_to_df_missions(robot_id_to_missions)
    if not is_single_row:
        check_no_moments_common(filename_missions_csv, df_missions, robot_id_to_missions)
    return df_missions


def convert(filename_events_tsv: str, filename_missions_csv: str) -> None:
    df_events = pd.read_csv(filename_events_tsv, sep="\t")
    df = convert_df_events_to_df_missions(df_events, filename_missions_csv)
    assert len(df) == len(df_events)

    for related_event in 'MinorCollision', 'MajorCollisionFromMinor':
        df = add_related_event_counts(df, related_event)

    assert len(df) == len(df_events)
    df.to_csv(filename_missions_csv, index=False)


def main() -> None:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <filename_events_tsv> <filename_missions_csv>")
        sys.exit(1)

    filename_events_tsv: str = sys.argv[1]
    filename_missions_csv: str = sys.argv[2]

    df = convert(filename_events_tsv, filename_missions_csv)
    with pd.option_context("display.max_rows", 50,
                           "display.max_columns", 10,
                           "display.width", WIDTH_CONSOLE):
        print(df)


if __name__ == "__main__":
    main()
