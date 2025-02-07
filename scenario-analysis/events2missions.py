#!/usr/bin/env python3
"""
This module reads a TSV file containing events and converts it into a
dictionary mapping robotID to missions. Each mission is defined as the
sequence of rows (from the TSV) starting with an event of type
'MissionStarted' and ending with an event of type 'MissionFinished'.
In the output, the 'event' field is a dictionary rather than a JSON string.
"""
import json
import shutil
import sys
from typing import Any, Dict, List

import pandas as pd

# Type aliases for clarity.
Row = Dict[str, Any]
Mission = List[Row]
MissionsDict = Dict[str, List[Mission]]

WIDTH_CONSOLE = shutil.get_terminal_size().columns


def parse_tsv_file(file_path: str, *, is_concise: bool = False) -> MissionsDict:
    """
    Parse the TSV file and convert it to a dictionary mapping robotID
    to missions.

    Each mission is a list of rows (dictionaries) from the TSV file.
    A mission starts when an event with type "MissionStarted" is found and
    ends when an event with type "MissionFinished" is encountered.
    The 'event' key in each row is converted from a JSON string into a
    dictionary.

    Args:
        file_path: The path to the TSV file.

    Returns:
        A dictionary where each key is a robotID and each value is a list
        of missions. Each mission is a list of rows (dictionaries) from the file.
    """
    # Read the TSV file using pandas; numeric columns are inferred.
    df: pd.DataFrame = pd.read_csv(file_path, sep="\t")
    rows: List[Row] = df.to_dict(orient="records")

    robot_id_to_missions: MissionsDict = {}
    active_missions: Dict[str, Mission] = {}

    for row in rows:
        event_str: str = row["event"]
        event_data: Dict[str, Any] = json.loads(event_str)
        # Replace the 'event' field with the parsed dictionary.
        row["event"] = event_data.copy()

        seconds: str = row["secondsVirtual"]
        event_type: str = event_data.pop("type")
        robot_id: str = event_data.pop("robotID")

        if is_concise:
            row_out = f"{seconds:6.1f}: {event_type}"
            if event_data:
                row_out += ": " + ", ".join(f"{key}={value!r}" for key, value in event_data.items())
        else:
            row_out = row

        if event_type == "MissionStarted":
            # Start a new mission for this robot.
            assert robot_id not in active_missions, (
                f"Robot {robot_id} already has an active mission."
            )
            active_missions[robot_id] = [row_out]
        elif event_type == "MissionFinished":
            # End the mission for this robot.
            assert robot_id in active_missions, (
                f"No active mission for robot {robot_id} to finish."
            )
            active_missions[robot_id].append(row_out)
            robot_id_to_missions.setdefault(robot_id, []).append(
                active_missions.pop(robot_id)
            )
        else:
            # Append the row to an active mission.
            assert robot_id in active_missions, (
                f"No active mission for robot {robot_id} to append to."
            )
            active_missions[robot_id].append(row_out)

    return dict(sorted(robot_id_to_missions.items()))


def missions_to_dataframe(robot_id_to_missions: MissionsDict) -> pd.DataFrame:
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


def add_minor_collision_counts(df: pd.DataFrame) -> pd.DataFrame:
    """
    For each row in the dataframe with event_type 'ForcingStarted',
    add counts of subsequent 'MinorCollision' events in four scopes:

      - "MinorCollision before forcing ends": Count of MinorCollision events
        after the ForcingStarted event until the next ForcingFinished event
        (in the same mission). If no ForcingFinished is found, count until mission end.

      - "MinorCollision before mission ends": Count of MinorCollision events
        after the ForcingStarted event until the mission ends.

      - "MinorCollision before scenario ends": Count of MinorCollision events
        after the ForcingStarted event until the end of the dataframe (for that robot).

      - "MinorCollision before next forcing starts": Count of MinorCollision events
        after the ForcingStarted event until the next ForcingStarted event
        (in the same mission). If no further ForcingStarted is found, count until mission end.

    For rows that are not ForcingStarted, these new columns will contain NaN.

    The dataframe is assumed to contain at least the following columns:
        - robotID
        - missionID
        - secondsVirtual
        - event_type

    Additionally, for events with the same robotID, missionID, and secondsVirtual,
    the ordering is defined as:
        any other event, ForcingFinished, MissionFinished, MissionStarted, ForcingStarted

    Returns:
        A new DataFrame (sorted by robotID, missionID, secondsVirtual, and event order)
        with the four additional columns.
    """
    # Work on a copy.
    df = df.copy()

    # --- Step 1. Define custom ordering for events at the same time ---
    def event_order(event_type: str) -> int:
        # Any event not in the mapping gets order 0.
        mapping = {
            "ForcingFinished": 1,
            "MissionFinished": 2,
            "MissionStarted": 3,
            "ForcingStarted": 4,
        }
        return mapping.get(event_type, 0)

    df["event_order"] = df["event_type"].apply(event_order)

    # Sort by robotID, missionID, secondsVirtual, then by event_order.
    df.sort_values(
        by=["robotID", "missionID", "secondsVirtual", "event_order"],
        inplace=True
    )

    # --- Step 2. Create helper columns for counting collisions ---
    # Mark 1 for MinorCollision events, else 0.
    df["is_minor_collision"] = (df["event_type"] == "MinorCollision").astype(int)

    # Cumulative collision counts within each mission and for each robot.
    df["cum_minor_mission"] = df.groupby(["robotID", "missionID"])["is_minor_collision"].cumsum()
    df["cum_minor_scenario"] = df.groupby("robotID")["is_minor_collision"].cumsum()

    # For scenario counts, get the final cumulative count for each robot.
    df["final_scenario"] = df.groupby("robotID")["cum_minor_scenario"].transform("last")

    # --- Step 3. Initialize new count columns ---
    new_columns = [
        "MinorCollision before forcing ends",
        "MinorCollision before next forcing starts",
        "MinorCollision before mission ends",
        "MinorCollision before scenario ends",
    ]
    for col in new_columns:
        df[col] = pd.NA

    # --- Step 4. Process each mission group ---
    # We group by robotID and missionID so that the counts are restricted to a mission.
    for (robot, mission), group in df.groupby(["robotID", "missionID"]):
        # Reset index within the group to iterate in sorted order,
        # but keep the original index in a column.
        g = group.reset_index()  # This creates a column 'index' with the original index.
        # Last cumulative collisions in the mission.
        last_cum_mission = g["cum_minor_mission"].iloc[-1]
        for i, row in g.iterrows():
            if row["event_type"] == "ForcingStarted":
                cum_at_forcing = row["cum_minor_mission"]
                # Collisions until mission end.
                mission_collisions = last_cum_mission - cum_at_forcing
                # Set "MinorCollision before mission ends".
                df.at[row["index"], "MinorCollision before mission ends"] = mission_collisions

                # --- Count until next ForcingFinished ---
                subsequent = g.iloc[i+1:]
                forcing_finished = subsequent[subsequent["event_type"] == "ForcingFinished"]
                if not forcing_finished.empty:
                    cum_at_finished = forcing_finished.iloc[0]["cum_minor_mission"]
                    forcing_collisions = cum_at_finished - cum_at_forcing
                else:
                    forcing_collisions = mission_collisions
                df.at[row["index"], "MinorCollision before forcing ends"] = forcing_collisions

                # --- Count until next ForcingStarted ---
                next_forcing = subsequent[subsequent["event_type"] == "ForcingStarted"]
                if not next_forcing.empty:
                    cum_at_next_forcing = next_forcing.iloc[0]["cum_minor_mission"]
                    next_forcing_collisions = cum_at_next_forcing - cum_at_forcing
                else:
                    next_forcing_collisions = mission_collisions
                df.at[row["index"], "MinorCollision before next forcing starts"] = next_forcing_collisions

                # --- Count until scenario end (all events for this robot) ---
                scenario_collisions = row["final_scenario"] - row["cum_minor_scenario"]
                df.at[row["index"], "MinorCollision before scenario ends"] = scenario_collisions

    # Optionally, drop helper columns.
    # df.drop(
    #     columns=[
    #         "event_order", "is_minor_collision", "cum_minor_mission",
    #         "cum_minor_scenario", "final_scenario"
    #     ],
    #     inplace=True
    # )

    return df


def main() -> None:
    """
    Main function to parse a TSV file provided as a command-line argument and
    print the resulting dictionary of missions in JSON format.
    """
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <tsv_file_path> <csv_file_path_out>")
        sys.exit(1)

    file_path: str = sys.argv[1]
    file_path_out: str = sys.argv[2]

    robot_id_to_missions: MissionsDict = parse_tsv_file(file_path, is_concise=False)
    df = missions_to_dataframe(robot_id_to_missions)
    df = add_minor_collision_counts(df)
    with pd.option_context("display.max_rows", 50,
                           "display.max_columns", 10,
                           "display.width", WIDTH_CONSOLE):
        print(df)
    df.to_csv(file_path_out, index=False)


if __name__ == "__main__":
    main()
