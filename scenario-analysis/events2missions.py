#!/usr/bin/env python3
"""
This module reads a TSV file containing events and converts it into a
dictionary mapping robotID to missions. Each mission is defined as the
sequence of rows (from the TSV) starting with an event of type
'MissionStarted' and ending with an event of type 'MissionFinished'.
In the output, the 'event' field is a dictionary rather than a JSON string.
"""

import json
import sys
from typing import Any, Dict, List

import pandas as pd

# Type aliases for clarity.
Row = Dict[str, Any]
Mission = List[Row]
MissionsDict = Dict[str, List[Mission]]


def parse_tsv_file(file_path: str) -> MissionsDict:
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
        row["event"] = event_data

        event_type: str = event_data["type"]
        robot_id: str = event_data["robotID"]

        if event_type == "MissionStarted":
            # Start a new mission for this robot.
            assert robot_id not in active_missions, (
                f"Robot {robot_id} already has an active mission."
            )
            active_missions[robot_id] = [row]
        elif event_type == "MissionFinished":
            # End the mission for this robot.
            assert robot_id in active_missions, (
                f"No active mission for robot {robot_id} to finish."
            )
            active_missions[robot_id].append(row)
            robot_id_to_missions.setdefault(robot_id, []).append(
                active_missions.pop(robot_id)
            )
        else:
            # Append the row to an active mission.
            assert robot_id in active_missions, (
                f"No active mission for robot {robot_id} to append to."
            )
            active_missions[robot_id].append(row)

    return robot_id_to_missions


def main() -> None:
    """
    Main function to parse a TSV file provided as a command-line argument and
    print the resulting dictionary of missions in JSON format.
    """
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <tsv_file_path>")
        sys.exit(1)

    file_path: str = sys.argv[1]
    robot_id_to_missions: MissionsDict = parse_tsv_file(file_path)

    # Print the resulting missions dictionary in JSON format.
    print(json.dumps(robot_id_to_missions, indent=4))


if __name__ == "__main__":
    main()
