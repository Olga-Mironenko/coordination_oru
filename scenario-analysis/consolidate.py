#!/usr/bin/env python3

import glob
import sys

import pandas as pd


def main(root_dir: str = ".") -> None:
    # Initialize a list to hold all rows
    rows = []

    # Iterate through directories and files
    for filename in glob.glob(f'{root_dir}/*/*.csv'):
        # Read the file as key-value pairs
        with open(filename) as file:
            row = {}
            for line in file:
                parts = line.rstrip('\n').split("\t")
                assert len(parts) == 2, (filename, line)
                row[parts[0]] = parts[1]
        rows.append(row)

    # Create the DataFrame
    df = pd.DataFrame(rows)
    df.to_csv(root_dir + "/sorted.csv", index=False)


if __name__ == "__main__":
    main(*sys.argv[1:])