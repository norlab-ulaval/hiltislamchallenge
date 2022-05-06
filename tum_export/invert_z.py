#!/usr/bin/env python3
import argparse
from collections import OrderedDict
from pathlib import Path

import pandas as pd

FIRST_ROW = "# timestamp_s tx ty tz qx qy qz qw"

def parse_arguments():
    """Parse bagfile name"""
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--tum", help="""TUM path""", required=True)
    args = parser.parse_args()
    return Path(args.tum)

def main():
    tumfp = parse_arguments()

    cols = FIRST_ROW[2:].split()

    df = pd.read_csv(tumfp, names=cols, index_col=False, sep=" ", skiprows=[0])
    inverted_df = df.copy()
    inverted_df.tz *= -1

    inverted_path = tumfp.parent / "inverted"
    inverted_path.mkdir(parents=True, exist_ok=True)
    inv_tum_path = inverted_path / tumfp.name

    print("Saving to", inv_tum_path)
    with open(inv_tum_path, "w") as f:
        f.write(FIRST_ROW + "\n")
    inverted_df.to_csv(inv_tum_path, index=False, header=False, mode="a", sep=" ")



if __name__ == "__main__":
    main()

