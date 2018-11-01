#!/bin/env python3.6
import collections
import os
import re
import sys

if __name__ == "__main__":
    try:
        b = sys.argv[1]
    except IndexError:
        raise AssertionError("Please Specify behavior")

    decisions_out = collections.defaultdict(list)
    for r, p, dfs in os.walk(os.path.join(b, "decisions")):
        for df in dfs:
            fn = ""
            with open(os.path.join(r, df), "r") as dp:
                for line in dp:
                    m = re.findall(r"(?<=class\s)[a-zA-Z0-9]*", line)
                    if m:
                        fn = m[0]
                        continue
                    if "_register" in line:
                        l = dp.readline().replace("return", "").replace(" ", "")
                        print(l)
                        decisions_out[fn] = eval(l)


    actions_out = []
    for r, _, afs in os.walk(os.path.join(b, "actions")):
        for af in afs:
            with open(os.path.join(r, af), "r") as dp:
                for line in dp:
                    m = re.findall(r"(?<=class\s)[a-zA-Z0-9]*", line)
                    if m:
                        actions_out.append(m[0])

    with open(os.path.join(b, "gen.dsd"), "w") as ww:
        ww.write("//** BEGIN AUTOGEN DO NOT EDIT\n\n")
        for k in decisions_out.keys():
            ww.write(f"${k}\n")
            for v in decisions_out[k]:
                ww.write(f"\t{v}\n")

        for k in actions_out:
            ww.write(f"@{k}\n")

        ww.write("END AUTOGEN **//\n")
