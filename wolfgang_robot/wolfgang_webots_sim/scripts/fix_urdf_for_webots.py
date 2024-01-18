#!/usr/bin/env python3

import sys
import xml.etree.ElementTree as ET

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <infile> <outfile>")
    exit(1)

infile_path = sys.argv[1]
outfile_path = sys.argv[2]

tree = ET.parse(infile_path)
root = tree.getroot()

for child in root:
    if child.tag == "link":
        found_intertial = False
        found_collision = False
        for link_element in child:
            if link_element.tag == "inertial":
                found_intertial = True
            elif link_element.tag == "collision":
                found_collision = True
        if found_intertial and (not found_collision):
            for link_element in child:
                if link_element.tag == "inertial":
                    child.remove(link_element)
                    print(f"removed inertial element for link: {child.attrib}")

tree.write(outfile_path)