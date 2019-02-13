#!/usr/bin/env python3

"""
Reads INFILE and creates an eagle scripts that
sets the attributes with names specified in ATTRIBUTES
of all devices in the BOM CSV.

Basically performs a backannotation from an exported BOM 
to a schematic
"""
import csv

INFILE = '../Amplifier+Mixer.csv'
ATTRIBUTES = ['CONRAD', 'MOUSER', 'Stock']

SCR_TEMPLATE = 'ATTRIBUTE {device} {attribute_name} \'{value}\';'

if __name__ == '__main__':
    with open(INFILE, newline='') as csvfile:
        bomreader = csv.DictReader(csvfile, delimiter=';')
        for row in bomreader:
            for attr_name in ATTRIBUTES:
                # check if the the row is present and non-empty
                if attr_name in row and row[attr_name].strip():
                    for device in row['Parts'].split(','):
                        print(SCR_TEMPLATE.format(
                            device=device.strip(), 
                            attribute_name=attr_name.strip(), 
                            value=row[attr_name].strip()))
