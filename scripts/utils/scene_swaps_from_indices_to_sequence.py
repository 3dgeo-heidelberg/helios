#!/bin/python3

# --------------------------------------------------------------------------- #
# AUTHOR: Alberto M. Esmoris Pena                                             #
# BRIEF: Script to transform a XML scene description with swaps given through #
#           indices to a sorted sequence of swaps.                            #
# --------------------------------------------------------------------------- #

# ---  IMPORTS  --- #
# ----------------- #
import numpy as np
import xml.etree.ElementTree as ET
import itertools
import sys


# ---   UTILS   --- #
# ----------------- #
def sequence_from_indices(old_node, i, last_old_node, last_new_node):
    """
    Transform a swap element in the old index-like format to the new
    sequence-like format.

    :param old_node: The swap element with indices.
    :param i: The index of the current node.
    :param last_old_node: The old swap element from the previous iteration (it
        can be None, i.e., for the first iteration).
    :param last_new_node: The new swap element from the previous iteration (it
        can be None, i.e., for the first iteration).
    :return: New node in sequence-like format.
    """
    # Handle consecutive repetitions of the same node with swapStep
    if old_node == last_old_node:
        swapStep = int(last_new_node.attrib.get("swapStep", 1))
        last_new_node.set("swapStep", str(swapStep + 1))
        return last_new_node
    # Handle general case
    new_node = ET.Element(old_node.tag, attrib=old_node.attrib)
    del new_node.attrib["swapIndices"]
    queue = [(old_child, new_node) for old_child in old_node]
    while len(queue) > 0:
        next_old, parent_new = queue[0]
        del queue[0]
        new = ET.SubElement(parent_new, next_old.tag, attrib=next_old.attrib)
        queue = queue + [(child, new) for child in next_old]
    return new_node


# ---   M A I N   --- #
# ------------------- #
if __name__ == "__main__":
    # Handle help
    if len(sys.argv) != 3:
        print(
            """
The script MUST receive exactly two arguments:

    1) Path to the input XML scene file with swaps in index-like format.

    2) Path to the output XML scene file with swaps in sequence-like format.

"""
        )
        sys.exit(1)
    # Load XML file representing a scene as first argument
    xml = ET.parse(sys.argv[1])
    # Extract all swap nodes
    root = xml.getroot()
    queue = [root]
    swap_nodes = []
    part_nodes = set()  # Only those parts with swap nodes
    while len(queue) > 0:
        node = queue[0]
        del queue[0]
        for child in node:
            queue.append(child)
            if child.tag == "swap":
                part_nodes.add(node)
                swap_nodes.append(
                    {
                        "part": node,  # Parent elementa (scene part)
                        "swap": child,  # The swap element inside the parent
                    }
                )
    # Generate sequence of ordered swaps for each part
    for part in part_nodes:
        swaps = [swap["swap"] for swap in swap_nodes if swap["part"] == part]
        swap_indices = [eval(swap.attrib["swapIndices"]) for swap in swaps]
        swap_max_index = np.max(list(itertools.chain(*swap_indices)))
        last_old_node, last_new_node = [None] * 2
        for i in range(swap_max_index):  # i = 1, ..., m
            old_node = [  # n_j in N s.t. i in I_j, where N nodes, I indices.
                swaps[j] for j in range(len(swaps)) if (i + 1) in swap_indices[j]
            ][0]
            new_node = sequence_from_indices(old_node, i, last_old_node, last_new_node)
            if new_node != last_new_node:
                part.append(new_node)
            last_old_node, last_new_node = old_node, new_node
    # Remove old swap elements
    for swap in swap_nodes:
        swap["part"].remove(swap["swap"])
    # Export transformed XML
    ET.indent(xml)  # Pretty format before writing
    xml.write(sys.argv[2], xml_declaration=True, encoding="utf-8")
