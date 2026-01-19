import sys
import os
import json


# ---  FUNCTIONS  --- #
# ------------------- #
def print_help():
    print("""
Input arguments:
    1 -> Path to first JSON report file
    2 -> Path to second JSON report file
""")


def parse_args(helpf=print_help):
    """Parse input arguments. Raise an exception if not correct arguments were
    given"""
    if len(sys.argv) == 1:
        helpf()
        exit(0)
    elif len(sys.argv) < 3:
        raise Exception(
            "{m} arguments were given but 2 are required".format(m=len(sys.argv) - 1)
        )
    data_path = sys.argv[1]
    if not validate_file_path(data_path):
        raise Exception(
            'The path "{p}"\n'
            "was given as path to first input file, but it is not valid"
        )
    datb_path = sys.argv[2]
    if not validate_file_path(datb_path):
        raise Exception(
            'The path "{p}"\n'
            "was given as path to second input file, but it is not valid"
        )
    return {"data_path": data_path, "datb_path": datb_path}


def validate_file_path(path):
    """Check path points to a valid existent file"""
    return os.path.exists(path) and os.path.isfile(path)


def read_data(path):
    """Read data from given file path"""
    with open(path, "r") as json_file:
        return json.load(json_file)


def compare_data(data, datb):
    """Compare data and datb, returning its differences"""
    diff = []
    pending = [(data, datb)]
    while len(pending) > 0:
        A, B = pending[0]
        pending = pending[1:]
        for keya, keyb in zip(A.keys(), B.keys()):
            if keya != keyb:
                raise (
                    "Incompatible format between reports\n"
                    "A has key {keya} while B has key {keyb}".format(
                        keya=keya, keyb=keyb
                    )
                )
            vala, valb = A[keya], B[keyb]
            if type(vala) is dict:
                pending.append((vala, valb))
            elif vala != valb:
                diff.append(
                    {"A": {"key": keya, "val": vala}, "B": {"key": keyb, "val": valb}}
                )
    return diff


def report_diff(diff):
    """Print the output of the compare_data function"""
    for i, d in enumerate(diff):
        print(
            "diff[{i}]:\n"
            "\tA.{daKey}: {daVal}\n"
            "\tB.{dbKey}: {dbVal}\n".format(
                i=i,
                daKey=d["A"]["key"],
                daVal=d["A"]["val"],
                dbKey=d["B"]["key"],
                dbVal=d["B"]["val"],
            )
        )
    print("There are {m} differences between given JSON reports".format(m=len(diff)))


# ---   M A I N   --- #
# ------------------- #
if __name__ == "__main__":
    args = parse_args()
    data, datb = read_data(args["data_path"]), read_data(args["datb_path"])
    diff = compare_data(data, datb)
    report_diff(diff)
