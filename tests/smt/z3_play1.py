"""
z3_play1.py
Description:
    Attempting to test simple versions of the constraints that we need for our work.
"""

import sys
sys.path.append("/Users/kwesirutledge/Documents/Development/z3/build/python/")
import z3


if __name__ == '__main__':
    print("extract_constraints.py")

    # Solve a Simple Big-M type of constraint
    b1 = z3.Bool("b1")
    b2 = z3.Bool("b2")
    b3 = z3.Bool("b3")

    # Call Solver
    s = z3.Solver()
    s.add(
        b1 + 2*b2 + b3 >= 2
    )
    s.add(
        b1 == False
    )
    s.add(
        b1 == b3
    )

    s.check()
    print(s.model())