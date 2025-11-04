#!/usr/bin/env python3
# 
# ref from here :- https://github.com/nickmccleery/utilities/blob/main/mimo_bode_example.py
#
import rich_click as click
from math import pi
from typing import List, Tuple
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

J = [8 -3 -3; -3 8 -3; -3 -3 8];
F = 0.2*eye(3);
A = -J\F;
B = inv(J);
C = eye(3);
D = 0;
sys_mimo = ss(A,B,C,D);

#  ----------- define you state models -------------------
def build_torsional_system_ss() -> Tuple[
    np.matrix,
    np.matrix,
    np.matrix,
    np.matrix
]:
    """
    Generates the state-space representation of a damped rotating body.
    Ref: https://uk.mathworks.com/help/control/ug/mimo-state-space-models.html

    Returns:
        Tuple[ np.matrix, np.matrix, np.matrix, np.matrix ]: A, B, C, D values.
    """
    J = np.matrix([
        [8, -3, -3],
        [-3, 8, -3],
        [-3, -3, 8]]
    )
    F = 0.2 * np.eye(3)
    A = np.linalg.lstsq(-J, F)[0]
    B = np.linalg.inv(J)
    C = np.matrix(np.eye(3))
    D = np.matrix(np.zeros((3, 3)))

    return A, B, C, D


def build_aircraft_system_ss() -> Tuple[
    np.matrix,
    np.matrix,
    np.matrix,
    np.matrix
]:
    """
    Generates the state-space representation of a jet transport.
    Ref: https://uk.mathworks.com/help/control/ug/mimo-state-space-models.html

    Returns:
        Tuple[ np.matrix, np.matrix, np.matrix, np.matrix ]: A, B, C, D values.
    """
    A = np.matrix(
        [
            [-0.0558, -0.9968, 0.0802, 0.0415],
            [0.590, -0.1150, -0.0318, 0],
            [-3.05, 0.388, -0.465, 0],
            [0, 0.0805, 1, 0]
        ]
    )

    B = np.matrix(
        [
            [0.0073, 0],
            [-0.475, 0.0077],
            [0.153, 0.143],
            [0, 0]
        ]
    )

    C = np.matrix(
        [
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ]
    )

    D = np.matrix(
        [
            [0, 0],
            [0, 0]
        ]
    )

    return A, B, C, D

#  ----------- functions for processing -------------------
def transform(
    A: np.matrix,
    B: np.matrix,
    C: np.matrix,
    D: np.matrix,
    i_input: int
) -> Tuple[np.array, np.array]:
    """
    Transforms a given segment of a state-space representation of a MIMO system
    to transfer function representation. This is done only for the specified
    input, and both C and D matrices must be sliced to represent a single
    output.

    Args:
        A (np.matrix): State matrix.
        B (np.matrix): Input matrix.
        C (np.matrix): Output matrix.
        D (np.matrix): Feedforward matrix.
        i_input (int): Index of input to be considered.

    Returns:
        Tuple[np.array, np.array]: Numerator and denominator terms for TF.
    """

    # Sanity check shapes.
    n1, n2 = A.shape
    n3, p1 = B.shape
    q1, n4 = C.shape
    q2, p2 = D.shape

    assert (n1 == n2 == n3 == n4)
    assert (p1 == p2)
    assert (q1 == q2)

    assert A.shape == (n1, n1)
    assert B.shape == (n1, p1)
    assert C.shape == (q1, n1)
    assert D.shape == (q1, p1)

    # Convert to transfer function.
    num, den = signal.ss2tf(A, B, C, D, i_input)

    return num, den

def convert_mimo_ss_system(
    A: np.matrix,
    B: np.matrix,
    C: np.matrix,
    D: np.matrix
) -> List[List[signal._ltisys.TransferFunctionContinuous]]:
    """
    Converts a MIMO state-space representation of system to an array of 
    transfer functions.

    Args:
        A (np.matrix): State matrix.
        B (np.matrix): Input matrix.
        C (np.matrix): Output matrix.
        D (np.matrix): Feedforward matrix.

    Returns:
        List[List[signal._ltisys.TransferFunctionContinuous]]: An array of
        transfer function definitions; really a list of lists.
    """
    p_inputs = B.shape[1]
    q_outputs = C.shape[0]

    # ! Pre-allocating a list of lists can cause some headaches...
    # ! output = [[None] * len(tfs)] * len(tfs[0]) -< THIS IS A TRAP
    # ! See: https://bit.ly/3COpGZH
    tfs = [[None]*q_outputs for i in range(p_inputs)]

    for i_out in range(0, q_outputs):
        C_current = C[i_out, :]
        D_current = D[i_out, :]
        for i_in in range(0, p_inputs):
            num, den = transform(A, B, C_current, D_current, i_in)
            tfs[i_out][i_in] = signal.TransferFunction(num, den)

    return tfs

def compute_mimo_bode(
    tfs: List[List[signal._ltisys.TransferFunctionContinuous]]
) -> List[List[dict]]:
    """
    Generate Bode plot data for an array of transfer functions.

    Args:
        tfs (List[List[signal._ltisys.TransferFunctionContinuous]]): Array of
        transfer functions.

    Returns:
        List[List[dict]]: Array of response objects for transfer functions.
    """

    # ! Pre-allocating a list of lists can cause some headaches...
    # ! output = [[None] * len(tfs)] * len(tfs[0]) -< THIS IS A TRAP
    # ! See: https://bit.ly/3COpGZH
    output = [[None]*len(tfs) for i in range(len(tfs[0]))]

    w_in = np.linspace(0, 2*pi, 1000)
    for j, row in enumerate(tfs):
        for k, sys in enumerate(row):
            w, mag, phase = signal.bode(sys, w=w_in)

            output[j][k] = {
                'w': w,
                'mag': mag,
                'phase': phase
            }
            del sys

    return output

def plot_mimo_bode(data: List[List[dict]]):
    """
    Plots and saves MIMO bode data.

    Args:
        data (List[List[dict]]): Bode/response data array.
    """
    fig = plt.figure()
    fig.set_size_inches(16, 10, True)

    n_rows = len(data) * 2
    n_cols = len(data[0])
    i_plot = 1

    for i, row in enumerate(data):
        for val in row:
            ax = fig.add_subplot(n_rows, n_cols, i_plot)
            ax.plot(val['w'], val['mag'])

            ax = fig.add_subplot(n_rows, n_cols, i_plot + n_cols)
            plt.plot(val['w'], val['phase'])

            i_plot += 1
        i_plot += n_cols

    plt.show()
    fig.savefig('output.png')

    return

@click.command()
@click.option("--state_space_model", default="aircraft", help="state space model to choose", type=click.Choice(["aircraft", "torsional"]))
def main(state_space_model: str) -> None:
    if state_space_model == "torsional":
        A, B, C, D = build_torsional_system_ss()
    elif state_space_model == "aircraft":
        A, B, C, D = build_aircraft_system_ss()
    else:
        print("valid options for the state space model are aircraft torsional")

    tfs = convert_mimo_ss_system(A, B, C, D)
    resp = compute_mimo_bode(tfs)
    plot_mimo_bode(resp)

if __name__ == "__main__":
    main()