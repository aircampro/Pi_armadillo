# ref:- pip install diffcp
# example ref:- https://web.stanford.edu/~boyd/papers/diff_cone_prog.html
#
import numpy as np
from scipy import sparse
import diffcp

cone_dict = {
    diffcp.ZERO: 3,
    diffcp.POS: 3,
    diffcp.SOC: [5]
}

m = 3 + 3 + 5
n = 5

A, b, c = diffcp.utils.random_cone_prog(m, n, cone_dict)
x, y, s, D, DT = diffcp.solve_and_derivative(A, b, c, cone_dict)

# evaluate the derivative
nonzeros = A.nonzero()
data = 1e-4 * np.random.randn(A.size)
dA = sparse.csc_matrix((data, nonzeros), shape=A.shape)
db = 1e-4 * np.random.randn(m)
dc = 1e-4 * np.random.randn(n)
dx, dy, ds = D(dA, db, dc)

# evaluate the adjoint of the derivative
dx = c
dy = np.zeros(m)
ds = np.zeros(m)
dA, db, dc = DT(dx, dy, ds)