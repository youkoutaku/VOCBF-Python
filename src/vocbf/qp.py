"""Small convex QPs solved by complete two-dimensional active-set enumeration.

The implementation is independent of the historical polygon-based solver.
Constraints use A u <= b. Soft CLF inequalities are eliminated analytically;
all 16 slack sign regions are checked, as are obstacle-side disjunctions.
"""
import itertools
import numpy as np


def quadratic(H, q, A, b, tolerance=2e-9):
    H, q = np.asarray(H, float), np.asarray(q, float)
    A, b = np.asarray(A, float).reshape(-1, 2), np.asarray(b, float)
    inv = np.linalg.inv(H)
    unconstrained = -inv @ q
    inverse_normals=A@inv
    denominators=np.sum(inverse_normals*A,axis=1)
    nz=denominators>1e-24
    projected=unconstrained+inverse_normals[nz]*((b[nz]-A[nz]@unconstrained)/denominators[nz])[:,None]
    i,j=np.triu_indices(len(b),1)
    determinant=A[i,0]*A[j,1]-A[i,1]*A[j,0]
    keep=abs(determinant)>1e-13*np.linalg.norm(A[i],axis=1)*np.linalg.norm(A[j],axis=1)
    i,j,determinant=i[keep],j[keep],determinant[keep]
    intersections=np.column_stack([(b[i]*A[j,1]-A[i,1]*b[j])/determinant,
                                   (A[i,0]*b[j]-b[i]*A[j,0])/determinant])
    candidates=np.vstack([unconstrained[None,:],projected,intersections])
    feasible=candidates[np.all(candidates@A.T<=b+tolerance,axis=1)]
    if not len(feasible):
        return None, np.inf
    objectives=.5*np.sum((feasible@H)*feasible,axis=1)+feasible@q
    best=feasible[int(np.argmin(objectives))]
    return best, float(.5*best@H@best+q@best)


def soft_clf(regions, clf_A, clf_c, weights, previous, Hdiag, Rdiag):
    best, value = None, np.inf
    for bits in itertools.product([False, True], repeat=len(clf_c)):
        active = np.asarray(bits)
        H = np.diag(Hdiag+Rdiag) + 2*(clf_A[active].T*weights[active])@clf_A[active]
        q = -Rdiag*previous + 2*clf_A[active].T@(weights[active]*clf_c[active])
        signs = np.where(active, -1., 1.)
        for A, b in regions:
            u, _ = quadratic(H, q, np.vstack([A, signs[:, None]*clf_A]),
                             np.r_[b, -signs*clf_c])
            if u is None:
                continue
            slack = np.maximum(0., clf_A@u+clf_c)
            objective = (.5*np.dot(Hdiag, u*u)
                         + .5*np.dot(Rdiag, (u-previous)**2)
                         + np.dot(weights, slack*slack))
            if objective < value:
                best, value = u, float(objective)
    return best, value
