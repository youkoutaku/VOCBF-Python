# Model and evidence scope

All systems are written as `xdot = f(x) + g(x) u`. “First-order” and “second-order” below describe the physical position/input relation. State dimension, dynamical order and barrier relative degree are different concepts. These are four concrete planar examples, not a promise that every arbitrary nonlinear control-affine system is supported.

| Example directory | CLI model | State | Input and equations |
|---|---|---|---|
| first_order_affine | single_integrator | `[px, py]` | `[vx, vy]`, `pdot=u` |
| second_order_affine | double_integrator | `[px, py, vx, vy]` | `[ax, ay]`, `pdot=v`, `vdot=u` |
| kinematic_unicycle | kinematic_unicycle | `[qx, qy, theta]` | `[v, omega]`, `qdot=v[cos(theta),sin(theta)]`, `thetadot=omega` |
| dynamic_unicycle | dynamic_unicycle | `[qx, qy, theta, v, omega]` | `[a, alpha]`, additionally `vdot=a`, `omegadot=alpha` |

For both unicycles the obstacle geometry is measured at `p=q+ell[cos(theta),sin(theta)]`, with `ell=0.15 m` by default. `--offset 0` retains the axle-centered model and exposes its lost instantaneous angular channel where applicable. It is not silently replaced with a nonzero certificate offset.

The `Model.affine` interface exposes drift and control matrices. A custom model also needs a defined position output, dimensions, physical bounds, nominal controller and integration. Verify the appropriate barrier relative degree and assumptions before reusing the example controller. Changing f/g is not sufficient to inherit the paper theorem.

The demo observer uses independent DOP853 integration at 21 points per hold, saves every actual input and reports sampled collisions. This is not a validated continuous-time enclosure. MPC prediction uses exact integrators and RK4 for unicycles; ZOCBF unicycle prediction uses a numerical flow, not a proved integration-error bound. Continuous-time HOCBF/VOCBF constraints do not automatically certify held-input intersample safety. Failed optimizations and unsuccessful tasks remain visible.

Each `run` output contains `trajectory.npz`, `events.json`, `summary.json` and `trajectory.png`. Position is in metres, time in seconds, angles in radians. Solver verification, script completion, safe execution and arrival are separate statements. These examples do not reproduce or update the prior five-method ranking.

## This method

Independent implementation of the paper equations on the original five-state acceleration-controlled unicycle, including the four CLFs, literal principal arctan chart, moving-obstacle derivative, soft slacks and side disjunctions. No verified author implementation or exact author-figure reproduction is claimed. Unspecified cost weights/gains and example geometry are declared.

The double integrator uses the same VO half-space geometry with Cartesian CLFs. First-order plants use an explicit dynamic extension: filter acceleration drives continuously ramped velocity commands. Their closed-loop state is augmented; they are not a direct first-order VOCBF theorem or a zero-order-held velocity controller.
