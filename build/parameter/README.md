Parameter setups for CFS 2D motion planning example

- `parameters_2d.txt` set up `max_iter`(maximum numbers of subproblem), `converge_trh` (threshold for convergence), `margin` (clearence away from obstacle), `v_max` (maximum velocity), `weight_self` (weight for penalizing input).
- `env.txt` set up obstacle in the following formate:
```
(number of obstacles)
(x obs1)
(y obs1)
(radius obs1)
(x obs2)
(y obs2)
(radius obs2)
.
.
.
```
- `ref_init_goal.txt` set up planning problem in the following formate:
```
(x initial) 
(y initial)
(x goal)
(y goal)
(Horizon)
```
