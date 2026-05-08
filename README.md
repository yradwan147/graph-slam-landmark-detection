# Landmark Detection & Tracking (Graph SLAM)

Final project for Udacity's *Computer Vision* nanodegree (nd891). A 2-D
Graph-SLAM implementation: a robot wanders a square world with motion and
measurement noise, observes landmarks within a sensor range, and we recover
the entire trajectory **and** the landmark positions in closed form by
inverting the constraint matrix Ω.

## Notebooks

```
1. Robot Moving and Sensing.ipynb       — implements robot.move and robot.sense
2. Omega and Xi, Constraints.ipynb     — small worked example of the matrices
3. Landmark Detection and Tracking.ipynb — full slam() + tests + visualisation
robot_class.py                          — final robot class with sense filled in
helpers.py                              — pre-supplied data generation
```

## What I implemented

* `robot.sense()` — iterates the landmarks, computes (`dx`, `dy`) with
  i.i.d. uniform noise of magnitude `measurement_noise`, and returns only
  the landmarks that fall within `measurement_range` (or all of them if
  `measurement_range == -1`).
* `initialize_constraints(N, num_landmarks, world_size)` — builds Ω as a
  `2(N + num_landmarks)`-square zero matrix and ξ as a column vector,
  anchoring the starting pose at the world centre with strength 1.
* `slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise)`
  — walks every (measurements, motion) pair in `data`, weights each
  constraint by 1/noise, and returns `μ = Ω⁻¹ ξ`.

## Running

```bash
pip install numpy matplotlib jupyter
jupyter notebook
# Then "Restart & Run All" on each of the three numbered notebooks.
```

## Standing-out work

* The `slam()` updates batch all eight Ω cells per measurement / motion
  step into a small list-of-tuples, so the math is concise and all the
  signs are visible at a glance.
* `sense()` collapses both the `measurement_range == -1` ("see all") and
  the bounded case into a single boolean expression, removing duplicate
  branching.
* The robot class works for noiseless tests too — passing
  `motion_noise=0` and `measurement_noise=0` makes every step
  deterministic.

## License

Educational submission for Udacity nd891. Starter scaffold © Udacity.
