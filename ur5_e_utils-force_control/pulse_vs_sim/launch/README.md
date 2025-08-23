# Launch files for simulated tests
## Introduction
This document relates the components of each launch file to the corresponding component of the visual servoing system being tested.

## Point source motion controllers
- `p0`: Fixed point source
- `p1`: Point source moving in elliptical patterns in xz- and xy-planes

## Point source localization systems
- `s0`: Channel data not published, simulated detections generated using TF information of point source and probe frames
- `s1`: Channel data published, simulated detections generated independent of channel data using TF information of point source and probe frames
- `s2`: Channel data published, 2D point source localization system (Gubbi and Bell, IEEE ICRA 2021) used to determine lateral and axial point source location information
- `sa`: Channel data published, 3D point source localization system (System A, Gubbi et al., IEEE TUFFC 2024?) used to determine three-dimensional point source location information (with elevation symmetry)
- `sd`: Channel data published, 3D point source localization system (System D, Gubbi et al., IEEE TUFFC 2024?) used to determine three-dimensional point source location information (with elevation symmetry)

## Estimation filters
- `e1`: Multi-track linear kalman filter tracking position candidates with elevation symmetry

## Guidance algorithms
- `g0`: Commanded probe pose matching current probe pose (i.e., no commanded movement)
- `g1`: No commanded tracking, but spiral search motion (Gubbi and Bell, IEEE ICRA 2021)
- `g2`: Commanded probe motion along lateral dimension of probe with spiral search algorithm (Gubbi and Bell, IEEE ICRA 2021)
- `g3`: Commanded probe translation along lateral and elevation dimensions of probe and rotation about axial dimension with spiral search algorithm

## Control systems
- `c0`: Probe held in initial pose regardless of commanded probe motion
- `c1`: Probe moved using PID controller to follow commanded probe motion

E.g., `v2_p0_s0_e1_g3_c1.launch` should define a visual servoing system tracking a fixed source with no channel data being published. The detections are simulated using TF information between the point source and probe frames. A multi-track LKF is used to filter these detections. The probe is commanded to move in the translational x-, translational y-, and rotational z-dimensions, and moved in each of these dimensions using an independent PID controller.
