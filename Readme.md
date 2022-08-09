# Algal Bloom Tracking

Required packages:
- [smarc_msgs fork](https://github.com/matthew-william-lock/smarc_msgs)

## Branches

- [main](https://github.com/matthew-william-lock/algalbloom-tracking) - purely directional control along front
- [zig-zag] - zig-zag pattern along front

## Recording ros bags

Example of recording odom and waypoint locations
```bash
rosbag record -O bag_name /sam/algae_tracking/chlorophyll_sampling /sam/algae_tracking/gradient /sam/algae_tracking/vp /sam/dr/lat_lon /sam/dr/odom /sam/smarc_bt/live_wp/wp
```

## Plotting SAM Mission Trajectory

After running the mission, data will be recorded to ```~/catkin_ws/install/share/smarc_algal_bloom_tracking/output/mission.h5```. This data can be plotted copying the file to ```~/catkin_ws/src/smarc_algal_bloom_tracking/output/mission.h5```, then navigating to ```~/catkin_ws/src/smarc_algal_bloom_tracking```and running:

```python
python scripts/plot_traj.py output/mission.h5 --ref --grad_error
```

*The plotting script is adapted from [Alexandre Rocha](https://github.com/avrocha)*
