# Algal Bloom Tracking

Required packages:
- [smarc_msgs fork](https://github.com/matthew-william-lock/smarc_msgs)

## Recording ros bags

Example of recording odom and waypoint locations
```bash
rosbag record -O bag_name /sam/dr/lat_lon /sam/dr/odom /sam/smarc_bt/live_wp/wp /sam/algae_tracking/chlorophyll_sampling
```