# Algal Bloom Tracking

Required packages:
- [smarc_msgs fork](https://github.com/matthew-william-lock/smarc_msgs)

## Recording ros bags

Example of recording odom and waypoint locations
```bash
rosbag record -O bag_name /sam/dr/odom /sam/algae_farm/wp
```