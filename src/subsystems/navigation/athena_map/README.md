# DEM Costmap Converter

This package converts Digital Elevation Model (DEM) TIFF files into nav2-compatible costmaps based on terrain slope analysis.

## Features

- **TIFF DEM Support**: Reads standard TIFF DEM files with elevation data
- **Slope-based Costing**: Calculates terrain slopes and assigns costs accordingly
- **Nav2 Compatible**: Follows nav2 costmap2d architecture (0-255 cost values)
- **Configurable Parameters**: Adjustable resolution, slope thresholds, and map origin


## Cost Assignment Strategy

The converter assigns costs based on terrain slope:

- **0-5 degrees**: Low cost (0-49) - Easy traversal
- **5-10 degrees**: Medium cost (50-149) - Moderate difficulty  
- **10-15 degrees**: High cost (150-252) - Difficult traversal
- **15+ degrees**: Lethal (255) - Impassable terrain

## Usage

### Basic Usage with DEM File

```bash
# Launch with your DEM file
ros2 launch apollo_map dem_costmap.launch.py dem_file_path:=/path/to/your/dem.tif

# With custom parameters
timeout 10s ros2 launch apollo_map dem_costmap.launch.py \
    dem_file_path:=src/athena_nav/athena_map/maps/north_site800m.tif \
    map_resolution:=0.5 \
    max_passable_slope_degrees:=15.0 \
    origin_x:=518376.0\
    origin_y:=4253272.0
```



### Direct Node Execution

```bash
# Run node directly with parameters
ros2 run apollo_map map_node \
    --ros-args \
    -p dem_file_path:=/path/to/your/dem.tif \
    -p map_resolution:=1.0 \
    -p max_passable_slope_degrees:=15.0
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dem_file_path` | string | "" | Path to DEM TIFF file |
| `map_resolution` | double | 1.0 | Map resolution (meters per pixel) |
| `max_passable_slope_degrees` | double | 15.0 | Maximum slope for passable terrain |
| `output_frame` | string | "map" | Frame ID for output costmap |
| `origin_x` | double | 0.0 | X coordinate of map origin |
| `origin_y` | double | 0.0 | Y coordinate of map origin |

## Topics

### Published
- `/costmap` (`nav_msgs/OccupancyGrid`): Generated costmap

## DEM File Requirements

- **Format**: TIFF (GeoTIFF recommended)
- **Data Type**: 16-bit or 32-bit elevation values
- **Coverage**: Should cover your planned operational area
- **Resolution**: Higher resolution DEMs produce more detailed costmaps

## Integration with Nav2

The generated costmap can be used with nav2 as a static layer:

```yaml
# Example nav2 costmap configuration
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  use_sim_time: True
  resolution: 1.0
  plugins: ["static_layer"]
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: true
    subscribe_to_updates: true
```

## Example: 800x800m Area Processing

For an 800x800 meter area with 1m resolution:
- Input: 800x800 pixel DEM TIFF
- Output: 800x800 cell costmap 
- Memory usage: ~640KB for costmap data
- Processing time: <1 second on modern hardware

## Building

Make sure you have the required dependencies:

```bash
# Install OpenCV for ROS2
sudo apt install ros-humble-cv-bridge ros-humble-opencv-contrib-python

# Build the package
cd ~/ros_ws
colcon build --packages-select apollo_map
```

## Troubleshooting

### Common Issues

1. **"Failed to load DEM file"**
   - Check file path is correct
   - Ensure TIFF file is readable
   - Verify OpenCV TIFF support

2. **"No DEM file path provided"**
   - Node will not publish costmap
   - Set `dem_file_path` parameter to valid DEM file

3. **High memory usage**
   - Reduce DEM resolution
   - Process smaller areas
   - Increase `map_resolution` parameter

### Debugging

Enable debug output:
```bash
ros2 run apollo_map map_node --ros-args --log-level debug
```

## Theory: Slope Calculation

The slope calculation uses Sobel operators to compute elevation gradients:

1. **Gradient Calculation**: `∇z = (∂z/∂x, ∂z/∂y)`
2. **Slope Magnitude**: `slope = atan(√(∂z/∂x)² + (∂z/∂y)²)`
3. **Angle Conversion**: Convert from radians to degrees
4. **Cost Mapping**: Apply piecewise linear cost function

This method provides robust slope estimation for terrain analysis. 