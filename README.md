# Module calibration

This module provides components for calibrating and detecting monitor surfaces using ultrasonic sensors. It includes a simulated sensor for testing and a monitor calibration component that performs automated calibration routines.

## Model jalen-monitor-cleaning:calibration:fake-sensor

A simulated ultrasonic sensor that models ray-casting against a virtual monitor plane. This component is useful for testing calibration algorithms without physical hardware.

The virtual monitor has a compound rotation (15° around X-axis, 10° around Y-axis) to test orientation calculations with non-axis-aligned planes.

### Configuration

This model requires the following configuration attributes:

#### Attributes

The following attributes are available for this model:

| Name     | Type   | Inclusion | Description                |
|----------|--------|-----------|----------------------------|
| `arm`    | string | Required  | Name of the arm component |
| `gantry` | string | Required  | Name of the gantry component |

#### Example Configuration

```json
{
  "arm": "my-arm",
  "gantry": "my-gantry"
}
```

### Readings

The sensor returns distance readings in **centimeters** through the standard `Readings()` method. It automatically calculates the sensor pose using the frame system based on the current arm and gantry positions.

**Returns:**
```json
{
  "distance": 24.53
}
```

The sensor simulates realistic behavior:
- Returns actual distance (in centimeters) when the ray hits the virtual monitor surface
- Returns 40.0 cm (max range, 400mm) when the ray misses the monitor
- Adds ±2mm noise to simulate real sensor readings

## Model jalen-monitor-cleaning:calibration:monitor-calibration

A generic component that performs automated monitor surface calibration using an arm, gantry, and ultrasonic sensor. The calibration routine detects the monitor's position, orientation, and boundaries.

> [!CAUTION]
> Only tested in specific arm/gantry/sensor configurations, but is possibly generalizable. Diagram CS

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "arm": "<arm-name>",
  "gantry": "<gantry-name>",
  "sensor": "<sensor-name>"
}
```

#### Attributes

The following attributes are available for this model:

| Name     | Type   | Inclusion | Description                |
|----------|--------|-----------|----------------------------|
| `arm`    | string | Required  | Name of the arm component |
| `gantry` | string | Required  | Name of the gantry component for horizontal movement |
| `sensor` | string | Required  | Name of the ultrasonic sensor component |

#### Example Configuration

```json
{
  "arm": "my-arm",
  "gantry": "my-gantry",
  "sensor": "ultrasonic-1"
}
```

### DoCommand

The calibration component provides a calibration routine through `DoCommand`. Call with any command to start calibration.

The calibration process:
1. Centers the gantry X-axis
2. Scans Z-axis to collect surface points
3. Scans X-axis (gantry) to collect surface points
4. Constructs a plane equation from the collected points
5. Finds top and bottom edges (Z limits)
6. Finds left and right edges (X limits)
7. Returns a visualization configuration with monitor position and orientation

**Returns:**
```json
{
  "name": "calibrated-monitor",
  "type": "generic",
  "model": "fake",
  "frame": {
    "parent": "world",
    "translation": {"x": 250.0, "y": -380.5, "z": 200.0},
    "orientation": {
      "type": "quaternion",
      "value": {"x": 0.123, "y": 0.456, "z": 0.789, "w": 0.321}
    },
    "geometry": {"type": "box", "x": 500.0, "y": 1.0, "z": 300.0}
  }
}
```

Returns a map containing visualization configuration for the detected monitor. To view the monitor in the Viz tab, either copy the configuration json into your config, or add a generic component to your machine, add a frame to that component and copy the frame config.
