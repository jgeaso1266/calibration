package calibration

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	FakeSensor = resource.NewModel("jalen-monitor-cleaning", "calibration", "fake-sensor")
)

func init() {
	resource.RegisterComponent(sensor.API, FakeSensor,
		resource.Registration[sensor.Sensor, *SensorConfig]{
			Constructor: newCalibrationFakeSensor,
		},
	)
}

type SensorConfig struct {
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns three values:
//  1. Required dependencies: other resources that must exist for this resource to work.
//  2. Optional dependencies: other resources that may exist but are not required.
//  3. An error if any Config fields are missing or invalid.
//
// The `path` parameter indicates
// where this resource appears in the machine's JSON configuration
// (for example, "components.0"). You can use it in error messages
// to indicate which resource has a problem.
func (cfg *SensorConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

// calibrationFakeSensor simulates an ultrasonic sensor pointing at a virtual monitor
type calibrationFakeSensor struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *SensorConfig

	cancelCtx  context.Context
	cancelFunc func()

	// Virtual monitor definition
	monitorCenter   r3.Vector // Center point of monitor in world coordinates
	monitorNormal   r3.Vector // Normal vector (direction monitor faces)
	monitorWidth    float64   // Width in mm
	monitorHeight   float64   // Height in mm
	monitorUpVector r3.Vector // Which direction is "up" on the monitor
}

func newCalibrationFakeSensor(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	conf, err := resource.NativeConfig[*SensorConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewFakeSensor(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewFakeSensor(_ context.Context, deps resource.Dependencies, name resource.Name, conf *SensorConfig, logger logging.Logger) (sensor.Sensor, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &calibrationFakeSensor{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,

		// Monitor centered at X=250mm (middle width), Y=-400 (in front of arm), Z=200mm (middle height)
		monitorCenter: r3.Vector{X: 250, Y: -400, Z: 200},

		// Monitor with compound rotation: 15° around X-axis, then 10° around Y-axis
		// This tests orientation calculation with a non-axis-aligned plane
		monitorNormal: func() r3.Vector {
			// Start with normal (0, 1, 0)
			normal := r3.Vector{X: 0, Y: 1, Z: 0}

			// First rotate around X-axis by 15°
			angleX := 15.0 * math.Pi / 180.0
			cosX, sinX := math.Cos(angleX), math.Sin(angleX)
			normal = r3.Vector{
				X: normal.X,
				Y: normal.Y*cosX - normal.Z*sinX,
				Z: normal.Y*sinX + normal.Z*cosX,
			}

			// Then rotate around Y-axis by 10°
			angleY := 10.0 * math.Pi / 180.0
			cosY, sinY := math.Cos(angleY), math.Sin(angleY)
			normal = r3.Vector{
				X: normal.X*cosY + normal.Z*sinY,
				Y: normal.Y,
				Z: -normal.X*sinY + normal.Z*cosY,
			}

			return normal
		}(),

		// Monitor dimensions (typical desktop monitor)
		monitorWidth:  500, // mm
		monitorHeight: 300, // mm

		// Up vector (Z direction is up)
		monitorUpVector: r3.Vector{X: 0, Y: 0, Z: 1},
	}

	return s, nil
}

func (s *calibrationFakeSensor) Name() resource.Name {
	return s.name
}

// Readings implements the sensor.Sensor interface
// Returns a map with "distance" key containing the ultrasonic reading in meters
func (s *calibrationFakeSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	// Get current sensor position from extra parameters
	x, ok := extra["x"].(float64)
	y, ok2 := extra["y"].(float64)
	z, ok3 := extra["z"].(float64)
	ox, ok4 := extra["ox"].(float64)
	oy, ok5 := extra["oy"].(float64)
	oz, ok6 := extra["oz"].(float64)
	th, ok7 := extra["th"].(float64)

	var pose spatialmath.Pose
	if ok && ok2 && ok3 && ok4 && ok5 && ok6 && ok7 {
		pose = spatialmath.NewPose(
			r3.Vector{
				X: x,
				Y: y,
				Z: z,
			},
			&spatialmath.OrientationVector{
				OX:    ox,
				OY:    oy,
				OZ:    oz,
				Theta: th,
			},
		)

		s.logger.Debugf("sensor pose in world frame: %+v", pose)
	}

	sensorPos := pose.Point()
	orientation := pose.Orientation()
	orientationVector := orientation.OrientationVectorRadians()
	sensorDirWorld := r3.Vector{
		X: orientationVector.OX,
		Y: orientationVector.OY,
		Z: orientationVector.OZ,
	}

	// Calculate intersection with monitor plane (in mm)
	distanceMM, hit := s.rayIntersectsMonitor(sensorPos, sensorDirWorld)

	if hit {
		// Add some realistic noise (±2mm)
		noise := (math.Sin(float64(sensorPos.X+sensorPos.Z)) * 2.0)
		distanceMM += noise

		s.logger.Debugf("Fake sensor: HIT at distance %.2f mm (pos: %.1f,%.1f,%.1f)",
			distanceMM, sensorPos.X, sensorPos.Y, sensorPos.Z)
	} else {
		// No hit - return a large distance (out of range)
		distanceMM = 400.0 // Ultrasonic sensor max range in mm
		s.logger.Debugf("Fake sensor: MISS, returning max distance (pos: %.1f,%.1f,%.1f)",
			sensorPos.X, sensorPos.Y, sensorPos.Z)
	}

	// Convert to meters for return value
	distanceMeters := distanceMM / 1000.0

	return map[string]interface{}{
		"distance": distanceMeters,
	}, nil
}

// rayIntersectsMonitor checks if a ray from the sensor hits the virtual monitor
// Returns (distance, true) if hit, (0, false) if miss
func (s *calibrationFakeSensor) rayIntersectsMonitor(rayOrigin, rayDir r3.Vector) (float64, bool) {
	// Normalize ray direction
	rayDir = rayDir.Normalize()

	// Check if ray is parallel to plane (dot product near zero)
	denom := rayDir.Dot(s.monitorNormal)
	if math.Abs(denom) < 0.001 {
		return 0, false // Ray is parallel to plane
	}

	// Calculate intersection with infinite plane
	// Plane equation: (P - monitorCenter) · monitorNormal = 0
	// Ray equation: P = rayOrigin + t * rayDir
	// Solving: t = (monitorCenter - rayOrigin) · monitorNormal / (rayDir · monitorNormal)

	centerToOrigin := s.monitorCenter.Sub(rayOrigin)
	t := centerToOrigin.Dot(s.monitorNormal) / denom

	if t < 0 {
		return 0, false // Intersection is behind the sensor
	}

	// Calculate intersection point
	intersectionPoint := rayOrigin.Add(rayDir.Mul(t))

	// Check if intersection point is within monitor bounds
	// Create a 2D coordinate system on the monitor plane

	// Right vector (perpendicular to normal and up vector)
	rightVector := s.monitorUpVector.Cross(s.monitorNormal).Normalize()

	// Recalculate up vector to ensure orthogonality
	upVector := s.monitorNormal.Cross(rightVector).Normalize()

	// Vector from monitor center to intersection point
	toIntersection := intersectionPoint.Sub(s.monitorCenter)

	// Project onto the monitor's 2D coordinate system
	u := toIntersection.Dot(rightVector) // Horizontal distance from center
	v := toIntersection.Dot(upVector)    // Vertical distance from center

	// Check if within bounds
	halfWidth := s.monitorWidth / 2
	halfHeight := s.monitorHeight / 2

	if math.Abs(u) <= halfWidth && math.Abs(v) <= halfHeight {
		// Hit! Return distance
		return t, true
	}

	// Intersection is outside monitor bounds
	return 0, false
}

func (s *calibrationFakeSensor) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *calibrationFakeSensor) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
