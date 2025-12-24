package calibration

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
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

type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type MonitorConfig struct {
	Center *Vector3 `json:"center,omitempty"` // mm - center point of monitor
	Normal *Vector3 `json:"normal,omitempty"` // direction vector - which way monitor faces
	Up     *Vector3 `json:"up,omitempty"`     // direction vector - which way is "up"
	Width  float64  `json:"width"`            // mm
	Height float64  `json:"height"`           // mm
}

type SensorConfig struct {
	Arm     string         `json:"arm"`
	Gantry  string         `json:"gantry"`
	Monitor *MonitorConfig `json:"monitor,omitempty"`
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
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("missing 'arm' field in %s", path)
	}
	if cfg.Gantry == "" {
		return nil, nil, fmt.Errorf("missing 'gantry' field in %s", path)
	}

	return []string{cfg.Arm, cfg.Gantry}, nil, nil
}

// calibrationFakeSensor simulates an ultrasonic sensor pointing at a virtual monitor
type calibrationFakeSensor struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *SensorConfig

	cancelCtx  context.Context
	cancelFunc func()

	arm    arm.Arm
	gantry gantry.Gantry
	fs     framesystem.RobotFrameSystem

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
	var err error
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	// Apply defaults for monitor configuration if not specified
	if conf.Monitor == nil {
		conf.Monitor = &MonitorConfig{}
	}
	if conf.Monitor.Center == nil {
		conf.Monitor.Center = &Vector3{X: 250, Y: -400, Z: 200}
	}
	if conf.Monitor.Normal == nil {
		conf.Monitor.Normal = &Vector3{X: 0, Y: 1, Z: 0}
	}
	if conf.Monitor.Width == 0 {
		conf.Monitor.Width = 500
	}
	if conf.Monitor.Height == 0 {
		conf.Monitor.Height = 300
	}
	if conf.Monitor.Up == nil {
		conf.Monitor.Up = &Vector3{X: 0, Y: 0, Z: 1}
	}

	s := &calibrationFakeSensor{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,

		// Monitor configuration from config
		monitorCenter:   r3.Vector{X: conf.Monitor.Center.X, Y: conf.Monitor.Center.Y, Z: conf.Monitor.Center.Z},
		monitorNormal:   r3.Vector{X: conf.Monitor.Normal.X, Y: conf.Monitor.Normal.Y, Z: conf.Monitor.Normal.Z},
		monitorWidth:    conf.Monitor.Width,
		monitorHeight:   conf.Monitor.Height,
		monitorUpVector: r3.Vector{X: conf.Monitor.Up.X, Y: conf.Monitor.Up.Y, Z: conf.Monitor.Up.Z},
	}

	logger.Infof("Fake sensor monitor config: center=%+v, normal=%+v, up=%+v, w=%.1f, h=%.1f",
		s.monitorCenter, s.monitorNormal, s.monitorUpVector, s.monitorWidth, s.monitorHeight)

	s.arm, err = arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, err
	}

	s.gantry, err = gantry.FromProvider(deps, conf.Gantry)
	if err != nil {
		return nil, err
	}

	s.fs, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}

	return s, nil
}

func (s *calibrationFakeSensor) Name() resource.Name {
	return s.name
}

// Readings implements the sensor.Sensor interface
// Returns a map with "distance" key containing the ultrasonic reading in centimeters
func (s *calibrationFakeSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	// Get sensor pose in world coordinates using the frame system
	sensorPoseInFrame, err := s.fs.GetPose(ctx, s.name.Name, "world", nil, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get sensor pose: %w", err)
	}

	pose := sensorPoseInFrame.Pose()
	s.logger.Debugf("sensor pose in world frame: %+v", pose)

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
		distanceMM = 4000.0 // Ultrasonic sensor max range in mm
		s.logger.Debugf("Fake sensor: MISS, returning max distance (pos: %.1f,%.1f,%.1f)",
			sensorPos.X, sensorPos.Y, sensorPos.Z)
	}

	// Convert to centimeters for return value (real ultrasonic sensors return cm)
	distanceCM := distanceMM / 10.0

	return map[string]interface{}{
		"distance": distanceCM,
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
