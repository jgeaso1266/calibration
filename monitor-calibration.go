package calibration

import (
	calibrationhelpers "calibration/calibration-helpers"
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
)

var (
	MonitorCalibration = resource.NewModel("jalen-monitor-cleaning", "calibration", "monitor-calibration")
)

func init() {
	resource.RegisterComponent(generic.API, MonitorCalibration,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newMonitorCalibration,
		},
	)
}

type Config struct {
	Arm        string `json:"arm"`
	Gantry     string `json:"gantry"`
	Sensor     string `json:"sensor"`
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
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("missing 'arm' field in %s", path)
	}
	if cfg.Gantry == "" {
		return nil, nil, fmt.Errorf("missing 'gantry' field in %s", path)
	}
	if cfg.Sensor == "" {
		return nil, nil, fmt.Errorf("missing 'sensor' field in %s", path)
	}
	return []string{cfg.Arm, cfg.Gantry, cfg.Sensor}, nil, nil
}

// monitorCalibration simulates an ultrasonic sensor pointing at a virtual monitor
type monitorCalibration struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()

	arm    arm.Arm
	gantry gantry.Gantry
	sensor sensor.Sensor
	calibrationConfig calibrationhelpers.CalibrationConfig

	fs framesystem.RobotFrameSystem

	doCommandLock sync.Mutex
}

func newMonitorCalibration(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewMonitorCalibration(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewMonitorCalibration(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	var err error

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &monitorCalibration{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	s.arm, err = arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, err
	}

	s.gantry, err = gantry.FromProvider(deps, conf.Gantry)
	if err != nil {
		return nil, err
	}

	s.sensor, err = sensor.FromProvider(deps, conf.Sensor)
	if err != nil {
		return nil, err
	}

	s.fs, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}

	s.calibrationConfig = calibrationhelpers.CalibrationConfig{
		Hardware: calibrationhelpers.HardwareConfig{
			GripperWidth: 106.4, // mm - default gripper width
			WorldFrame:   "world",
		},
		Scanning: calibrationhelpers.ScanningConfig{
			ZStepSize:   10.0, // mm
			ZNumSteps:   10,
			XNumSteps:   10,
			GantrySpeed: 50.0, // mm/sec
		},
		Detection: calibrationhelpers.DetectionConfig{
			PlaneThreshold: 20.0, // mm
			EdgeStepSize:   10.0, // mm
		},
		ArmPositions: calibrationhelpers.DefaultArmPositions,
	}

	return s, nil
}

func (s *monitorCalibration) Name() resource.Name {
	return s.name
}

func (s *monitorCalibration) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.doCommandLock.Lock()
	defer s.doCommandLock.Unlock()

	s.logger.Info("=== STARTING CALIBRATION ===")

	// STEP 1: Center the X axis (gantry position)
	s.logger.Info("Step 1: Centering X axis (gantry)...")
	centerPosition, err := calibrationhelpers.CenterGantry(ctx, s.gantry, s.calibrationConfig.Scanning)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("Moving gantry to center position: %f mm", centerPosition)
	s.logger.Info("✓ Gantry centered")

	// STEP 2: Scan Z axis to collect points that should form a straight line on the monitor plane
	s.logger.Info("Step 2: Scanning Z axis to detect straight line...")
	zScanPoints, err := calibrationhelpers.PerformZScan(ctx, s.logger, s.fs, s.sensor, s.arm, s.calibrationConfig)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("✓ Collected %d points along Z axis", len(zScanPoints))

	// Fit a line to the Z scan points
	_, zPoint2, err := calibrationhelpers.FitLineToPoints(s.logger, zScanPoints)
	if err != nil {
		return nil, fmt.Errorf("failed to fit line to Z scan: %w", err)
	}
	s.logger.Info("✓ Fitted line to Z scan points")

	// STEP 3: Scan X axis (move gantry) to collect points that form another straight line
	s.logger.Info("Step 3: Scanning X axis (gantry) to detect straight line...")
	xScanPoints, err := calibrationhelpers.PerformXScan(ctx, s.logger, s.fs, s.sensor, s.arm, s.gantry, s.calibrationConfig)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("✓ Collected %d points along X axis", len(xScanPoints))

	// Fit a line to the X scan points
	xPoint1, xPoint2, err := calibrationhelpers.FitLineToPoints(s.logger, xScanPoints)
	if err != nil {
		return nil, fmt.Errorf("failed to fit line to X scan: %w", err)
	}
	s.logger.Info("✓ Fitted line to X scan points")

	// STEP 4: Use those 3 points to construct a plane
	s.logger.Info("Step 4: Constructing plane from 3 points...")
	plane, err := calibrationhelpers.CalculatePlaneFrom3Points(zPoint2, xPoint1, xPoint2)
	if err != nil {
		return nil, fmt.Errorf("failed to calculate plane: %w", err)
	}
	s.logger.Infof("✓ Plane equation: %f*x + %f*y + %f*z = %f", plane.A, plane.B, plane.C, plane.D)

	// STEP 5: Find Z limits (top and bottom edges)
	s.logger.Info("Step 5: Finding Z limits (top and bottom edges)...")

	// Center gantry again for edge detection
	_, err = calibrationhelpers.CenterGantry(ctx, s.gantry, s.calibrationConfig.Scanning)
	if err != nil {
		return nil, err
	}

	// Reset arm position for bottom edge search
	err = s.arm.MoveToJointPositions(ctx, s.calibrationConfig.ArmPositions.BottomScan, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to reset arm: %w", err)
	}

	s.logger.Info("Searching for bottom edge...")
	bottomResult, err := calibrationhelpers.FindVerticalEdge(ctx, s.logger, s.fs, s.sensor, s.arm, plane, -1, s.calibrationConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to find bottom edge: %w", err)
	}

	// Reset arm and find top edge
	err = s.arm.MoveToJointPositions(ctx, s.calibrationConfig.ArmPositions.TopScan, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to reset arm: %w", err)
	}

	s.logger.Info("Searching for top edge...")
	topResult, err := calibrationhelpers.FindVerticalEdge(ctx, s.logger, s.fs, s.sensor, s.arm, plane, 1, s.calibrationConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to find top edge: %w", err)
	}

	s.logger.Infof("✓ Z limits found: bottom=%f, top=%f, height=%f",
		bottomResult.SurfacePoint.Z, topResult.SurfacePoint.Z, topResult.SurfacePoint.Z-bottomResult.SurfacePoint.Z)

	// STEP 6: Find X limits (left and right edges)
	s.logger.Info("Step 6: Finding X limits (left and right edges)...")

	// Reset arm to middle position
	err = s.arm.MoveToJointPositions(ctx, s.calibrationConfig.ArmPositions.Home, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to reset arm: %w", err)
	}

	// Get gantry lengths for edge detection
	gantryLengths, err := s.gantry.Lengths(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get gantry lengths: %w", err)
	}

	s.logger.Info("Searching for left edge...")
	leftResult, err := calibrationhelpers.FindHorizontalEdge(ctx, s.logger, s.fs, s.sensor, s.gantry, plane, gantryLengths, 1, s.calibrationConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to find left edge: %w", err)
	}

	s.logger.Info("Searching for right edge...")
	rightResult, err := calibrationhelpers.FindHorizontalEdge(ctx, s.logger, s.fs, s.sensor, s.gantry, plane, gantryLengths, -1, s.calibrationConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to find right edge: %w", err)
	}

	s.logger.Infof("✓ X limits found: left=%.1f, right=%.1f, width=%.1f",
		leftResult.SurfacePoint.X, rightResult.SurfacePoint.X, leftResult.SurfacePoint.X-rightResult.SurfacePoint.X)

	// STEP 7: Inferred rectangle bounds
	s.logger.Info("Step 7: Inferred rectangle bounds:")
	s.logger.Infof("  Bottom-left: (surface X=%.1f, Z=%.1f)", leftResult.SurfacePoint.X, bottomResult.SurfacePoint.Z)
	s.logger.Infof("  Bottom-right: (surface X=%.1f, Z=%.1f)", rightResult.SurfacePoint.X, bottomResult.SurfacePoint.Z)
	s.logger.Infof("  Top-left: (surface X=%.1f, Z=%.1f)", leftResult.SurfacePoint.X, topResult.SurfacePoint.Z)
	s.logger.Infof("  Top-right: (surface X=%.1f, Z=%.1f)", rightResult.SurfacePoint.X, topResult.SurfacePoint.Z)
	s.logger.Infof("  Dimensions: width=%.1f mm, height=%.1f mm",
		leftResult.SurfacePoint.X-rightResult.SurfacePoint.X, topResult.SurfacePoint.Z-bottomResult.SurfacePoint.Z)

	// Create calibration result
	result := calibrationhelpers.CalibrationResult{
		Plane:   plane,
		BottomZ: bottomResult.SurfacePoint.Z,
		TopZ:    topResult.SurfacePoint.Z,
		LeftX:   leftResult.SurfacePoint.X,
		RightX:  rightResult.SurfacePoint.X,
		XPoint1: xPoint1,
		XPoint2: xPoint2,
		ZPoint1: zPoint2,
	}

	// Generate visualization and print results
	vizConfig := calibrationhelpers.GenerateVisualizationConfig(s.logger, result, s.calibrationConfig.Hardware.WorldFrame)

	return vizConfig, nil
}

func (s *monitorCalibration) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
