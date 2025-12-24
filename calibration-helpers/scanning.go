package calibrationhelpers

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

// CenterGantry centers the gantry at the midpoint of its range
func CenterGantry(ctx context.Context, gantry gantry.Gantry, config ScanningConfig) (float64, error) {
	gantryLengths, err := gantry.Lengths(ctx, nil)
	if err != nil {
		return 0, fmt.Errorf("failed to get gantry lengths: %w", err)
	}

	centerPosition := gantryLengths[0] / 2

	if err := gantry.MoveToPosition(ctx, []float64{centerPosition}, []float64{config.GantrySpeed}, nil); err != nil {
		return 0, fmt.Errorf("failed to center gantry: %w", err)
	}

	return centerPosition, nil
}

// PerformZScan scans vertically along the Z-axis, collecting surface points
func PerformZScan(ctx context.Context, logger logging.Logger, fs framesystem.RobotFrameSystem,
	sensor sensor.Sensor, arm arm.Arm, config CalibrationConfig) ([]Point3D, error) {

	var points []Point3D

	// Reset arm to starting position
	if err := arm.MoveToJointPositions(ctx, config.ArmPositions.Home, nil); err != nil {
		return nil, fmt.Errorf("failed to reset arm: %w", err)
	}

	for i := 0; i < config.Scanning.ZNumSteps; i++ {
		// Get surface point
		reading, err := GetSurfacePoint(ctx, logger, fs, sensor, config.Hardware.WorldFrame)
		if err != nil {
			return nil, fmt.Errorf("failed to get sensor reading at step %d: %w", i, err)
		}

		points = append(points, reading.SurfacePoint)
		logger.Infof("Z scan point %d: depth=%f, surface=(%f, %f, %f)",
			i+1, reading.Depth, reading.SurfacePoint.X, reading.SurfacePoint.Y, reading.SurfacePoint.Z)

		// Move up for next reading (except after last point)
		if i < config.Scanning.ZNumSteps-1 {
			armPose, err := arm.EndPosition(ctx, nil)
			if err != nil {
				return nil, fmt.Errorf("failed to get arm position: %w", err)
			}

			nextPose := spatialmath.NewPose(
				r3.Vector{
					X: armPose.Point().X,
					Y: armPose.Point().Y,
					Z: armPose.Point().Z + config.Scanning.ZStepSize,
				},
				armPose.Orientation(),
			)

			if err := arm.MoveToPosition(ctx, nextPose, nil); err != nil {
				return nil, fmt.Errorf("failed to move arm up to pose %+v: %w", nextPose.Point(), err)
			}
		}
	}

	return points, nil
}

// PerformXScan scans horizontally along the X-axis (gantry), collecting surface points
func PerformXScan(ctx context.Context, logger logging.Logger, fs framesystem.RobotFrameSystem,
	sensor sensor.Sensor, arm arm.Arm, gantry gantry.Gantry,
	config CalibrationConfig) ([]Point3D, error) {

	var points []Point3D

	// Reset arm to starting position
	if err := arm.MoveToJointPositions(ctx, config.ArmPositions.Home, nil); err != nil {
		return nil, fmt.Errorf("failed to reset arm: %w", err)
	}

	// Get gantry range
	gantryLengths, err := gantry.Lengths(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get gantry lengths: %w", err)
	}

	// Scan across full gantry range
	xStepSize := gantryLengths[0] / float64(config.Scanning.XNumSteps-1)
	startXPosition := 0.0

	for i := 0; i < config.Scanning.XNumSteps; i++ {
		// Move gantry to position
		xPosition := startXPosition + float64(i)*xStepSize
		if err := gantry.MoveToPosition(ctx, []float64{xPosition}, []float64{config.Scanning.GantrySpeed}, nil); err != nil {
			return nil, fmt.Errorf("failed to move gantry: %w", err)
		}

		// Get surface point
		reading, err := GetSurfacePoint(ctx, logger, fs, sensor, config.Hardware.WorldFrame)
		if err != nil {
			return nil, fmt.Errorf("failed to get sensor reading at step %d: %w", i, err)
		}

		points = append(points, reading.SurfacePoint)
		logger.Infof("X scan point %d: gantry=%f, depth=%f, surface=(%f, %f, %f)",
			i+1, xPosition, reading.Depth, reading.SurfacePoint.X, reading.SurfacePoint.Y, reading.SurfacePoint.Z)
	}

	return points, nil
}
