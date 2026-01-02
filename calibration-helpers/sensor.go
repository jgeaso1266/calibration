package calibrationhelpers

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

// Point3D represents a 3D point in world space
type Point3D struct {
	X, Y, Z float64
}

// SensorReading encapsulates a complete sensor measurement
type SensorReading struct {
	Depth        float64
	SurfacePoint Point3D
	SensorPose   spatialmath.Pose
}

// GetSurfacePoint performs the complete sensor reading workflow:
// 1. Get sensor pose in world frame
// 2. Read depth with pose parameters
// 3. Calculate actual surface point in world coordinates
func GetSurfacePoint(ctx context.Context, logger logging.Logger, fs framesystem.RobotFrameSystem,
	sensor sensor.Sensor, worldFrame string) (SensorReading, error) {

	// Get sensor pose in world coordinates
	sensorPoseInFrame, err := fs.GetPose(ctx, sensor.Name().Name, worldFrame, nil, nil)
	if err != nil {
		return SensorReading{}, fmt.Errorf("failed to get sensor pose: %w", err)
	}

	sensorPose := sensorPoseInFrame.Pose()

	// Read depth with current pose parameters
	depthReading, err := sensor.Readings(ctx, map[string]any{
		"x":  sensorPose.Point().X,
		"y":  sensorPose.Point().Y,
		"z":  sensorPose.Point().Z,
		"ox": sensorPose.Orientation().OrientationVectorRadians().OX,
		"oy": sensorPose.Orientation().OrientationVectorRadians().OY,
		"oz": sensorPose.Orientation().OrientationVectorRadians().OZ,
		"th": sensorPose.Orientation().OrientationVectorRadians().Theta,
	})
	if err != nil {
		return SensorReading{}, fmt.Errorf("failed to get sensor reading: %w", err)
	}

	// Sensor returns distance in meters, convert to millimeters
	depthMeters := depthFloat(depthReading)
	depth := depthMeters * 1000.0

	// Calculate actual surface point
	surfacePoint, err := calculateWorldPoint(ctx, logger, fs, sensor.Name().Name, depth)
	if err != nil {
		return SensorReading{}, fmt.Errorf("failed to calculate world point: %w", err)
	}

	return SensorReading{
		Depth:        depth,
		SurfacePoint: surfacePoint,
		SensorPose:   sensorPose,
	}, nil
}

// depthFloat extracts the depth value from sensor readings
func depthFloat(d map[string]interface{}) float64 {
	return d["distance"].(float64)
}

// calculateWorldPoint takes the sensor position and depth reading and returns the actual point on the monitor surface
// This assumes the sensor is pointing in the direction of its orientation
func calculateWorldPoint(ctx context.Context, logger logging.Logger, fs framesystem.RobotFrameSystem, sensorName string, depth float64) (Point3D, error) {
	// Get sensor pose in world coordinates (frame system handles all transformations)
	sensorPoseInFrame, err := fs.GetPose(ctx, sensorName, "world", nil, nil)
	if err != nil {
		return Point3D{}, fmt.Errorf("failed to get sensor pose in world frame: %w", err)
	}

	sensorPose := sensorPoseInFrame.Pose()
	sensorPos := sensorPose.Point()

	// Extract the direction vector from the orientation
	orientation := sensorPose.Orientation()
	orientVec := orientation.OrientationVectorRadians()

	// The orientation vector gives us the direction the sensor points
	sensorDirX := orientVec.OX
	sensorDirY := orientVec.OY
	sensorDirZ := orientVec.OZ

	// Normalize the direction (should already be normalized, but make sure)
	length := (sensorDirX*sensorDirX + sensorDirY*sensorDirY + sensorDirZ*sensorDirZ)
	if length > 0 {
		invLength := 1.0 / length
		sensorDirX *= invLength
		sensorDirY *= invLength
		sensorDirZ *= invLength
	}

	// Calculate the actual surface point in world coordinates
	// Surface point = sensor position + (depth * sensor direction)
	surfacePoint := Point3D{
		X: sensorPos.X + depth*sensorDirX,
		Y: sensorPos.Y + depth*sensorDirY,
		Z: sensorPos.Z + depth*sensorDirZ,
	}

	logger.Debugf("Sensor world pos=(%f,%f,%f), dir=(%f,%f,%f), depth=%f, surface=(%f,%f,%f)",
		sensorPos.X, sensorPos.Y, sensorPos.Z,
		sensorDirX, sensorDirY, sensorDirZ,
		depth,
		surfacePoint.X, surfacePoint.Y, surfacePoint.Z)

	return surfacePoint, nil
}
