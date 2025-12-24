package calibrationhelpers

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

// Plane represents a plane equation: Ax + By + Cz = D
type Plane struct {
	A, B, C, D float64
}

// EdgeSearchResult contains the result of an edge search
type EdgeSearchResult struct {
	SurfacePoint Point3D
	Found        bool
}

// FindVerticalEdge searches for an edge by moving vertically (up or down)
// zDirection: +1 for up (top edge), -1 for down (bottom edge)
func FindVerticalEdge(ctx context.Context, logger logging.Logger, fs framesystem.RobotFrameSystem,
	sensor sensor.Sensor, arm arm.Arm, plane Plane, zDirection int, config CalibrationConfig) (EdgeSearchResult, error) {
	var edgeName string
	if zDirection == 1 {
		edgeName = "top"
	} else {
		edgeName = "bottom"
	}
	var result EdgeSearchResult

	for {
		armPose, err := arm.EndPosition(ctx, nil)
		if err != nil {
			return result, fmt.Errorf("failed to get arm position: %w", err)
		}

		// Get surface point
		reading, err := GetSurfacePoint(ctx, logger, fs, sensor, config.Hardware.WorldFrame)
		if err != nil {
			return result, fmt.Errorf("failed to get sensor reading: %w", err)
		}

		// Check if point is on the plane
		distanceFromPlane := PointDistanceFromPlane(reading.SurfacePoint, plane)
		logger.Debugf("%s search - Arm Z=%.1f, surface=(%.1f,%.1f,%.1f), dist from plane=%.1f",
			edgeName, armPose.Point().Z, reading.SurfacePoint.X, reading.SurfacePoint.Y, reading.SurfacePoint.Z, distanceFromPlane)

		// If we've gone past the edge (point no longer on plane)
		if distanceFromPlane > config.Detection.PlaneThreshold {
			result.Found = true
			logger.Infof("✓ Found %s edge at arm Z=%.1f (point distance from plane: %.1f mm)", edgeName, result.SurfacePoint.Z, distanceFromPlane)
			break
		}

		// Update last valid surface point
		result.SurfacePoint = reading.SurfacePoint

		// Move in Z direction
		poseX := armPose.Point().X
		worldArmPose, err := fs.GetPose(ctx, arm.Name().Name, config.Hardware.WorldFrame, nil, nil)
		if err != nil {
			return result, fmt.Errorf("failed to get arm world pose: %w", err)
		}

		nextPose := spatialmath.NewPose(
			r3.Vector{
				X: poseX,
				Y: armPose.Point().Y,
				Z: armPose.Point().Z + float64(zDirection)*config.Detection.EdgeStepSize,
			},
			armPose.Orientation(),
		)

		err = arm.MoveToPosition(ctx, nextPose, nil)

		// Handle collision - try moving away from itself in +X direction
		for err != nil && poseX < -worldArmPose.Pose().Point().Y+config.Hardware.GripperWidth {
			logger.Debugf("%s search - hit joint limit at X=%.1f, moving in +x dir", edgeName, poseX)
			poseX += config.Detection.EdgeStepSize
			nextPose := spatialmath.NewPose(
				r3.Vector{
					X: poseX,
					Y: armPose.Point().Y,
					Z: armPose.Point().Z + float64(zDirection)*config.Detection.EdgeStepSize,
				},
				armPose.Orientation(),
			)
			err = arm.MoveToPosition(ctx, nextPose, nil)
		}

		if err != nil {
			// Now too close to monitor, give up
			logger.Infof("✓ Found %s edge at arm Z=%.1f (joint limit)", edgeName, result.SurfacePoint.Z)
			break
		}
	}

	return result, nil
}

// FindHorizontalEdge searches for an edge by scanning the gantry
// xDirection: +1 for left edge (scan outward), -1 for right edge (scan inward)
func FindHorizontalEdge(ctx context.Context, logger logging.Logger, fs framesystem.RobotFrameSystem,
	sensor sensor.Sensor, gantry gantry.Gantry, plane Plane,
	gantryLengths []float64, xDirection int, config CalibrationConfig) (EdgeSearchResult, error) {
	var edgeName string
	if xDirection == 1 {
		edgeName = "left"
	} else {
		edgeName = "right"
	}
	var result EdgeSearchResult
	centerPos := gantryLengths[0] / 2

	// Determine start, end, and step based on direction
	var currentPos float64
	var endPos float64
	step := config.Detection.EdgeStepSize * float64(xDirection)

	if xDirection > 0 {
		// Left edge: scan from center to max
		currentPos = centerPos
		endPos = gantryLengths[0]
	} else {
		// Right edge: scan from center to 0
		currentPos = centerPos
		endPos = 0
	}

	for {
		// Check if we've reached the end
		if (xDirection > 0 && currentPos > endPos) || (xDirection < 0 && currentPos < endPos) {
			break
		}

		err := gantry.MoveToPosition(ctx, []float64{currentPos}, []float64{config.Scanning.GantrySpeed}, nil)
		if err != nil {
			return result, fmt.Errorf("failed to move gantry: %w", err)
		}

		// Get surface point
		reading, err := GetSurfacePoint(ctx, logger, fs, sensor, config.Hardware.WorldFrame)
		if err != nil {
			currentPos += step
			continue
		}

		distanceFromPlane := PointDistanceFromPlane(reading.SurfacePoint, plane)
		logger.Debugf("%s search - Gantry X=%.1f, dist from plane=%.1f", edgeName, currentPos, distanceFromPlane)

		// If we've gone past the edge (point no longer on plane)
		if distanceFromPlane > config.Detection.PlaneThreshold {
			result.Found = true
			logger.Infof("✓ Found %s edge at gantry position X=%.1f (dist from plane=%.1f)", edgeName, result.SurfacePoint.X, distanceFromPlane)
			break
		}

		result.SurfacePoint = reading.SurfacePoint
		currentPos += step
	}

	if !result.Found {
		logger.Infof("Could not find %s edge within gantry range, using point at end position", edgeName)
		// Get surface point at the final position
		err := gantry.MoveToPosition(ctx, []float64{endPos}, []float64{config.Scanning.GantrySpeed}, nil)
		if err != nil {
			return result, fmt.Errorf("failed to move gantry to end position: %w", err)
		}
		reading, err := GetSurfacePoint(ctx, logger, fs, sensor, config.Hardware.WorldFrame)
		if err != nil {
			return result, fmt.Errorf("failed to get final surface point: %w", err)
		}
		logger.Infof("Using reading at gantry X=%.1f for %s edge: %+v", endPos, edgeName, reading.SurfacePoint)
		result.SurfacePoint = reading.SurfacePoint
	}

	return result, nil
}

// PointDistanceFromPlane calculates the distance of a point from a plane
func PointDistanceFromPlane(point Point3D, plane Plane) float64 {
	// Distance = |Ax + By + Cz - D| / sqrt(A² + B² + C²)
	numerator := math.Abs(plane.A*point.X + plane.B*point.Y + plane.C*point.Z - plane.D)
	denominator := math.Sqrt(plane.A*plane.A + plane.B*plane.B + plane.C*plane.C)
	return numerator / denominator
}

// CalculatePlaneFrom3Points calculates a plane from 3 non-collinear points
func CalculatePlaneFrom3Points(p1, p2, p3 Point3D) (Plane, error) {
	// Create two vectors in the plane
	v1 := Point3D{X: p2.X - p1.X, Y: p2.Y - p1.Y, Z: p2.Z - p1.Z}
	v2 := Point3D{X: p3.X - p1.X, Y: p3.Y - p1.Y, Z: p3.Z - p1.Z}

	// Normal vector is the cross product
	normal := Point3D{
		X: v1.Y*v2.Z - v1.Z*v2.Y,
		Y: v1.Z*v2.X - v1.X*v2.Z,
		Z: v1.X*v2.Y - v1.Y*v2.X,
	}

	// Check for collinear points
	length := math.Sqrt(normal.X*normal.X + normal.Y*normal.Y + normal.Z*normal.Z)
	if length < 0.001 {
		return Plane{}, fmt.Errorf("points are collinear, cannot define a plane")
	}

	// Ensure normal points away from monitor (positive Y direction preferred)
	// This makes the orientation calculation consistent
	if normal.Y < 0 {
		normal.X = -normal.X
		normal.Y = -normal.Y
		normal.Z = -normal.Z
	}

	// Plane equation: A*x + B*y + C*z = D
	plane := Plane{
		A: normal.X,
		B: normal.Y,
		C: normal.Z,
		D: normal.X*p1.X + normal.Y*p1.Y + normal.Z*p1.Z,
	}

	return plane, nil
}
