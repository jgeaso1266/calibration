package calibrationhelpers

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
)

// CalibrationResult holds the final calibration data
type CalibrationResult struct {
	Plane         Plane
	BottomZ       float64
	TopZ          float64
	LeftX         float64
	RightX        float64
	MonitorWidth  float64
	MonitorHeight float64

	// 3 Points for orientation calculation
	XPoint1 Point3D
	XPoint2 Point3D
	ZPoint1 Point3D
}

// GenerateVisualizationConfig creates a Viam robot config snippet for visualizing the monitor
// NOTE: does not work with rotations about the Y axis
func GenerateVisualizationConfig(logger logging.Logger, result CalibrationResult, worldFrame string) map[string]interface{} {
	// Calculate center of monitor
	centerX := (result.LeftX + result.RightX) / 2
	centerZ := (result.BottomZ + result.TopZ) / 2
	width := result.LeftX - result.RightX
	height := result.TopZ - result.BottomZ

	// Calculate Y position on the plane at the center
	// From plane equation: A*x + B*y + C*z = D
	// Solving for y: y = (D - A*centerX - C*centerZ) / B
	centerY := (result.Plane.D - result.Plane.A*centerX - result.Plane.C*centerZ) / result.Plane.B

	// Build orientation using the plane normal as Y-axis (perpendicular to monitor surface)

	// Step 1: Calculate normalized plane normal (this becomes localY)
	normalLength := math.Sqrt(result.Plane.A*result.Plane.A + result.Plane.B*result.Plane.B + result.Plane.C*result.Plane.C)
	localY := r3.Vector{
		X: result.Plane.A / normalLength,
		Y: result.Plane.B / normalLength,
		Z: result.Plane.C / normalLength,
	}

	// Convert calibration points to r3.Vector
	xPt1 := r3.Vector{X: result.XPoint1.X, Y: result.XPoint1.Y, Z: result.XPoint1.Z}
	xPt2 := r3.Vector{X: result.XPoint2.X, Y: result.XPoint2.Y, Z: result.XPoint2.Z}

	// Step 2: Get the direction from XPoint1 to XPoint2 (width direction on monitor)
	xDir := xPt2.Sub(xPt1).Normalize()

	// Step 3: Local Z axis perpendicular to both Y and X direction
	// Z = xDir × Y (this will be roughly "up" on the monitor)
	localZ := xDir.Cross(localY).Normalize()

	// Step 4: Local X axis perpendicular to Y and Z (ensures perfect orthogonality)
	// X = Y × Z (to stay same direction as xDir)
	localX := localY.Cross(localZ).Normalize()

	// Convert rotation matrix to quaternion
	rotMatrix, err := spatialmath.NewRotationMatrix([]float64{
		localX.X, localX.Y, localX.Z,
		localY.X, localY.Y, localY.Z,
		localZ.X, localZ.Y, localZ.Z,
	})
	if err != nil {
		logger.Errorf("Error creating rotation matrix: %v", err)
		return nil
	}
	quaternion := rotMatrix.Quaternion()

	config := map[string]any{
		"name":  "calibrated-monitor",
		"type":  "generic",
		"model": "fake",
		"frame": map[string]any{
			"parent": worldFrame,
			"translation": map[string]any{
				"x": centerX,
				"y": centerY,
				"z": centerZ,
			},
			"orientation": map[string]any{
				"type": "quaternion",
				"value": map[string]any{
					"x": quaternion.Imag,
					"y": quaternion.Jmag,
					"z": quaternion.Kmag,
					"w": quaternion.Real,
				},
			},
			"geometry": map[string]any{
				"type": "box",
				"x":    width,
				"y":    1.0,
				"z":    height,
			},
		},
	}
	return config
}
