package calibrationhelpers

import (
	"fmt"
	"math"

	"go.viam.com/rdk/logging"
	"gonum.org/v1/gonum/mat"

)

// fitLineToPoints performs simple least-squares line fitting to a set of 3D points
// Returns two points on the line (start and end of the fitted line through the data)
// This is kept here because it's simple and used locally
func FitLineToPoints(logger logging.Logger, points []Point3D) (Point3D, Point3D, error) {
	if len(points) < 2 {
		return Point3D{}, Point3D{}, fmt.Errorf("need at least 2 points to fit a line")
	}

	// Calculate centroid (mean point)
	var centroid Point3D
	for _, p := range points {
		centroid.X += p.X
		centroid.Y += p.Y
		centroid.Z += p.Z
	}
	centroid.X /= float64(len(points))
	centroid.Y /= float64(len(points))
	centroid.Z /= float64(len(points))

	// Calculate direction vector using PCA (simplified - use first principal component)
	// For simplicity, just use the vector from first to last point as the direction
	// A proper implementation would use SVD (see helper.go)
	direction := Point3D{
		X: points[len(points)-1].X - points[0].X,
		Y: points[len(points)-1].Y - points[0].Y,
		Z: points[len(points)-1].Z - points[0].Z,
	}

	// Normalize direction
	length := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)
	direction.X /= length
	direction.Y /= length
	direction.Z /= length

	// Return two points on the line: centroid ± some distance along the direction
	halfSpan := length / 2
	p1 := Point3D{
		X: centroid.X - halfSpan*direction.X,
		Y: centroid.Y - halfSpan*direction.Y,
		Z: centroid.Z - halfSpan*direction.Z,
	}
	p2 := Point3D{
		X: centroid.X + halfSpan*direction.X,
		Y: centroid.Y + halfSpan*direction.Y,
		Z: centroid.Z + halfSpan*direction.Z,
	}

	logger.Infof("Fitted line through %d points: (%f,%f,%f) to (%f,%f,%f)",
		len(points), p1.X, p1.Y, p1.Z, p2.X, p2.Y, p2.Z)

	return p1, p2, nil
}

// fitLineToPointsProper performs proper PCA-based line fitting using SVD
// Returns the centroid and a unit direction vector
func fitLineToPointsProper(logger logging.Logger, points []Point3D) (centroid Point3D, direction Point3D, err error) {
	if len(points) < 2 {
		return Point3D{}, Point3D{}, fmt.Errorf("need at least 2 points to fit a line")
	}

	n := len(points)

	// Step 1: Calculate centroid (mean point)
	centroid = Point3D{}
	for _, p := range points {
		centroid.X += p.X
		centroid.Y += p.Y
		centroid.Z += p.Z
	}
	centroid.X /= float64(n)
	centroid.Y /= float64(n)
	centroid.Z /= float64(n)

	// Step 2: Create centered data matrix (subtract centroid from each point)
	// Matrix is n×3 where each row is (x-cx, y-cy, z-cz)
	data := mat.NewDense(n, 3, nil)
	for i, p := range points {
		data.Set(i, 0, p.X-centroid.X)
		data.Set(i, 1, p.Y-centroid.Y)
		data.Set(i, 2, p.Z-centroid.Z)
	}

	// Step 3: Perform SVD on the centered data
	// SVD decomposes data = U * Σ * V^T
	// The columns of V are the principal components (directions of variance)
	// The first column of V (largest singular value) is our line direction
	var svd mat.SVD
	ok := svd.Factorize(data, mat.SVDThin)
	if !ok {
		return Point3D{}, Point3D{}, fmt.Errorf("SVD factorization failed")
	}

	// Get the V matrix (right singular vectors)
	var v mat.Dense
	svd.VTo(&v)

	// The first column of V is the principal direction (direction of maximum variance)
	// This is our line direction
	direction = Point3D{
		X: v.At(0, 0), // First column, first row
		Y: v.At(1, 0), // First column, second row
		Z: v.At(2, 0), // First column, third row
	}

	// Normalize the direction (should already be normalized from SVD, but make sure)
	length := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)
	direction.X /= length
	direction.Y /= length
	direction.Z /= length

	// Optional: Calculate how well the points fit the line (for debugging)
	singularValues := svd.Values(nil)
	logger.Debugf("Singular values: %v", singularValues)
	logger.Debugf("Line fitting quality: largest/second = %.2f (higher is better)",
		singularValues[0]/singularValues[1])

	logger.Infof("Fitted line (PCA): centroid=(%f,%f,%f), direction=(%f,%f,%f)",
		centroid.X, centroid.Y, centroid.Z, direction.X, direction.Y, direction.Z)

	return centroid, direction, nil
}

// fitLineToPointsProperReturnEndpoints is a wrapper that returns two points on the line
// (compatible with the original function signature)
func fitLineToPointsProperReturnEndpoints(logger logging.Logger, points []Point3D) (Point3D, Point3D, error) {
	centroid, direction, err := fitLineToPointsProper(logger, points)
	if err != nil {
		return Point3D{}, Point3D{}, err
	}

	// Calculate span of data along the line direction to determine endpoint distance
	// Project each point onto the line and find min/max projection distance
	var minProj, maxProj float64
	for i, p := range points {
		// Vector from centroid to point
		v := Point3D{
			X: p.X - centroid.X,
			Y: p.Y - centroid.Y,
			Z: p.Z - centroid.Z,
		}
		// Project onto direction vector (dot product)
		projection := v.X*direction.X + v.Y*direction.Y + v.Z*direction.Z

		if i == 0 {
			minProj = projection
			maxProj = projection
		} else {
			if projection < minProj {
				minProj = projection
			}
			if projection > maxProj {
				maxProj = projection
			}
		}
	}

	// Return two points: centroid ± projection distance along direction
	p1 := Point3D{
		X: centroid.X + minProj*direction.X,
		Y: centroid.Y + minProj*direction.Y,
		Z: centroid.Z + minProj*direction.Z,
	}
	p2 := Point3D{
		X: centroid.X + maxProj*direction.X,
		Y: centroid.Y + maxProj*direction.Y,
		Z: centroid.Z + maxProj*direction.Z,
	}

	logger.Infof("Line endpoints: (%f,%f,%f) to (%f,%f,%f)",
		p1.X, p1.Y, p1.Z, p2.X, p2.Y, p2.Z)

	return p1, p2, nil
}

// calculateLineFitError calculates the RMS error of points from the fitted line (for validation)
func calculateLineFitError(points []Point3D, centroid, direction Point3D) float64 {
	sumSquaredError := 0.0
	for _, p := range points {
		// Vector from centroid to point
		v := Point3D{
			X: p.X - centroid.X,
			Y: p.Y - centroid.Y,
			Z: p.Z - centroid.Z,
		}
		// Project onto direction
		projection := v.X*direction.X + v.Y*direction.Y + v.Z*direction.Z
		
		// Point on line closest to p
		closestPoint := Point3D{
			X: centroid.X + projection*direction.X,
			Y: centroid.Y + projection*direction.Y,
			Z: centroid.Z + projection*direction.Z,
		}
		
		// Distance from p to closest point on line
		dx := p.X - closestPoint.X
		dy := p.Y - closestPoint.Y
		dz := p.Z - closestPoint.Z
		distanceSquared := dx*dx + dy*dy + dz*dz
		
		sumSquaredError += distanceSquared
	}
	
	return math.Sqrt(sumSquaredError / float64(len(points)))
}
