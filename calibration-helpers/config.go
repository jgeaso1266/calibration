package calibrationhelpers

import (
	"errors"
	"math"
	"os"
)

// CalibrationConfig holds all configuration for the calibration workflow
type CalibrationConfig struct {
	Hardware     HardwareConfig
	Scanning     ScanningConfig
	Detection    DetectionConfig
	Robot        RobotConfig
	ArmPositions ArmPositions
}

// HardwareConfig contains hardware-specific parameters
type HardwareConfig struct {
	GripperWidth float64 // mm - gripper width for collision avoidance
	WorldFrame   string  // reference frame name for coordinate transforms
}

// ScanningConfig contains parameters for the scanning phase
type ScanningConfig struct {
	ZStepSize   float64 // mm - vertical step size for Z-axis scan
	ZNumSteps   int     // number of Z-axis scan points
	XNumSteps   int     // number of X-axis (gantry) scan points
	GantrySpeed float64 // mm/sec - gantry movement speed
}

// DetectionConfig contains parameters for edge detection
type DetectionConfig struct {
	PlaneThreshold float64 // mm - distance threshold for edge detection
	EdgeStepSize   float64 // mm - step size when searching for edges
}

// RobotConfig contains robot connection and component information
type RobotConfig struct {
	Address    string // Robot address (e.g., "*.viam.cloud")
	MachineID  string // Viam machine ID
	APIKey     string // Viam API key (should be loaded from env var)
	SensorName string // Depth sensor component name
	ArmName    string // Arm component name
	GantryName string // Gantry component name
}

// ArmPositions contains named arm joint positions for different phases
type ArmPositions struct {
	Home       []float64 // Starting position for calibration
	BottomScan []float64 // Position for bottom edge scanning
	TopScan    []float64 // Position for top edge scanning
}

// DefaultArmPositions provides standard arm positions for a 5-joint arm
var DefaultArmPositions = ArmPositions{
	Home:       []float64{0, -math.Pi / 2, math.Pi / 2, 0, -math.Pi / 2},
	BottomScan: []float64{0, 0, math.Pi / 2, -math.Pi / 2, math.Pi / 2},
	TopScan:    []float64{0, 0, math.Pi / 2, -math.Pi / 2, -math.Pi / 2},
}

// NewDefaultConfig creates a configuration with default values and env var overrides
func NewDefaultConfig() CalibrationConfig {
	return CalibrationConfig{
		Hardware: HardwareConfig{
			GripperWidth: 106.4, // mm - default gripper width
			WorldFrame:   "world",
		},
		Scanning: ScanningConfig{
			ZStepSize:   10.0, // mm
			ZNumSteps:   10,
			XNumSteps:   10,
			GantrySpeed: 50.0, // mm/sec
		},
		Detection: DetectionConfig{
			PlaneThreshold: 20.0, // mm
			EdgeStepSize:   10.0, // mm
		},
		Robot: RobotConfig{
			Address:    getEnvOrDefault("VIAM_ROBOT_ADDRESS", ""),
			MachineID:  getEnvOrDefault("VIAM_MACHINE_ID", ""),
			APIKey:     getEnvOrDefault("VIAM_API_KEY", ""),
			SensorName: getEnvOrDefault("VIAM_SENSOR_NAME", "test-sensor-for-monitor"),
			ArmName:    getEnvOrDefault("VIAM_ARM_NAME", "arm-1"),
			GantryName: getEnvOrDefault("VIAM_GANTRY_NAME", "gantry-1"),
		},
		ArmPositions: DefaultArmPositions,
	}
}

// Validate checks that all configuration values are valid
func (c *CalibrationConfig) Validate() error {
	if c.Robot.SensorName == "" || c.Robot.ArmName == "" || c.Robot.GantryName == "" {
		return errors.New("component names cannot be empty")
	}
	if c.Scanning.ZNumSteps < 2 || c.Scanning.XNumSteps < 2 {
		return errors.New("scan steps must be >= 2")
	}
	if c.Hardware.GripperWidth <= 0 {
		return errors.New("gripper width must be positive")
	}
	if c.Detection.PlaneThreshold <= 0 {
		return errors.New("plane threshold must be positive")
	}
	if c.Scanning.ZStepSize <= 0 || c.Detection.EdgeStepSize <= 0 {
		return errors.New("step sizes must be positive")
	}
	if c.Scanning.GantrySpeed <= 0 {
		return errors.New("gantry speed must be positive")
	}
	if len(c.ArmPositions.Home) == 0 || len(c.ArmPositions.BottomScan) == 0 || len(c.ArmPositions.TopScan) == 0 {
		return errors.New("arm positions must be defined")
	}
	return nil
}

// getEnvOrDefault returns the environment variable value or a default if not set
func getEnvOrDefault(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}
