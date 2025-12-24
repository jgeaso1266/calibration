package main

import (
	"calibration"

	"go.viam.com/rdk/components/generic"
	sensor "go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{ sensor.API, calibration.FakeSensor},
		resource.APIModel{ generic.API, calibration.MonitorCalibration},
	)
}
