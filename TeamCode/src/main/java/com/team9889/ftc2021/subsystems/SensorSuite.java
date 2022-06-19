package com.team9889.ftc2021.subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SensorSuite {

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private double frontDistanceM;

    public SensorSuite () {}

    public void init(HardwareMap hardwareMap) {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mrf");

        this.update();
    }

    public void update() {
        frontDistanceM = rangeSensor.rawUltrasonic() / 100.f;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Distance M", this.getDistance());
    }

    public double getDistance() {
        return frontDistanceM;
    }
}
