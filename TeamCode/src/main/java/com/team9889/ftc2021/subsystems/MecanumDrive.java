package com.team9889.ftc2021.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.lib.RevIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public RevIMU imu;
    private List<LynxModule> allHubs;
    private double frontLeftTargetVelocity, frontRightTargetVelocity, backLeftTargetVelocity, backRightTargetVelocity;

    public MecanumDrive () {}

    public void init(HardwareMap hardwareMap) {
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        this.frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        this.backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        this.backRight = hardwareMap.get(DcMotorEx.class, "rb");

        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = new RevIMU("imu", hardwareMap);

        this.allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : this.allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.update();
    }

    public void update() {
        for (LynxModule module : this.allHubs) {
            module.clearBulkCache();
        }

        imu.update();

        this.frontLeft.setPower(this.frontLeftTargetVelocity);
        this.frontRight.setPower(this.frontRightTargetVelocity);
        this.backLeft.setPower(this.backLeftTargetVelocity);
        this.backRight.setPower(this.backRightTargetVelocity);
    }

    private double round(double value, int significantFigures) {
        double k = Math.pow(10., significantFigures);
        return Math.round(value * k) / k;
    }

    private double round(double value) {
        return round(value, 0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("frontLeft Position", this.frontLeft.getCurrentPosition());
        telemetry.addData("frontRight Position", this.frontRight.getCurrentPosition());
        telemetry.addData("backLeft Position", this.backLeft.getCurrentPosition());
        telemetry.addData("backRight Position", this.backRight.getCurrentPosition());
        telemetry.addData("IMU Angle", round(this.getAngle(AngleUnit.DEGREES), 3));
        telemetry.addData("IMU X Raw Acceleration", this.imu.getRawAcceleration().xAccel);
        telemetry.addData("IMU Y Raw Acceleration", this.imu.getRawAcceleration().yAccel);
        telemetry.addData("IMU Z Raw Acceleration", this.imu.getRawAcceleration().zAccel);
        telemetry.addData("IMU X Filtered Acceleration", this.imu.getFilteredAcceleration().xAccel);
        telemetry.addData("IMU Y Filtered Acceleration", this.imu.getFilteredAcceleration().yAccel);
        telemetry.addData("IMU Z Filtered Acceleration", this.imu.getFilteredAcceleration().zAccel);
        telemetry.addData("IMU X Velocity", round(this.imu.getVelocity().xVeloc, 5));
        telemetry.addData("IMU Y Velocity", round(this.imu.getVelocity().yVeloc, 5));
        telemetry.addData("IMU Z Velocity", round(this.imu.getVelocity().zVeloc, 5));
        telemetry.addData("IMU X", round(this.imu.getPosition().x, 5));
        telemetry.addData("IMU Y", round(this.imu.getPosition().y, 5));
        telemetry.addData("IMU Z", round(this.imu.getPosition().z, 5));
    }

    public void setPower(double frontLeftTargetVelocity, double frontRightTargetVelocity, double backLeftTargetVelocity, double backRightTargetVelocity) {
        this.frontLeftTargetVelocity = frontLeftTargetVelocity;
        this.frontRightTargetVelocity = frontRightTargetVelocity;
        this.backLeftTargetVelocity = backLeftTargetVelocity;
        this.backRightTargetVelocity = backRightTargetVelocity;
    }

    public void setPower(double leftStickX, double leftStickY, double rightStickX){
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - 3 * Math.PI / 4;
        double rightX = rightStickX;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        this.setPower(v1, v2,v3, v4);
    }

    public double getAngle(AngleUnit angleUnit) {
        return this.imu.getNormalHeading(angleUnit);
    }

    public double[] getPositions() {
        double[] velocities = new double[4];
        velocities[0] = this.frontLeft.getCurrentPosition();
        velocities[1] = this.frontRight.getCurrentPosition();
        velocities[2] = this.backLeft.getCurrentPosition();
        velocities[3] = this.backRight.getCurrentPosition();
        return velocities;
    }

    public double[] getSpeeds() {
        double[] velocities = new double[4];
        velocities[0] = this.frontLeft.getVelocity();
        velocities[1] = this.frontRight.getVelocity();
        velocities[2] = this.backLeft.getVelocity();
        velocities[3] = this.backRight.getVelocity();
        return velocities;
    }
}
