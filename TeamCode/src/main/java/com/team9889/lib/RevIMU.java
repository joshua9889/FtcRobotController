package com.team9889.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.lib.Filter;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by joshua9889 on 10/6/2017.
 * Class to make using the rev imu easy
 */

public class RevIMU {
    private BNO055IMU imu;
    private ElapsedTime timer = new ElapsedTime();

    public RevIMU(String id, HardwareMap hardwareMap, Velocity initialVelocity, Position initialPosition, Acceleration accelerationMinimum){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        this.imu = hardwareMap.get(BNO055IMU.class, id);
        this.imu.initialize(parameters);

        this.integratedVelocity = initialVelocity;
        this.integratedPosition = initialPosition;
        if(accelerationMinimum.xAccel == 0.0 && accelerationMinimum.yAccel == 0.0 && accelerationMinimum.zAccel == 0.0) {
            this.accelerationMinimum = new Acceleration();
            this.accelerationMinimum.xAccel = 0.0;
            this.accelerationMinimum.yAccel = 0.0;
            this.accelerationMinimum.zAccel = 0.0;
        } else {
            this.accelerationMinimum = accelerationMinimum;
        }

        this.update();
    }

    public RevIMU(String id, HardwareMap hardwareMap) {
        this(id, hardwareMap, new Velocity(), new Position(), new Acceleration());
    }

    private Orientation currentOrientation, lastOrientation = new Orientation();
    private Acceleration acceleration, accelerationFiltered = new Acceleration(), accelerationMinimum;
    private Velocity integratedVelocity;
    private Position integratedPosition;

    // Need to adjust these values
    private Filter xAccelerationFilter = new Filter(15000,44100, Filter.PassType.Highpass,1);
    private Filter yAccelerationFilter = new Filter(15000,44100, Filter.PassType.Highpass,1);
    private Filter zAccelerationFilter = new Filter(15000,44100, Filter.PassType.Highpass,1);

    public void update() {
        if (imu != null) {
            currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            acceleration = imu.getLinearAcceleration();

            xAccelerationFilter.Update((float) getRawAcceleration().xAccel);
            yAccelerationFilter.Update((float) getRawAcceleration().yAccel);
            zAccelerationFilter.Update((float) getRawAcceleration().zAccel);

            accelerationFiltered.xAccel = xAccelerationFilter.getValue();
            accelerationFiltered.yAccel = yAccelerationFilter.getValue();
            accelerationFiltered.zAccel = zAccelerationFilter.getValue();

            testIntegration();
        } else {
            RobotLog.d("IMU not initialized");
        }
    }

    // Need to implement a high pass filter so it doesn't drift as much
    private void testIntegration() {
        Orientation deltaOrientation = currentOrientation;
        deltaOrientation.firstAngle -= lastOrientation.firstAngle;
        deltaOrientation.secondAngle -= lastOrientation.secondAngle;
        deltaOrientation.thirdAngle -= lastOrientation.thirdAngle;

        double dt = timer.seconds();

        if (Math.abs(getFilteredAcceleration().xAccel) > accelerationMinimum.xAccel) {
            integratedVelocity.xVeloc += getFilteredAcceleration().xAccel * dt;
            integratedPosition.x += integratedVelocity.xVeloc * dt;
        }

        if (Math.abs(getFilteredAcceleration().yAccel) > accelerationMinimum.yAccel) {
            integratedVelocity.yVeloc += getFilteredAcceleration().yAccel * dt;
            integratedPosition.y += integratedVelocity.yVeloc * dt;
        }

        if (Math.abs(getFilteredAcceleration().zAccel) > accelerationMinimum.zAccel) {
            integratedVelocity.zVeloc += getFilteredAcceleration().zAccel * dt;
            integratedPosition.z += integratedVelocity.zVeloc * dt;
        }

        lastOrientation = currentOrientation;
        timer.reset();
    }

    public double getNormalHeading(AngleUnit angleUnit) {
        return currentOrientation.toAngleUnit(angleUnit).firstAngle;
    }

    public Position getPosition() {
        return integratedPosition;
    }

    public Velocity getVelocity() {
        return integratedVelocity;
    }

    public Acceleration getFilteredAcceleration() {
        return accelerationFiltered;
    }

    public Acceleration getRawAcceleration() {
        return acceleration;
    }
}
