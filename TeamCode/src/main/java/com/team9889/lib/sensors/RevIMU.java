package com.team9889.lib.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by joshua9889 on 10/6/2017.
 * Class to make using the rev imu easy
 */

public class RevIMU {

    private BNO055IMU imu;
    private AxesOrder axesOrder;

    public RevIMU(String id, HardwareMap hardwareMap) {
        this(id, hardwareMap, AxesOrder.ZXY);
    }

    public RevIMU(String id, HardwareMap hardwareMap, AxesOrder imuOrientation){
        axesOrder = imuOrientation;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        this.imu = hardwareMap.get(BNO055IMU.class, id);
        this.imu.initialize(parameters);
    }

    public double getCurrentRotation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS).firstAngle;
    }

}
