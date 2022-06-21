package com.team9889.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2021.Constants;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    double initialAcceleration = 0.1;
    double measurementNoise = 0.2;
    double processNoise = 0.001;
    double dt = 0.1d;

//    { 1d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d},
//    { 0d, 1d, 0d, 0d, 0d, 0d, 0d, 0d, 0d},
//    { 0d, 0d, 1d, 0d, 0d, 0d, 0d, 0d, 0d},
//    { 0d, 0d, 0d, 1d, 0d, 0d, 0d, 0d, 0d},
//    { 0d, 0d, 0d, 0d, 1d, 0d, 0d, 0d, 0d},
//    { 0d, 0d, 0d, 0d, 0d, 1d, 0d, 0d, 0d},
//    { 0d, 0d, 0d, 0d, 0d, 0d, 1d, 0d, 0d},
//    { 0d, 0d, 0d, 0d, 0d, 0d, 0d, 1d, 0d},
//    { 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 1d}

    RealMatrix A = new Array2DRowRealMatrix(new double[][] {
            { 1d, 0d, 0d},
            { 0d, 1d, 0d},
            { 0d, 0d, 1d}
    });

    RealMatrix B = null;

    RealMatrix H = new Array2DRowRealMatrix(new double[][] {
            { 1d, 0d, 0d},
            { 0d, 1d, 0d},
            { 0d, 0d, 1d}
    });

    RealVector x = new ArrayRealVector(new double[] { initialAcceleration, initialAcceleration, initialAcceleration });

    RealMatrix Q = new Array2DRowRealMatrix(new double[][] {
            { processNoise, 0d,             0d},
            { 0d,           processNoise,   0d},
            { 0d,           0d,             processNoise}
    });

    RealMatrix P0 = new Array2DRowRealMatrix(new double[][] {
            { 1d, 0d, 0d},
            { 0d, 1d, 0d},
            { 0d, 0d, 1d}
    });

    RealMatrix R = new Array2DRowRealMatrix(new double[][] {
            { measurementNoise, 0d, 0d},
            { 0d, measurementNoise, 0d},
            { 0d, 0d, measurementNoise}
    });

    ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
    MeasurementModel mm = new DefaultMeasurementModel(H, R);
    KalmanFilter accelKalmanFilter = new KalmanFilter(pm, mm);

    public void update() {
        if (imu != null) {
            currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            acceleration = imu.getLinearAcceleration();

            Acceleration minimizedAcceleration = new Acceleration();
            minimizedAcceleration.xAccel = Math.abs(acceleration.xAccel) > 0.2 ? acceleration.xAccel : 0;
            minimizedAcceleration.yAccel = Math.abs(acceleration.yAccel) > 0.2 ? acceleration.yAccel : 0;
            minimizedAcceleration.zAccel = Math.abs(acceleration.zAccel) > 0.2 ? acceleration.zAccel : 0;

            accelKalmanFilter.predict();

            RealVector z = new ArrayRealVector(new double[] { minimizedAcceleration.xAccel, minimizedAcceleration.yAccel, minimizedAcceleration.zAccel });

            accelKalmanFilter.correct(z);

            accelerationFiltered.xAccel = accelKalmanFilter.getStateEstimation()[0];
            accelerationFiltered.yAccel = accelKalmanFilter.getStateEstimation()[1];
            accelerationFiltered.zAccel = accelKalmanFilter.getStateEstimation()[2];

//            testFilter1();
            testIntegration();
        } else {
            RobotLog.d("IMU not initialized");
        }
    }

    // Need to adjust these values
    private Filter xAccelerationFilter = new Filter(Constants.frequency, Constants.sampleRate, Filter.PassType.Highpass,Constants.resonance);
    private Filter yAccelerationFilter = new Filter(Constants.frequency, Constants.sampleRate, Filter.PassType.Highpass,Constants.resonance);
    private Filter zAccelerationFilter = new Filter(Constants.frequency, Constants.sampleRate, Filter.PassType.Highpass,Constants.resonance);

    private CircularBuffer xBuffer = new CircularBuffer(Constants.windowSize);
    private CircularBuffer yBuffer = new CircularBuffer(Constants.windowSize);
    private CircularBuffer zBuffer = new CircularBuffer(Constants.windowSize);

    private void testFilter1() {
        // xAccelerationFilter.Update((float) getRawAcceleration().xAccel);
        // yAccelerationFilter.Update((float) getRawAcceleration().yAccel);
        // zAccelerationFilter.Update((float) getRawAcceleration().zAccel);

        xBuffer.addValue(getRawAcceleration().xAccel);
        yBuffer.addValue(getRawAcceleration().yAccel);
        zBuffer.addValue(getRawAcceleration().zAccel);

        //  accelerationFiltered.xAccel = xBuffer.getAverage();
        //  accelerationFiltered.yAccel = yBuffer.getAverage();
        //  accelerationFiltered.zAccel = zBuffer.getAverage();

        accelerationFiltered.xAccel = Math.abs(xBuffer.getAverage()) > 0.2 ? xBuffer.getAverage() : 0;
        accelerationFiltered.yAccel = Math.abs(yBuffer.getAverage()) > 0.2 ? yBuffer.getAverage() : 0;
        accelerationFiltered.zAccel = Math.abs(zBuffer.getAverage()) > 0.2 ? zBuffer.getAverage() : 0;
    }

    boolean startIntegration = true;
    ElapsedTime settleTimer = new ElapsedTime();
    // Need to implement a high pass filter so it doesn't drift as much
    private void testIntegration() {
        if (!startIntegration && settleTimer.milliseconds() > 10000) {
            Orientation deltaOrientation = currentOrientation;
            deltaOrientation.firstAngle -= lastOrientation.firstAngle;
            deltaOrientation.secondAngle -= lastOrientation.secondAngle;
            deltaOrientation.thirdAngle -= lastOrientation.thirdAngle;

            double dt = timer.seconds();

            
            integratedVelocity.xVeloc += getFilteredAcceleration().xAccel * dt;
            integratedPosition.x += integratedVelocity.xVeloc * dt;

            integratedVelocity.yVeloc += getFilteredAcceleration().yAccel * dt;
            integratedPosition.y += integratedVelocity.yVeloc * dt;

            integratedVelocity.zVeloc += getFilteredAcceleration().zAccel * dt;
            integratedPosition.z += integratedVelocity.zVeloc * dt;

            lastOrientation = currentOrientation;
            timer.reset();
        } else if (startIntegration) {
            settleTimer.reset();
            startIntegration = false;
        }
    }

    public Orientation getNormalHeading() {
        return currentOrientation;
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

    public void toTelemetry(Telemetry telemetry) {
        telemetry.addData("IMU Angle (deg)", this.getNormalHeading().toAngleUnit(AngleUnit.DEGREES).firstAngle);
        telemetry.addData("IMU X Raw Acceleration (m/s^2)", this.getRawAcceleration().xAccel);
        telemetry.addData("IMU Y Raw Acceleration (m/s^2)", this.getRawAcceleration().yAccel);
        telemetry.addData("IMU Z Raw Acceleration (m/s^2)", this.getRawAcceleration().zAccel);
        telemetry.addData("IMU X Filtered Acceleration (m/s^2)", this.getFilteredAcceleration().xAccel);
        telemetry.addData("IMU Y Filtered Acceleration (m/s^2)", this.getFilteredAcceleration().yAccel);
        telemetry.addData("IMU Z Filtered Acceleration (m/s^2)", this.getFilteredAcceleration().zAccel);
        telemetry.addData("IMU X Velocity (m/s)", this.getVelocity().xVeloc);
        telemetry.addData("IMU Y Velocity (m/s)", this.getVelocity().yVeloc);
        telemetry.addData("IMU Z Velocity (m/s)", this.getVelocity().zVeloc);
        telemetry.addData("IMU X (m)", this.getPosition().x);
        telemetry.addData("IMU Y (m)", this.getPosition().y);
        telemetry.addData("IMU Z (m)", this.getPosition().z);
    }
}
