package com.team9889.ftc2021.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.team9889.lib.sensors.RevIMU;
import com.team9889.lib.Twist;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

@Config
public class MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public RevIMU imu;
    private List<LynxModule> allHubs;
    private HardwareMap voltageHardwareMap;
    private DriveControlState driveControlState = DriveControlState.OPEN_LOOP;

    public static PIDFCoefficients motorCoefficients = new PIDFCoefficients(12,  0,   0, 650), lastMotorCoefficients;


    private double frontLeftTargetVelocity, frontRightTargetVelocity, backLeftTargetVelocity, backRightTargetVelocity;

    public enum DriveControlState {
        VELOCITY_CONTROL, OPEN_LOOP
    }

    public MecanumDrive () {}

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, DriveControlState.VELOCITY_CONTROL);
    }

    public void init(HardwareMap hardwareMap, DriveControlState driveControlState) {
        this.voltageHardwareMap = hardwareMap;
        this.driveControlState = driveControlState;

        this.frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        this.frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        this.backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        this.backRight = hardwareMap.get(DcMotorEx.class, "rb");

        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            switch (this.driveControlState) {
                case VELOCITY_CONTROL:
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, motorCoefficients);
                    break;
                case OPEN_LOOP:
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    return;
            }
        }

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

        if (this.driveControlState == DriveControlState.VELOCITY_CONTROL) {
            setPIDFCoefficients(motorCoefficients);
            sendWheelVelocities();
        }
    }

    private void setPIDFCoefficients(PIDFCoefficients coefficients) {
        if (coefficients != lastMotorCoefficients) {
            this.frontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            this.frontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            this.backLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            this.backRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }

        lastMotorCoefficients = coefficients;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : voltageHardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        return result;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("% Voltage", getBatteryVoltage());

        telemetry.addData("! frontLeft Position (m)", getPositions()[0]);
        telemetry.addData("! frontRight Position (m)", getPositions()[1]);
        telemetry.addData("! backLeft Position (m)", getPositions()[2]);
        telemetry.addData("! backRight Position (m)", getPositions()[3]);

        telemetry.addData("# frontLeft Target Velocity (m/s)", frontLeftTargetVelocity);
        telemetry.addData("# frontRight Target Velocity (m/s)", frontRightTargetVelocity);
        telemetry.addData("# backLeft Target Velocity (m/s)", backLeftTargetVelocity);
        telemetry.addData("# backRight Target Velocity (m/s)", backRightTargetVelocity);

        telemetry.addData("@ frontLeft Velocity (m/s)", getVelocities()[0]);
        telemetry.addData("@ frontRight Velocity (m/s)", getVelocities()[1]);
        telemetry.addData("@ backLeft Velocity (m/s)", getVelocities()[2]);
        telemetry.addData("@ backRight Velocity (m/s)", getVelocities()[3]);

        getCurrentTwist().toTelemetry("Current Twist", telemetry);
        getTargetTwist().toTelemetry("Target Twist", telemetry);

        imu.toTelemetry(telemetry);
    }

    public void setPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        this.frontLeft.setPower(frontLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backLeft.setPower(backLeftPower);
        this.backRight.setPower(backRightPower);
    }

    public void setWheelVelocities(double frontLeftTargetVelocity, double frontRightTargetVelocity, double backLeftTargetVelocity, double backRightTargetVelocity) {
        this.frontLeftTargetVelocity = frontLeftTargetVelocity;
        this.frontRightTargetVelocity = frontRightTargetVelocity;
        this.backLeftTargetVelocity = backLeftTargetVelocity;
        this.backRightTargetVelocity = backRightTargetVelocity;
    }

    private void sendWheelVelocities() {
        this.frontLeft.setVelocity(metersToTicks(this.frontLeftTargetVelocity));
        this.frontRight.setVelocity(metersToTicks(this.frontRightTargetVelocity));
        this.backLeft.setVelocity(metersToTicks(this.backLeftTargetVelocity));
        this.backRight.setVelocity(metersToTicks(this.backRightTargetVelocity));
    }

    /**
     *  Taken from https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
     *
     *  v_fl , v_fr, v_rl and v_rr represent the linear velocities for the front left,
     *      front right, rear left and rear right wheel respectively.
     *
     *  vx and vy represent the robot's base linear velocity in the x and y direction respectively.
     *      The x direction is in front of the robot.
     *
     *  w_z is angular velocity of the robot's base around the z-axis.
     *
     *  lx and ly represent the distance from the robot's center to the wheels projected on the
     *      x and y axis respectively.
     **/
    private final double lx = 150.0 / 1000.;
    private final double ly = 192.4 / 1000.;

    private Twist currentTargetTwist = new Twist();

    public void setTargetTwist(Twist twist){
        this.currentTargetTwist = twist;

        Velocity velocityInMS = currentTargetTwist.translationalVelocity.toUnit(DistanceUnit.METER);
        AngularVelocity angularVelocityInRadS = currentTargetTwist.angularVelocity.toAngleUnit(AngleUnit.RADIANS);

        double vx = velocityInMS.xVeloc;
        double vy = velocityInMS.yVeloc;

        double wz = angularVelocityInRadS.zRotationRate;

        double v_fl = vx - vy - (lx + ly) * wz;
        double v_fr = vx + vy + (lx + ly) * wz;
        double v_rl = vx + vy - (lx + ly) * wz;
        double v_rr = vx - vy + (lx + ly) * wz;

        this.setWheelVelocities(v_fl, v_fr, v_rl, v_rr);
    }

    public Twist getTargetTwist() {
        return currentTargetTwist;
    }

    public Twist getCurrentTwist() {
        Twist measuredTwist = new Twist();

        measuredTwist.translationalVelocity.unit = DistanceUnit.METER;
        measuredTwist.angularVelocity.unit = AngleUnit.RADIANS;

        double v_fl = getVelocities()[0], v_fr = getVelocities()[1];
        double v_rl = getVelocities()[2], v_rr = getVelocities()[3];

        measuredTwist.translationalVelocity.xVeloc =  (v_fl + v_fr + v_rl + v_rr) / 4.;
        measuredTwist.translationalVelocity.yVeloc = (-v_fl + v_fr + v_rl - v_rr) / 4.;

        measuredTwist.angularVelocity.zRotationRate = (float) ((-v_fl+v_fr-v_rl+v_rr) / (4.0 * (lx+ly)));

        return measuredTwist;
    }

    private double ticksToMeters(double ticks) {
        return Math.PI * 0.096 * ticks/537.7;
    }

    private int metersToTicks(double meters) {
        return (int) (19.2 * meters / (2 * Math.PI * (0.096 / 2.0)));
    }

    private double meterToInches(double meters) {
        return meters * 39.37;
    }

    public double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    public double getAngle(AngleUnit angleUnit) {
        return this.imu.getNormalHeading().toAngleUnit(angleUnit).firstAngle;
    }

    public double[] getPositions() {
        double[] positions = new double[4];
        positions[0] = this.ticksToMeters(this.frontLeft.getCurrentPosition());
        positions[1] = this.ticksToMeters(this.frontRight.getCurrentPosition());
        positions[2] = this.ticksToMeters(this.backLeft.getCurrentPosition());
        positions[3] = this.ticksToMeters(this.backRight.getCurrentPosition());
        return positions;
    }

    public double[] getVelocities() {
        double[] velocities = new double[4];
        velocities[0] = this.ticksToMeters(this.frontLeft.getVelocity());
        velocities[1] = this.ticksToMeters(this.frontRight.getVelocity());
        velocities[2] = this.ticksToMeters(this.backLeft.getVelocity());
        velocities[3] = this.ticksToMeters(this.backRight.getVelocity());
        return velocities;
    }
}
