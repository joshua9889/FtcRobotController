package com.team9889.ftc2021.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.lib.sensors.RevIMU;

/**
 * Example Mecanum Drive with Two Types of Odometry Methods
 */
public class MecanumDrive {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public RevIMU imu;

    public ThreeWheelOdometry threeWheelOdometry = new ThreeWheelOdometry();
    public TwoWheelOdometry twoWheelOdometry = new TwoWheelOdometry();

    public void init(HardwareMap hardwareMap) {

        /**
         *  Motor Set up
         */

        // Get Hardware Map
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        this.frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        this.backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        this.backRight = hardwareMap.get(DcMotorEx.class, "rb");

        // Right side is Reversed
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset All Encoders, Run using Encoders (Speed Control), and Brake at Zero Power.
        for (DcMotorEx motor : new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /**
         * IMU Setup
         */

        // Simplified IMU Object
        this.imu = new RevIMU("imu", hardwareMap);
    }

    /**
     * Send Velocities to Hub
     * @param frontLeftTargetVelocity m/s
     * @param frontRightTargetVelocity m/s
     * @param backLeftTargetVelocity m/s
     * @param backRightTargetVelocity m/s
     */
    public void setWheelVelocities(double frontLeftTargetVelocity,
                                   double frontRightTargetVelocity,
                                   double backLeftTargetVelocity,
                                   double backRightTargetVelocity) {
        this.frontLeft.setVelocity(metersToTicks(frontLeftTargetVelocity));
        this.frontRight.setVelocity(metersToTicks(frontRightTargetVelocity));
        this.backLeft.setVelocity(metersToTicks(backLeftTargetVelocity));
        this.backRight.setVelocity(metersToTicks(backRightTargetVelocity));
    }

    /**
     *  Equations taken from https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
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

    /**
     * Robot Relative Velocity Command
     * @param vx m/s
     * @param vy m/s
     * @param wz rad/s
     */
    public void setTargetTwistRobotRelative(double vx, double vy, double wz){
        double v_fl = vx - vy - (lx + ly) * wz;
        double v_fr = vx + vy + (lx + ly) * wz;
        double v_rl = vx + vy - (lx + ly) * wz;
        double v_rr = vx - vy + (lx + ly) * wz;

        this.setWheelVelocities(v_fl, v_fr, v_rl, v_rr);
    }

    /**
     * Field Relative Velocity Command
     * @param vx m/s
     * @param vy m/s
     * @param wz rad/s
     */
    public void setTargetTwistFieldRelative (double vx, double vy, double wz) {
        double angleOffset = threeWheelOdometry.getPoseEstimate().getHeading();
        double vx_modified = vx * Math.cos(angleOffset) - vy * Math.sin(angleOffset);
        double vy_modified = vx * Math.sin(angleOffset) + vy * Math.cos(angleOffset);

        setTargetTwistRobotRelative(vx_modified, vy_modified, wz);
    }

    /**
     * @param ticks Ticks of the Drive Encoders
     * @return Meters of the Drive Encoders
     */
    public static double ticksToMeters(double ticks) {
        return Math.PI * 0.096 * ticks/537.7;
    }

    /**
     * @param meters Meters of the Drive Encoders
     * @return Ticks of Drive Encoders
     */
    public static int metersToTicks(double meters) {
        return (int) (19.2 * meters / (2 * Math.PI * (0.096 / 2.0)));
    }
}
