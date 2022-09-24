package com.team9889.ftc2021.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.lib.sensors.RevIMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

public class MecanumDrive {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public RevIMU imu;

    public Odometry odometry = new Odometry();

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

        DcMotorEx[] motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};

        // Reset All Encoders, Run using Encoders (Speed Control), and Brake at Zero Power.
        for (DcMotorEx motor : motors) {
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

    public void setWheelVelocities(double frontLeftTargetVelocity, double frontRightTargetVelocity, double backLeftTargetVelocity, double backRightTargetVelocity) {
        this.frontLeft.setVelocity(metersToTicks(frontLeftTargetVelocity));
        this.frontRight.setVelocity(metersToTicks(frontRightTargetVelocity));
        this.backLeft.setVelocity(metersToTicks(backLeftTargetVelocity));
        this.backRight.setVelocity(metersToTicks(backRightTargetVelocity));
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

    public void setTargetTwistRobotRelative(double vx, double vy, double wz){
        double v_fl = vx - vy - (lx + ly) * wz;
        double v_fr = vx + vy + (lx + ly) * wz;
        double v_rl = vx + vy - (lx + ly) * wz;
        double v_rr = vx - vy + (lx + ly) * wz;

        this.setWheelVelocities(v_fl, v_fr, v_rl, v_rr);
    }

    public void setTargetTwistFieldRelative (double vx, double vy, double wz) {
        double angleOffset = odometry.getPoseEstimate().getHeading();
        double vx_modified = vx * Math.cos(angleOffset) - vy * Math.sin(angleOffset);
        double vy_modified = vx * Math.sin(angleOffset) + vy * Math.cos(angleOffset);

        setTargetTwistRobotRelative(vx_modified, vy_modified, wz);
    }

    private double ticksToMeters(double ticks) {
        return Math.PI * 0.096 * ticks/537.7;
    }

    private int metersToTicks(double meters) {
        return (int) (19.2 * meters / (2 * Math.PI * (0.096 / 2.0)));
    }


    /**
     * Pose Estimation Using Road Runner ThreeTrackingWheelLocalizer
     */

    public class Odometry extends ThreeTrackingWheelLocalizer {

        private double leftEncoderM, rightEncoderM, centerEncoderM;

        public void updateEncoderPositions(double l, double r, double c) {
            leftEncoderM = l;
            rightEncoderM = r;
            centerEncoderM = c;
        }

        public Odometry() {

            // Locations of Wheels Relative to (0,0,0) on Robot
            super(Arrays.asList(
                    new Pose2d(-1.375, -7.5, 0),
                    new Pose2d(2.56625, 7.5, 0),
                    new Pose2d(0.25, 7.25, Math.toRadians(90))
            ));
        }

        @Override
        public List<Double> getWheelPositions() {
            return Arrays.asList(
                leftEncoderM, rightEncoderM, centerEncoderM
            );
        }
    }
}
