package com.team9889.ftc2021.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.team9889.ftc2021.subsystems.Intake;
import com.team9889.ftc2021.subsystems.MecanumDrive;

/**
 * Created by joshua9889 on 9/23/2022.
 */
public class RedSideAuto extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {

        // Update the Encoder Values
        drive.threeWheelOdometry.updateEncoderPositions(
                intake.backIntake.getCurrentPosition(),
                intake.intakeTransfer.getCurrentPosition(),
                intake.frontIntake.getCurrentPosition()
        );

        // Update the Pose Estimation
        drive.threeWheelOdometry.update();

        // Current Pose Estimation of the robot on the field
        Pose2d estimate = drive.threeWheelOdometry.getPoseEstimate();

        // Target Pose
        Pose2d target = new Pose2d(1, 0, 0);

        // Difference of Pose (Error)
        Pose2d difference = target.minus(estimate);

        // Simple Proportional Controller, one for translation, one for rotation
        double kp_t = 4;
        Pose2d translationResponse = difference.times(kp_t);

        double kp_r = 0.7;
        Pose2d rotationalResponse = difference.times(kp_r);

        // Set the velocity of the robot
        drive.setTargetTwistFieldRelative(translationResponse.getX(), translationResponse.getY(), rotationalResponse.getHeading());
    }
}
