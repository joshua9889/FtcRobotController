package com.team9889.ftc2021.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2021.DriverStation;
import com.team9889.ftc2021.subsystems.Intake;
import com.team9889.ftc2021.subsystems.MecanumDrive;

@TeleOp
public class MecanumOpMode extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    DriverStation driverStation = new DriverStation();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        driverStation.init(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            drive.twoWheelOdometry.updateEncoderIMU(
                    intake.backIntake.getCurrentPosition(),
                    intake.frontIntake.getCurrentPosition(),
                    drive.imu.getCurrentRotation()
            );

            double[] velocities = driverStation.getCommandedDriveVelocity();
            drive.setTargetTwistRobotRelative(velocities[0], velocities[1], velocities[2]);

            drive.twoWheelOdometry.update();
        }
    }
}
