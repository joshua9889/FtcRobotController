package com.team9889.ftc2021.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2021.DriverStation;
import com.team9889.ftc2021.subsystems.MecanumDrive;

@TeleOp
public class MecanumOpMode extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();
    DriverStation driverStation = new DriverStation();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        driverStation.init(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            double[] velocities = driverStation.getCommandedDriveVelocity();
            drive.setTargetTwistRobotRelative(velocities[0], velocities[1], velocities[2]);

            drive.odometry.update();
        }
    }
}
