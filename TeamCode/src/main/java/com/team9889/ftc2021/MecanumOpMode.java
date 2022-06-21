package com.team9889.ftc2021;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2021.subsystems.MecanumDrive;
import com.team9889.lib.Rate;
import com.team9889.lib.Twist;

@TeleOp
public class MecanumOpMode extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();

    DriverStation driverStation = new DriverStation();
    Rate rate = new Rate(50);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive.init(hardwareMap);
        driverStation.init(gamepad1, gamepad2);

        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("Waiting for Start", "");
        telemetry.update();

        waitForStart();

        telemetry.clear();

        while (opModeIsActive()) {
            Twist command = driverStation.getTwistCommand();
            drive.setTargetTwist(command);

            drive.update();

            double dt = rate.getRealDt();
            telemetry.addData("Loop dt (ms)", dt * 1000);
            telemetry.addData("Loop Hz", 1. / dt);
            drive.updateTelemetry(telemetry);
            telemetry.update();

            rate.sleep();
        }
    }
}
