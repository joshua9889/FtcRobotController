package com.team9889.ftc2021;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2021.subsystems.MecanumDrive;

@TeleOp
public class MecanumOpMode extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();

    DriverStation driverStation = new DriverStation();
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        driverStation.init(gamepad1, gamepad2);

        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("Waiting", "for Start");
        telemetry.update();

        waitForStart();

        telemetry.clear();

        while (opModeIsActive()) {
            drive.setPower(driverStation.getX(),driverStation.getY(),driverStation.getAngular());

            drive.update();

            telemetry.addData("Loop dt (ms)", Math.round(loopTimer.milliseconds()));
            drive.updateTelemetry(telemetry);
            loopTimer.reset();
        }
    }
}
