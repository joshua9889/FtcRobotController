package com.team9889.ftc2021;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2021.subsystems.MecanumDrive;

@TeleOp
@Disabled
public class TestMecanumOpMode extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive.init(hardwareMap, false);

        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("Waiting for Start", "");
        telemetry.update();

        waitForStart();

        timer.reset();

        while (opModeIsActive() && timer.seconds() < 5) {
            drive.setPower(1, 0,0,0);

            drive.update();

            telemetry.addData("Motor", "Front Left");
            drive.updateTelemetry(telemetry);
            telemetry.update();
        }

        drive.setPower(0, 0,0,0);
        sleep(5000);
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 5) {
            drive.setPower(0, 1,0,0);

            drive.update();

            telemetry.addData("Motor", "Front Right");
            drive.updateTelemetry(telemetry);
            telemetry.update();

        }

        drive.setPower(0, 0,0,0);
        sleep(5000);
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 5) {
            drive.setPower(0, 0,1,0);

            drive.update();

            telemetry.addData("Motor", "Back Left");
            drive.updateTelemetry(telemetry);
            telemetry.update();
        }

        drive.setPower(0, 0,0,0);
        sleep(5000);
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 5) {
            drive.setPower(0, 0,0,1);

            drive.update();

            telemetry.addData("Motor", "Back Right");
            drive.updateTelemetry(telemetry);
            telemetry.update();
        }

        drive.setPower(0, 0,0,0);
        sleep(10000);
        timer.reset();


    }
}
