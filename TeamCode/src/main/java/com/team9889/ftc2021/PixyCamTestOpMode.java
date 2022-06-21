package com.team9889.ftc2021;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.team9889.lib.Rate;

@TeleOp
@Disabled
public class PixyCamTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        byte[] pixyData;

        I2cDeviceSynch pixyCam;
        pixyCam = hardwareMap.i2cDeviceSynch.get("pixyCam");
        Rate rate = new Rate(60);

        waitForStart();

        while(opModeIsActive()) {
            pixyCam.engage();

            telemetry.addData("! dt", 1. / rate.getRealDt());

            int[] registerList = new int[]{0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};
            int register = 0x50;
            pixyData = pixyCam.read(register, 5);
            int count = 0;
            for (int data : pixyData) {
                telemetry.addData(register + " | " + count, data);
                count++;
            }

            telemetry.update();
            rate.sleep();
        }
    }
}

