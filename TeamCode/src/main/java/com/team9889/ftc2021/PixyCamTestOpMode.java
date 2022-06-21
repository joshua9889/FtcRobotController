package com.team9889.ftc2021;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.team9889.lib.Rate;

@TeleOp
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
//            for (int register=79; register < 87; register++) {
            pixyData = pixyCam.read(register, 5);
            int count = 0;
            for (int data : pixyData) {
                telemetry.addData(register + " | " + count, data);
                count++;
            }
//        }

//            telemetry.addData("Lenth", pixyData[0]);

//            int x = pixyData[1];
//            int y = pixyData[2];
//            int width = pixyData[3];
//            int height = pixyData[4];
//
//            int left_point_x = x - (width/2);
//            int left_point_y = y - (height/2);
//
//            int right_point_x = x + (width/2);
//            int right_point_y = y + (height/2);

//            telemetry.addData("Length", pixyData[0]);
//            telemetry.addData("1 X", x);
//            telemetry.addData("1 Y", y);
//            telemetry.addData("1 Width", width);
//            telemetry.addData("1 Height", height);
//
//            telemetry.addData("2 Left X", left_point_x);
//            telemetry.addData("2 Left Y", left_point_y);
//            telemetry.addData("2 Right X", right_point_x);
//            telemetry.addData("2 Right Y", right_point_y);

            telemetry.update();
            rate.sleep();
        }
    }
}

