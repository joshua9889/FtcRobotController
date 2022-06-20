package com.team9889.ftc2021;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp
public class PixyCamTestOpMode extends LinearOpMode {
    I2cDeviceSynch pixyCam;

    double x, y, width, height, numObjects;

    byte[] pixyData;

    @Override
    public void runOpMode() throws InterruptedException {

        pixyCam = hardwareMap.i2cDeviceSynch.get("pixyCam");


        waitForStart();

        while(opModeIsActive()){
            pixyCam.engage();

            pixyData = pixyCam.read(0x5c, 8);

            x = pixyData[1];
            y = pixyData[2];
            width = pixyData[3];
            height = pixyData[4];
            numObjects = pixyData[0];

            int counter = 0;
            for (int data :pixyData) {
                telemetry.addData(String.valueOf(counter), data);
                counter++;
            }
//            telemetry.addData("0", 0xff&pixyData[0]);
//            telemetry.addData("1", 0xff&pixyData[1]);
//            telemetry.addData("2", 0xff&pixyData[2]);
//            telemetry.addData("3", 0xff&pixyData[3]);
//            telemetry.addData("4", 0xff&pixyData[4]);
//            telemetry.addData("X", x);
//            telemetry.addData("Y", y);
//            telemetry.addData("width", width);
//            telemetry.addData("height", height);

            telemetry.addData("Length", pixyData.length);
            telemetry.update();
            sleep (500);
        }

    }
}
