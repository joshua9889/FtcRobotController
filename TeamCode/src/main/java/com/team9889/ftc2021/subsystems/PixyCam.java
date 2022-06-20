package com.team9889.ftc2021.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PixyCam {

    //Declare a new I2cDeviceSync
    I2cDeviceSynch pixy;

    double x, y, width, height, numObjects;

    byte[] pixyData;

    public PixyCam() {}

    public void init(HardwareMap hardwareMap) {

        //Get name from hardware config
        pixy = hardwareMap.i2cDeviceSynch.get("pixyCam");

        pixy.setI2cAddress(I2cAddr.create7bit(0x54));

        //setting Pixy's read window. You'll want these exact parameters, and you can reference the
        // SDK Documentation to learn more
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,
                I2cDeviceSynch.ReadMode.REPEAT);
        pixy.setReadWindow(readWindow);
    }

    public void update() {
        pixy.engage();

        pixyData = pixy.read(0x51, 5);

        x = pixyData[1];
        y = pixyData[2];
        width = pixyData[3];
        height = pixyData[4];
        numObjects = pixyData[0];
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("0", 0xff&pixyData[0]);
        telemetry.addData("1", 0xff&pixyData[1]);
        telemetry.addData("2", 0xff&pixyData[2]);
        telemetry.addData("3", 0xff&pixyData[3]);
        telemetry.addData("4", 0xff&pixyData[4]);
        telemetry.addData("Length", pixyData.length);
        //send every byte of data that we can to the phone screen
//        telemetry.addData("Byte 0", pixy.read8(0));
//        telemetry.addData("Byte 1", pixy.read8(1));
//        telemetry.addData("Byte 2", pixy.read8(2));
//        telemetry.addData("Byte 3", pixy.read8(3));
//        telemetry.addData("Byte 4", pixy.read8(4));
//        telemetry.addData("Byte 5", pixy.read8(5));
//        telemetry.addData("Byte 6", pixy.read8(6));
//        telemetry.addData("Byte 7", pixy.read8(7));
//        telemetry.addData("Byte 8", pixy.read8(8));
//        telemetry.addData("Byte 9", pixy.read8(9));
//        telemetry.addData("Byte 10", pixy.read8(10));
//        telemetry.addData("Byte 11", pixy.read8(11));
//        telemetry.addData("Byte 12", pixy.read8(12));
//        telemetry.addData("Byte 13", pixy.read8(13));
//        telemetry.update();
    }
}
