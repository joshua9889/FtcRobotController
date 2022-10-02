package com.team9889.ftc2021.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor backIntake, frontIntake, intakeTransfer;

    public void init(HardwareMap hardwareMap) {
        backIntake = hardwareMap.get(DcMotor.class, "back");
        frontIntake = hardwareMap.get(DcMotor.class, "front");
        intakeTransfer = hardwareMap.get(DcMotor.class, "transfer");
    }
}
