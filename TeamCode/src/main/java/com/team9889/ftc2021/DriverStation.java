package com.team9889.ftc2021;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriverStation {
    private Gamepad gamepad1, gamepad2;

    public void init(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public double getX() {
        return gamepad1.left_stick_y;
    }

    public double getY() {
        return gamepad1.left_stick_x;
    }

    public double getAngular() {
        return gamepad1.right_stick_x;
    }
}
