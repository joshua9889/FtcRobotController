package com.team9889.ftc2021;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriverStation {
    private Gamepad gamepad1, gamepad2;

    public void init(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public static double maxVelocity = (312./60.) * Math.PI * 0.096;    // m/s
    public static double maxAngularVelocity = 2 * Math.PI;              // Rad/s

    public double[] getCommandedDriveVelocity() {
        double x = -gamepad1.left_stick_y * maxVelocity;
        double y =  -gamepad1.left_stick_x * maxVelocity;
        double w = -gamepad1.right_stick_x * maxAngularVelocity;

        return new double[] {x, y, w};
    }
}
