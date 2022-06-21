package com.team9889.ftc2021;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.team9889.lib.Twist;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class DriverStation {
    private Gamepad gamepad1, gamepad2;

    public void init(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    private final double maxVelocity = (312./60.) * Math.PI * 0.096;    // m/s
    private final double maxAngularVelocity = Math.PI;                  // Rad/s

    public Twist getTwistCommand() {
        Twist twistCommand = new Twist();
        twistCommand.translationalVelocity.unit = DistanceUnit.METER;

        twistCommand.translationalVelocity.xVeloc = -gamepad1.left_stick_y * maxVelocity;
        twistCommand.translationalVelocity.yVeloc =  gamepad1.left_stick_x * maxVelocity;

        twistCommand.angularVelocity.unit = AngleUnit.RADIANS;
        twistCommand.angularVelocity.zRotationRate = (float) (gamepad1.right_stick_x * maxAngularVelocity);

        return twistCommand;
    }
}
