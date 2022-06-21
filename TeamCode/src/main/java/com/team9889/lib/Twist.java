package com.team9889.lib;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Class to combine Velocity and AngularVelocity data
 * Inspired by the ROS standard message Twist
 */
public class Twist {
    public Velocity translationalVelocity;
    public AngularVelocity angularVelocity;

    public Twist(Velocity translationalVelocity, AngularVelocity angularVelocity) {
        this.translationalVelocity = translationalVelocity;
        this.angularVelocity = angularVelocity;
    }

    public Twist() {
        this(new Velocity(), new AngularVelocity());
    }

    public void toTelemetry(String prefix, Telemetry telemetry) {
        Velocity vel = this.translationalVelocity.toUnit(DistanceUnit.METER);
        AngularVelocity deg = this.angularVelocity.toAngleUnit(AngleUnit.RADIANS);

        telemetry.addData(prefix + " xVel", vel.xVeloc);
        telemetry.addData(prefix + " yVel", vel.yVeloc);
        telemetry.addData(prefix + " zVel", vel.zVeloc);
        telemetry.addData(prefix + " x rad/s", deg.xRotationRate);
        telemetry.addData(prefix + " y rad/s", deg.yRotationRate);
        telemetry.addData(prefix + " z rad/s", deg.zRotationRate);
    }
}
