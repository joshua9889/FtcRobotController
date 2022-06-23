package com.team9889.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by joshua9889 on 6/21/2022.
 */
public class Pose {
    public Position position;
    public Orientation orientation;

    public Pose(Position position, Orientation orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Pose() {
        this(new Position(), new Orientation());
        this.position.unit = DistanceUnit.METER;
        this.orientation.angleUnit = AngleUnit.RADIANS;
        this.orientation.axesOrder = AxesOrder.ZXY;
    }

    public static Pose integrateXYFromTwist(Twist twist, Pose lastPose, ElapsedTime dt) {
        Pose pose = new Pose();

        Velocity velocity = twist.translationalVelocity.toUnit(DistanceUnit.METER);
        AngularVelocity angularVelocity = twist.angularVelocity.toAngleUnit(AngleUnit.RADIANS);

        double dt_sec = dt.seconds();
        double d_angle = angularVelocity.zRotationRate * dt_sec;

        Position dP = new Position();
        dP.x = velocity.xVeloc * dt_sec * Math.cos(d_angle) + velocity.yVeloc * dt_sec * Math.sin(d_angle);
        dP.y = velocity.yVeloc * dt_sec * Math.cos(d_angle) + velocity.xVeloc * dt_sec * Math.sin(d_angle);

        pose.position.x = dP.x + lastPose.position.x;
        pose.position.y = dP.y + lastPose.position.y;
        pose.orientation.firstAngle = (float) (d_angle * dt_sec + lastPose.orientation.firstAngle);

        dt.reset();
        return pose;
    }
}
