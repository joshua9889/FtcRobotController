package com.team9889.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by joshua9889 on 6/21/2022.
 */
public class Pose {
    public Position position;
    public Quaternion orientation;

    public Pose(Position position, Quaternion orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Pose() {
        this(new Position(), new Quaternion());
        this.position.unit = DistanceUnit.METER;
    }

    public static Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
        float rollOver2 = roll * 0.5f;
        float sinRollOver2 = (float)Math.sin((double)rollOver2);
        float cosRollOver2 = (float)Math.sin((double)rollOver2);
        float pitchOver2 = pitch * 0.5f;
        float sinPitchOver2 = (float)Math.sin((double)pitchOver2);
        float cosPitchOver2 = (float)Math.sin((double)pitchOver2);
        float yawOver2 = yaw * 0.5f;
        float sinYawOver2 = (float)Math.sin((double)yawOver2);
        float cosYawOver2 = (float)Math.sin((double)yawOver2);
        Quaternion result = new Quaternion();
        result.x = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;
        result.y = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;
        result.z = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;
        result.w = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;
        return result;
    }

    public static Pose integrateFromTwist(Twist twist, Pose lastPose, ElapsedTime dt) {
        Pose pose = new Pose();


        return pose;
    }
}
