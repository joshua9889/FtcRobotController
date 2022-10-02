package com.team9889.ftc2021.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.team9889.ftc2021.subsystems.MecanumDrive;

import java.util.Arrays;
import java.util.List;

/**
 * Created by joshua9889 on 9/24/2022.
 */
public class TwoWheelOdometry extends TwoTrackingWheelLocalizer {

    private double forwardEncoder, sidewaysEncoder;
    private double imuAngle;

    public void updateEncoderIMU(double l, double c, double angle) {
        forwardEncoder = MecanumDrive.ticksToMeters(l);
        sidewaysEncoder = MecanumDrive.ticksToMeters(c);
        imuAngle = angle;
    }

    public TwoWheelOdometry() {
        super(Arrays.asList(
                new Pose2d(-1.375, -7.5, 0),
                new Pose2d(0.25, 7.25, Math.toRadians(90))
        ));
    }

    @Override
    public double getHeading() {
        return imuAngle;
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                forwardEncoder, sidewaysEncoder
        );
    }
}