package com.team9889.ftc2021.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.List;

/**
 * Created by joshua9889 on 9/24/2022.
 */
public class ThreeWheelOdometry  extends ThreeTrackingWheelLocalizer {

    private double leftEncoderM, rightEncoderM, centerEncoderM;

    public void updateEncoderPositions(double l, double r, double c) {
        leftEncoderM = MecanumDrive.ticksToMeters(l);
        rightEncoderM = MecanumDrive.ticksToMeters(r);
        centerEncoderM = MecanumDrive.ticksToMeters(c);
    }

    public ThreeWheelOdometry() {

        // Locations of Wheels Relative to (0,0,0) on Robot
        super(Arrays.asList(
                new Pose2d(-1.375, -7.5, 0),
                new Pose2d(2.56625, 7.5, 0),
                new Pose2d(0.25, 7.25, Math.toRadians(90))
        ));
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                leftEncoderM, rightEncoderM, centerEncoderM
        );
    }
}