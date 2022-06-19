package com.team9889.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//I dont think this works
public class CruiseControlAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {
    BNO055IMU.Parameters parameters;
    Acceleration acceleration;

    ElapsedTime timer = new ElapsedTime();
    Velocity integratedVelocity = new Velocity(), initialVelocity = new Velocity();
    Position integratedPosition = new Position(), initialPosition = new Position();

    @Override
    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.initialVelocity = initialVelocity;
        this.initialPosition = initialPosition;
    }

    @Override
    public Position getPosition() {
        return new Position();
    }

    @Override
    public Velocity getVelocity() {
        return integratedVelocity;
    }

    @Override
    public Acceleration getAcceleration() {
        RobotLog.d("IMU TESTING", "HEY");
        return this.acceleration == null ? new Acceleration() : this.acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        double dt = timer.seconds();
        RobotLog.d("IMU TESTING", String.valueOf(dt));
        acceleration = linearAcceleration;

        integratedVelocity.xVeloc += acceleration.xAccel * dt - initialVelocity.xVeloc;
        integratedVelocity.yVeloc += acceleration.yAccel * dt - initialVelocity.yVeloc;
        integratedVelocity.zVeloc += acceleration.zAccel * dt - initialVelocity.zVeloc;
        timer.reset();
    }
}
