package com.team9889.lib.math;

/**
 * Created by joshua9889 on 6/22/2022.
 * Store velocity
 * linear: meters per second
 * angular: radians per second
 */
public class Twist {
    public Vector3 linear;
    public Vector3 angular;

    public Twist(Vector3 linear, Vector3 angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public Twist() {
        this(new Vector3(), new Vector3());
    }

    /**
     *
     * @param acceleration
     * @param dt Milliseconds
     * @return Twist of the acceleration
     */
    public Twist integrateAcceleration(Acceleration acceleration, double dt) {
        Vector3 linear = acceleration.linear.multipleScalar(dt);
        Vector3 angular = acceleration.angular.multipleScalar(dt);
        return new Twist(linear, angular);
    }

    /**
     *
     * @param acceleration
     * @param dt Milliseconds
     * @return Twist of the acceleration
     */
    public Twist integrateAcceleration(Acceleration acceleration, Twist lastTwist, double dt) {
        Vector3 linear = acceleration.linear.multipleScalar(dt).add(lastTwist.linear);
        Vector3 angular = acceleration.angular.multipleScalar(dt).add(lastTwist.angular);
        return new Twist(linear, angular);
    }
}
