package com.team9889.lib;

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
}
