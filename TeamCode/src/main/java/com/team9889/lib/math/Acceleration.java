package com.team9889.lib.math;

import com.team9889.lib.math.Vector3;

/**
 * Created by joshua9889 on 6/22/2022.
 * Store velocity
 * linear: meters per second^2
 * angular: radians per second^2
 */
public class Acceleration {
    public Vector3 linear;
    public Vector3 angular;

    public Acceleration(Vector3 linear, Vector3 angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public Acceleration() {
        this(new Vector3(), new Vector3());
    }
}
