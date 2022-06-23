package com.team9889.lib;

/**
 * Created by joshua9889 on 6/22/2022.
 * Vector in space
 */
public class Vector3 {
    public double x, y, z;

    public Vector3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3() {
        this.x = 0.f;
        this.y = 0.f;
        this.z = 0.f;
    }
}
