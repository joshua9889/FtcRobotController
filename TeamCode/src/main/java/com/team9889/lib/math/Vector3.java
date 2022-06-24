package com.team9889.lib.math;

/**
 * Created by joshua9889 on 6/22/2022.
 * Vector in space
 */
public class Vector3 {
    public double x, y, z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3() {
        this.x = 0.f;
        this.y = 0.f;
        this.z = 0.f;
    }

    public Vector3 multipleScalar(Vector3 vector, double scale) {
        Vector3 vectorCopy = vector.copy();
        vectorCopy.x *= scale;
        vectorCopy.y *= scale;
        vectorCopy.z *= scale;
        return vectorCopy;
    }

    public Vector3 multipleScalar(double scale) {
        Vector3 vectorCopy = this.copy();
        vectorCopy.x *= scale;
        vectorCopy.y *= scale;
        vectorCopy.z *= scale;
        return vectorCopy;
    }

    public Vector3 add(Vector3 vector1, Vector3 vector2) {
        Vector3 vector = vector1.copy();
        vector.x += vector2.x;
        vector.y += vector2.y;
        vector.z += vector2.z;
        return vector;
    }

    public Vector3 add(Vector3 vector1) {
        Vector3 vector = this.copy();
        vector.x += vector1.x;
        vector.y += vector1.y;
        vector.z += vector1.z;
        return vector;
    }

    public Vector3 copy() {
        return new Vector3(this.x, this.y, this.z);
    }
}
