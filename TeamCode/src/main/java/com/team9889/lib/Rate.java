package com.team9889.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Rate {

    private ElapsedTime timer = new ElapsedTime();
    private double lastDt = 0, realDt = 0;
    private double hz = 100;

    public Rate (double targetHz) {
        this.hz = targetHz;
    }

    public void sleep() {
        lastDt = timer.seconds();

        while (timer.seconds() < 1.0 / this.hz)
            Thread.yield();

        realDt = timer.seconds();
        timer.reset();
    }

    public double getDT() {
        return lastDt;
    }

    public double getRealDt() {
        return realDt;
    }
}
