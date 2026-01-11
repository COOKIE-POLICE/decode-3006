package org.firstinspires.ftc.teamcode.modules;

public class BangBangController {
    private final double bangBangThreshold;
    private final double bangBangPower;

    public BangBangController(double bangBangThreshold, double bangBangPower) {
        this.bangBangThreshold = bangBangThreshold;
        this.bangBangPower = bangBangPower;
    }

    public boolean shouldUseBangBang(double error) {
        return Math.abs(error) > bangBangThreshold;
    }

    public double calculate(double error) {
        if (error > bangBangThreshold) {
            return bangBangPower;
        } else if (error < -bangBangThreshold) {
            return 0.0;
        }
        return 0.0;
    }

    public double getBangBangThreshold() {
        return bangBangThreshold;
    }
}