package org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SmartLauncherPIDFCalculationStrategy implements LauncherPIDFCalculationStrategy {
    public static final double kV = 16.625/1550;
    public static final double nominalVoltage = 12.0;

    @Override
    public PIDFCoefficients getCalculatedPidf(double targetVelocity, double batteryVoltage) {
        double feedforward = kV * targetVelocity * (nominalVoltage / batteryVoltage);
        return new PIDFCoefficients(0.075, 0, 0, feedforward);
    }
}
