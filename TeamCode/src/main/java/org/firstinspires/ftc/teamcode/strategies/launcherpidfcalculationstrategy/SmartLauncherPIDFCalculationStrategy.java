package org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy;

import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SmartLauncherPIDFCalculationStrategy implements LauncherPIDFCalculationStrategy {

    public static final double kV = 12.0;
    public static final double nominalVoltage = 13.7;

    // Asymmetric P gains
    private static final double P_UP = 1.5;
    private static final double P_DOWN = 0.005;

    @Override
    public PIDFCoefficients getCalculatedPidf(
            double targetVelocity,
            double batteryVoltage,
            double currentVelocity
    ) {
        double feedforward = kV * (nominalVoltage / batteryVoltage);

        double proportional =
                currentVelocity < targetVelocity ? P_UP : P_DOWN;
        double derivative = 0.0;
        double integral = 0.0;

        return new PIDFCoefficients(
                proportional,
                integral,
                derivative,
                feedforward,
                MotorControlAlgorithm.PIDF
        );
    }
}
