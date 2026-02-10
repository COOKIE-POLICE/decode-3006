package org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy;

import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.LauncherPIDFCalculationStrategy;

public class SmartLauncherPIDFCalculationStrategy implements LauncherPIDFCalculationStrategy {

    public static final double kV = 0.0;
    private final InterpLUT pLUT;

    public SmartLauncherPIDFCalculationStrategy() {
        pLUT = new InterpLUT();
        pLUT.add(0.0, 0.1);
        pLUT.add(0.005, 0.5);
        pLUT.add(0.01, 1);
        pLUT.add(0.02, 2);
        pLUT.add(0.05, 10);
        pLUT.add(1.00, 100);
        pLUT.add(10.00, 1000);
        pLUT.createLUT();
    }

    @Override
    public PIDFCoefficients getCalculatedPidf(
            double targetVelocity,
            double batteryVoltage,
            double currentVelocity
    ) {
        double feedforward = kV;

        double error = targetVelocity - currentVelocity;
        double absPercentError = Math.abs(error) / targetVelocity;
        double proportional = pLUT.get(absPercentError);

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
