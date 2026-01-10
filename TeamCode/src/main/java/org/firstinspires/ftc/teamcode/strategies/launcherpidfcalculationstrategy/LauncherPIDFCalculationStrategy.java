package org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public interface LauncherPIDFCalculationStrategy {
    PIDFCoefficients getCalculatedPidf(double targetVelocity, double batteryVoltage);
}
