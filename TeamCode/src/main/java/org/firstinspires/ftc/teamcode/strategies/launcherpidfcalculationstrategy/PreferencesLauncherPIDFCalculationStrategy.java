package org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Preferences;

public class PreferencesLauncherPIDFCalculationStrategy implements LauncherPIDFCalculationStrategy {

    @Override
    public PIDFCoefficients getCalculatedPidf(double targetVelocity, double batteryVoltage, double currentVelocity) {
        return Preferences.Launcher.pidfCoefficients;
    }
}
