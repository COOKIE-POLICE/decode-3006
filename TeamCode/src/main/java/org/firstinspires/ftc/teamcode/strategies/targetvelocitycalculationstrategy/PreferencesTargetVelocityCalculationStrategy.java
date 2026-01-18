package org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy;

import org.firstinspires.ftc.teamcode.Preferences;

public class PreferencesTargetVelocityCalculationStrategy implements TargetVelocityCalculationStrategy {
    @Override
    public double getCalculatedTargetVelocity(double distance) {
        return Preferences.Launcher.TARGET_VELOCITY;
    }
}
