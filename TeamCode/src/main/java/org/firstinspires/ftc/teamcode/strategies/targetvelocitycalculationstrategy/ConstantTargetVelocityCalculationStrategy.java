package org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy;

public class ConstantTargetVelocityCalculationStrategy implements TargetVelocityCalculationStrategy {
    @Override
    public double getCalculatedTargetVelocity(double distance) {
        return 1550;
    }
}
