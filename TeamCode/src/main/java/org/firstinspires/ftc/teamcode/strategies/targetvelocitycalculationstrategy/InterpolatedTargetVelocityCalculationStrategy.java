package org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class InterpolatedTargetVelocityCalculationStrategy implements TargetVelocityCalculationStrategy {
    private InterpLUT controlPoints;
    public InterpolatedTargetVelocityCalculationStrategy() {
        controlPoints = new InterpLUT();
        controlPoints.add(137.5, 1550);
        controlPoints.add(76.15, 1480);
    }
    @Override
    public double getCalculatedTargetVelocity(double distance) {
        return controlPoints.get(distance);
    }
}
