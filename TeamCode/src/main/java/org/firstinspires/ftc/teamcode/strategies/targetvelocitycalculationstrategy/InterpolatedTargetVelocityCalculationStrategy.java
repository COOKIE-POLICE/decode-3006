package org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class InterpolatedTargetVelocityCalculationStrategy implements TargetVelocityCalculationStrategy {
    private InterpLUT controlPoints;
    public InterpolatedTargetVelocityCalculationStrategy() {
        controlPoints = new InterpLUT();
        controlPoints.add(0, 0);
        controlPoints.add(53.45, 1450);
        controlPoints.add(61.37, 1550); // !!!
        controlPoints.add(81.48, 1650);
        controlPoints.add(105.68, 1775);
        controlPoints.add(122.08, 1900);
        controlPoints.add(138.90, 1950); // good
        controlPoints.add(155.60, 2025);
        controlPoints.add(170.1182, 2230);
        controlPoints.add(200.0, 2400); // !!!
        controlPoints.add(240.0, 2600);
        controlPoints.createLUT();


    }
    @Override
    public double getCalculatedTargetVelocity(double distance) {
        return controlPoints.get(distance);
    }
}
