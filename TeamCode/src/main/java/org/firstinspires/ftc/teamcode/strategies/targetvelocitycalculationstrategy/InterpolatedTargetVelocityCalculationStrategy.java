package org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class InterpolatedTargetVelocityCalculationStrategy implements TargetVelocityCalculationStrategy {
    private InterpLUT controlPoints;
    public InterpolatedTargetVelocityCalculationStrategy() {
        controlPoints = new InterpLUT();
        controlPoints.add(0, 0);
        controlPoints.add(49.66, 1625);
        controlPoints.add(65.43, 1650);
        controlPoints.add(82.82, 1700);
        controlPoints.add(91.74, 1750);
        controlPoints.add(93.37, 1750);
        controlPoints.add(100.26, 1750);
        controlPoints.add(110.72, 1850);
        controlPoints.add(119.71, 1900);
        controlPoints.add(138.90, 1950); // good
        controlPoints.add(155.60, 2025);
        controlPoints.add(170.1182, 2230);
        controlPoints.add(200.0, 2400); // !!!-------p
        controlPoints.add(240.0, 2600);
        controlPoints.add(300.0, 3000);
        controlPoints.createLUT();


    }
    @Override
    public double getCalculatedTargetVelocity(double distance) {
        return controlPoints.get(distance);
    }
}
