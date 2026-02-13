package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private LLResult latestResult;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, Preferences.LIMELIGHT);
        limelight.start();
    }
    public void shutdown() {
        limelight.shutdown();
    }
    public boolean hasValidTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public double getTargetX() {
        if (!hasValidTarget()) {
            return 0.0;
        }
        return latestResult.getTx();
    }

    public double getTargetY() {
        if (!hasValidTarget()) {
            return 0.0;
        }
        return latestResult.getTy();
    }

    public double getTargetArea() {
        if (!hasValidTarget()) {
            return 0.0;
        }
        return latestResult.getTa();
    }

    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }
}