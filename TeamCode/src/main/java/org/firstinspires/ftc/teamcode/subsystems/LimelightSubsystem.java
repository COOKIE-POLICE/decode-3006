package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;

    private LimelightSubsystem(Builder builder) {
        this.limelight = builder.hardwareMap.get(Limelight3A.class, builder.limelightName);
        this.limelight.start();
    }

    public static final class Builder {
        private final HardwareMap hardwareMap;
        private String limelightName = Preferences.LIMELIGHT;

        public Builder(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        public Builder limelightName(String limelightName) {
            this.limelightName = limelightName;
            return this;
        }

        public LimelightSubsystem build() {
            return new LimelightSubsystem(this);
        }
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }

    public void shutdown() { limelight.shutdown(); }

    public boolean hasValidTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public double getTargetX() { return hasValidTarget() ? latestResult.getTx() : 0.0; }
    public double getTargetY() { return hasValidTarget() ? latestResult.getTy() : 0.0; }
    public double getTargetArea() { return hasValidTarget() ? latestResult.getTa() : 0.0; }
}