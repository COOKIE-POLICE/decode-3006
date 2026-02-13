package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.PreferencesLauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.InterpolatedTargetVelocityCalculationStrategy;

public final class SubsystemFactory {

    private SubsystemFactory() {
        throw new AssertionError();
    }

    public static IntakeSubsystem createIntake(HardwareMap hardwareMap) {
        return new IntakeSubsystem.Builder(hardwareMap)
                .motorName("intakerMotor")
                .direction(DcMotorSimple.Direction.FORWARD)
                .build();
    }

    public static LauncherSubsystem createLauncher(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
        return new LauncherSubsystem.Builder(hardwareMap, follower, goalPose)
                .motorName("launcherMotor")
                .direction(DcMotorSimple.Direction.FORWARD)
                .targetVelocityStrategy(new InterpolatedTargetVelocityCalculationStrategy())
                .pidfStrategy(new PreferencesLauncherPIDFCalculationStrategy())
                .build();
    }

    public static BlockerSubsystem createBlocker(HardwareMap hardwareMap) {
        return new BlockerSubsystem.Builder(hardwareMap)
                .servoName("blockerServo")
                .blockPosition(1.0)
                .admitPosition(0.570)
                .build();
    }

    public static IndicatorSubsystem createIndicator(HardwareMap hardwareMap) {
        return new IndicatorSubsystem.Builder(hardwareMap)
                .servoName("indicator")
                .build();
    }

    public static LimelightSubsystem createLimelight(HardwareMap hardwareMap) {
        return new LimelightSubsystem.Builder(hardwareMap)
                .limelightName("limelight")
                .build();
    }
}