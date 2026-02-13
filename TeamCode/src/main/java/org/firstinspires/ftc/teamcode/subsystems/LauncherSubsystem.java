package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.LauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.PreferencesLauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.InterpolatedTargetVelocityCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.TargetVelocityCalculationStrategy;
import org.firstinspires.ftc.teamcode.modules.TunedMotor;

public class LauncherSubsystem extends SubsystemBase {

    private static final double VELOCITY_TOLERANCE = 1.0;

    private final TunedMotor tunedMotor;
    private final VoltageSensor battery;
    private final Follower follower;
    private final Pose goalPose;
    private final TargetVelocityCalculationStrategy targetVelocityStrategy;
    private final LauncherPIDFCalculationStrategy pidfStrategy;

    private boolean isLaunching;

    private LauncherSubsystem(Builder builder) {
        DcMotorEx motor = builder.hardwareMap.get(DcMotorEx.class, builder.motorName);
        this.battery = builder.hardwareMap.voltageSensor.iterator().next();
        this.follower = builder.follower;
        this.goalPose = builder.goalPose;
        this.targetVelocityStrategy = builder.targetVelocityStrategy;
        this.pidfStrategy = builder.pidfStrategy;
        this.isLaunching = false;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients initialPidf = pidfStrategy.getCalculatedPidf(0, battery.getVoltage(), 0);
        this.tunedMotor = new TunedMotor(motor, initialPidf.p, initialPidf.i, initialPidf.d, initialPidf.f);
        this.tunedMotor.setTolerance(0.0);
        this.tunedMotor.setDirection(builder.direction);
    }

    public static final class Builder {
        private final HardwareMap hardwareMap;
        private final Follower follower;
        private final Pose goalPose;
        private String motorName = Preferences.LAUNCHER_MOTOR;
        private DcMotorSimple.Direction direction = Preferences.Launcher.DIRECTION;
        private TargetVelocityCalculationStrategy targetVelocityStrategy = new InterpolatedTargetVelocityCalculationStrategy();
        private LauncherPIDFCalculationStrategy pidfStrategy = new PreferencesLauncherPIDFCalculationStrategy();

        public Builder(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
            this.hardwareMap = hardwareMap;
            this.follower = follower;
            this.goalPose = goalPose;
        }

        public Builder motorName(String motorName) {
            this.motorName = motorName;
            return this;
        }

        public Builder direction(DcMotorSimple.Direction direction) {
            this.direction = direction;
            return this;
        }

        public Builder targetVelocityStrategy(TargetVelocityCalculationStrategy strategy) {
            this.targetVelocityStrategy = strategy;
            return this;
        }

        public Builder pidfStrategy(LauncherPIDFCalculationStrategy strategy) {
            this.pidfStrategy = strategy;
            return this;
        }

        public LauncherSubsystem build() {
            return new LauncherSubsystem(this);
        }
    }

    @Override
    public void periodic() {
        if (!isLaunching) return;

        double targetVelocity = getTargetVelocity();
        PIDFCoefficients pidf = getCurrentPidf();

        tunedMotor.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
        tunedMotor.setTargetVelocity(targetVelocity);
        tunedMotor.update(getVelocity());
    }

    public void startLaunching() { isLaunching = true; }

    public void stopLaunching() {
        tunedMotor.getMotor().setPower(0.0);
        isLaunching = false;
    }

    public boolean isLaunching() { return isLaunching; }

    public boolean isAtTargetVelocity() { return getPercentError() < VELOCITY_TOLERANCE; }

    public double getTargetVelocity() {
        return targetVelocityStrategy.getCalculatedTargetVelocity(getDistanceToGoal());
    }

    public double getVelocity() { return Math.abs(tunedMotor.getVelocity()); }

    public double getPercentError() {
        double target = getTargetVelocity();
        return target == 0.0 ? 0.0 : Math.abs(target - getVelocity()) / target * 100.0;
    }

    public double getDistanceToGoal() {
        Pose current = follower.getPose();
        return Math.hypot(
                goalPose.getX() - current.getX(),
                goalPose.getY() - current.getY()
        );
    }

    private PIDFCoefficients getCurrentPidf() {
        return pidfStrategy.getCalculatedPidf(getTargetVelocity(), battery.getVoltage(), getVelocity());
    }
}