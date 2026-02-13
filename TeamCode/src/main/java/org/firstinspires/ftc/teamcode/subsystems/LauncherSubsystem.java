package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.LauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.PreferencesLauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.InterpolatedTargetVelocityCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.PreferencesTargetVelocityCalculationStrategy;
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

    public LauncherSubsystem(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, Preferences.LAUNCHER_MOTOR);
        this.battery = hardwareMap.voltageSensor.iterator().next();
        this.follower = follower;
        this.goalPose = goalPose;
        this.targetVelocityStrategy = new InterpolatedTargetVelocityCalculationStrategy();
        this.pidfStrategy = new PreferencesLauncherPIDFCalculationStrategy();
        this.isLaunching = false;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients initialPidf = pidfStrategy.getCalculatedPidf(0, battery.getVoltage(), 0);
        this.tunedMotor = new TunedMotor(motor, initialPidf.p, initialPidf.i, initialPidf.d, initialPidf.f);
        this.tunedMotor.setTolerance(0.0);
        tunedMotor.setDirection(Preferences.Launcher.DIRECTION);
    }

    @Override
    public void periodic() {
        if (isLaunching) {
            double targetVelocity = getTargetVelocity();
            PIDFCoefficients pidf = getCurrentPIDF();

            tunedMotor.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
            tunedMotor.setTargetVelocity(targetVelocity);
            tunedMotor.update(getVelocity());
        }
    }

    public void startLaunching() {
        isLaunching = true;
    }

    public void stopLaunching() {
        tunedMotor.getMotor().setPower(0.0);
        isLaunching = false;
    }

    public boolean isLaunching() {
        return isLaunching;
    }

    public boolean isAtTargetVelocity() {
        return getPercentError() < VELOCITY_TOLERANCE;
    }

    public double getTargetVelocity() {
        return targetVelocityStrategy.getCalculatedTargetVelocity(getDistanceToGoal());
    }

    public double getVelocity() {
        return Math.abs(tunedMotor.getVelocity());
    }

    public double getPercentError() {
        double target = getTargetVelocity();
        if (target == 0.0) {
            return 0.0;
        }
        return Math.abs(target - getVelocity()) / target * 100.0;
    }

    public Pose getCurrentPose() {
        return follower.getPose();
    }

    public double getDistanceToGoal() {
        Pose current = follower.getPose();
        double deltaX = goalPose.getX() - current.getX();
        double deltaY = goalPose.getY() - current.getY();
        return Math.hypot(deltaX, deltaY);
    }

    private PIDFCoefficients getCurrentPIDF() {
        return pidfStrategy.getCalculatedPidf(getTargetVelocity(), battery.getVoltage(), getVelocity());
    }
}