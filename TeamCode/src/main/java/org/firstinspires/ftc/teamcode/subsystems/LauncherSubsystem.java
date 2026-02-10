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
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.SmartLauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.InterpolatedTargetVelocityCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.PreferencesTargetVelocityCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.TargetVelocityCalculationStrategy;

public class LauncherSubsystem extends SubsystemBase {
    private static final double VELOCITY_TOLERANCE = 1.0;

    private final DcMotorEx launcherMotor;
    private final VoltageSensor battery;
    private final Follower follower;
    private final Pose goalPose;
    private final TargetVelocityCalculationStrategy targetVelocityCalculationStrategy;
    private final LauncherPIDFCalculationStrategy launcherPidfCalculationStrategy;

    private boolean isLaunching;
    private PIDFCoefficients activePidfCoefficients;

    public LauncherSubsystem(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
        this.launcherMotor = hardwareMap.get(DcMotorEx.class, Preferences.LAUNCHER_MOTOR);
        this.battery = hardwareMap.voltageSensor.iterator().next();
        this.follower = follower;
        this.goalPose = goalPose;
        this.targetVelocityCalculationStrategy = new InterpolatedTargetVelocityCalculationStrategy();
        this.launcherPidfCalculationStrategy = new SmartLauncherPIDFCalculationStrategy();
        this.isLaunching = false;
        this.activePidfCoefficients = null;

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setDirection(Preferences.Launcher.DIRECTION);
    }
    @Override
    public void periodic() {
        activePidfCoefficients = getCurrentPIDFCoefficients();
        launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, activePidfCoefficients);
        if (isLaunching()) {
            double targetVelocity = getTargetVelocity();
            launcherMotor.setVelocity(targetVelocity);
        }
    }

    public void startLaunching() {
        double targetVelocity = getTargetVelocity();
        launcherMotor.setVelocity(targetVelocity);
        isLaunching = true;
    }

    public void stopLaunching() {
        launcherMotor.setPower(0.0);
        activePidfCoefficients = null;
        isLaunching = false;
    }

    public void setPower(double power) {
        launcherMotor.setPower(power);
    }

    public double getPower() {
        return launcherMotor.getPower();
    }

    public double getVelocityMagnitude() {
        return Math.abs(launcherMotor.getVelocity());
    }

    public boolean isLaunching() {
        return isLaunching;
    }

    public double getDistanceToGoal() {
        Pose currentPose = follower.getPose();
        double deltaX = goalPose.getX() - currentPose.getX();
        double deltaY = goalPose.getY() - currentPose.getY();
        return Math.hypot(deltaX, deltaY);
    }

    public double getTargetVelocity() {
        return targetVelocityCalculationStrategy.getCalculatedTargetVelocity(getDistanceToGoal());
    }

    public PIDFCoefficients getCurrentPIDFCoefficients() {
        return launcherPidfCalculationStrategy.getCalculatedPidf(getTargetVelocity(), getBatteryVoltage(), getVelocityMagnitude());
    }

    public double getP() {
        if (activePidfCoefficients != null) {
            return activePidfCoefficients.p;
        }
        return getCurrentPIDFCoefficients().p;
    }

    public double getI() {
        if (activePidfCoefficients != null) {
            return activePidfCoefficients.i;
        }
        return getCurrentPIDFCoefficients().i;
    }

    public double getD() {
        if (activePidfCoefficients != null) {
            return activePidfCoefficients.d;
        }
        return getCurrentPIDFCoefficients().d;
    }

    public double getF() {
        if (activePidfCoefficients != null) {
            return activePidfCoefficients.f;
        }
        return getCurrentPIDFCoefficients().f;
    }

    public double getBatteryVoltage() {
        return battery.getVoltage();
    }

    public boolean isAtTargetVelocity() {
        return getPercentError() < VELOCITY_TOLERANCE;
    }

    public double getVelocityError() {
        return Math.abs(getTargetVelocity() - getVelocityMagnitude());
    }

    public double getPercentError() {
        double targetVelocity = getTargetVelocity();
        if (targetVelocity == 0.0) {
            return 0.0;
        }
        return (getVelocityError() / targetVelocity) * 100.0;
    }

    public void setVelocity(double velocity) {
        launcherMotor.setVelocity(velocity);
    }

    public void setVelocityWithPIDF(double velocity, PIDFCoefficients pidfCoefficients) {
        launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcherMotor.setVelocity(velocity);
    }

    public DcMotorEx getMotor() {
        return launcherMotor;
    }

    public Pose getCurrentPose() {
        return follower.getPose();
    }

    public Pose getGoalPose() {
        return goalPose;
    }
}