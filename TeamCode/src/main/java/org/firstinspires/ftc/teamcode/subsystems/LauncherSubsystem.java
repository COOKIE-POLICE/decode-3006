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
import org.firstinspires.ftc.teamcode.modules.BangBangController;
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.LauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.launcherpidfcalculationstrategy.SmartLauncherPIDFCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.ConstantTargetVelocityCalculationStrategy;
import org.firstinspires.ftc.teamcode.strategies.targetvelocitycalculationstrategy.TargetVelocityCalculationStrategy;

public class LauncherSubsystem extends SubsystemBase {
    private static final double VELOCITY_TOLERANCE = 30.0;
    private static final double BANG_BANG_THRESHOLD = 200.0;
    private static final double BANG_BANG_POWER = 1.0;

    private final DcMotorEx launcherMotor;
    private final VoltageSensor battery;
    private final Follower follower;
    private final Pose goalPose;
    private final TargetVelocityCalculationStrategy targetVelocityCalculationStrategy;
    private final LauncherPIDFCalculationStrategy launcherPidfCalculationStrategy;
    private final BangBangController bangBangController;

    private boolean isLaunching;
    private double targetVelocity;
    private PIDFCoefficients activePidfCoefficients;
    private boolean usingBangBang;

    public LauncherSubsystem(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
        this.launcherMotor = hardwareMap.get(DcMotorEx.class, Preferences.LAUNCHER_MOTOR);
        this.battery = hardwareMap.voltageSensor.iterator().next();
        this.follower = follower;
        this.goalPose = goalPose;
        this.targetVelocityCalculationStrategy = new ConstantTargetVelocityCalculationStrategy();
        this.launcherPidfCalculationStrategy = new SmartLauncherPIDFCalculationStrategy();
        this.bangBangController = new BangBangController(BANG_BANG_THRESHOLD, BANG_BANG_POWER);
        this.isLaunching = false;
        this.targetVelocity = 0.0;
        this.activePidfCoefficients = null;
        this.usingBangBang = false;

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setDirection(Preferences.Launcher.DIRECTION);
    }

    public void startLaunching() {
        targetVelocity = calculateTargetVelocity();
        activePidfCoefficients = getCurrentPIDFCoefficients();
        isLaunching = true;
        usingBangBang = true;
    }

    public void stopLaunching() {
        launcherMotor.setPower(0.0);
        activePidfCoefficients = null;
        targetVelocity = 0.0;
        isLaunching = false;
        usingBangBang = false;
    }

    @Override
    public void periodic() {
        if (!isLaunching) {
            return;
        }

        double error = getVelocityError();

        if (bangBangController.shouldUseBangBang(error)) {
            if (!usingBangBang) {
                launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                usingBangBang = true;
            }
            double power = bangBangController.calculate(error);
            launcherMotor.setPower(power);
        }
        else {
            if (usingBangBang) {
                launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, activePidfCoefficients);
                launcherMotor.setVelocity(targetVelocity);
                usingBangBang = false;
            }
        }
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
        return targetVelocity;
    }

    private double calculateTargetVelocity() {
        return targetVelocityCalculationStrategy.getCalculatedTargetVelocity(getDistanceToGoal());
    }

    public PIDFCoefficients getCurrentPIDFCoefficients() {
        return launcherPidfCalculationStrategy.getCalculatedPidf(targetVelocity, getBatteryVoltage());
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
        return Math.abs(getVelocityError()) < VELOCITY_TOLERANCE;
    }

    public double getVelocityError() {
        return targetVelocity - getVelocityMagnitude();
    }

    public double getPercentError() {
        if (targetVelocity == 0.0) {
            return 0.0;
        }
        return (getVelocityError() / targetVelocity) * 100.0;
    }

    public void setVelocity(double velocity) {
        this.targetVelocity = velocity;
        launcherMotor.setVelocity(velocity);
    }

    public void setVelocityWithPIDF(double velocity, PIDFCoefficients pidfCoefficients) {
        this.targetVelocity = velocity;
        this.activePidfCoefficients = pidfCoefficients;
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

    public boolean isUsingBangBang() {
        return usingBangBang;
    }

    public BangBangController getBangBangController() {
        return bangBangController;
    }
}