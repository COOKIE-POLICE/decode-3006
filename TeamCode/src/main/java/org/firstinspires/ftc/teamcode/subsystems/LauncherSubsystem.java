package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class LauncherSubsystem extends SubsystemBase {
    private final DcMotorEx launcherMotor;

    private boolean isLaunching = false;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, Preferences.LAUNCHER_MOTOR);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setDirection(Preferences.Launcher.DIRECTION);
        PIDFCoefficients pidfCoefficients = Preferences.Launcher.PIDF_COEFFICIENTS;
        launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void startLaunching() {
        isLaunching = true;
        launcherMotor.setVelocity(Preferences.Launcher.LAUNCH_VELOCITY);
    }

    public void stopLaunching() {
        isLaunching = false;
        launcherMotor.setVelocity(0.0);
    }

    public boolean isLaunching() {
        return isLaunching;
    }
    public double getVelocity() {
        return launcherMotor.getVelocity();
    }
}