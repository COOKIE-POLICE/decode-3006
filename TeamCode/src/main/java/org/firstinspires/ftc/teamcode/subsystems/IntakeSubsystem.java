package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

/**
 * IntakeSubsystem controls only the intake motor.
 */
 public class IntakeSubsystem extends SubsystemBase {
     private final DcMotor intakeMotor;

    private enum IntakeState {
        STOPPED,
        INTAKING,
        EJECTING
    }

    private IntakeState currentState = IntakeState.STOPPED;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, Preferences.INTAKE_MOTOR);
        intakeMotor.setDirection(Preferences.Intake.DIRECTION);

    }

    public void startIntaking() {
        currentState = IntakeState.INTAKING;
        intakeMotor.setPower(1);
    }
    public void startEjecting() {
        currentState = IntakeState.EJECTING;
        intakeMotor.setPower(-1);
    }

    public void stop() {
        currentState = IntakeState.STOPPED;
        intakeMotor.setPower(0.0);
    }

    /**
     * Set the intake motor power directly (for manual control).
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Check if the intake is currently running.
     */
    public boolean isIntaking() {
        return currentState == IntakeState.INTAKING;
    }

    /**
     * Check if the intake is currently ejecting.
     */
    public boolean isEjecting() {
        return currentState == IntakeState.EJECTING;
    }

    /**
     * Check if the intake is stopped.
     */
    public boolean isStopped() {
        return currentState == IntakeState.STOPPED;
    }

    /**
     * Get the current intake state.
     */
    public String getState() {
        return currentState.toString();
    }
    
    /**
     * Get the last detected color sensor.
     * @return "LEFT", "RIGHT", or "NONE"
     */
    public String getLastDetectedSensor() { return "NONE"; }
    
    /**
     * Get the left color sensor color.
     */
    public int getLeftColor() { return 0; }
    
    /**
     * Get the right color sensor color.
     */
    public int getRightColor() { return 0; }
}
