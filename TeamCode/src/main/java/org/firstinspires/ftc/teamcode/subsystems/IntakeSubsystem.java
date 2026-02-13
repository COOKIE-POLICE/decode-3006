package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;

    private IntakeSubsystem(Builder builder) {
        intakeMotor = builder.hardwareMap.get(DcMotor.class, builder.motorName);
        intakeMotor.setDirection(builder.direction);
    }

    public static final class Builder {
        private HardwareMap hardwareMap;
        private String motorName = Preferences.INTAKE_MOTOR;
        private DcMotorSimple.Direction direction = Preferences.Intake.DIRECTION;

        public Builder(HardwareMap hardwareMap) {
            hardwareMap = hardwareMap;
        }

        public Builder motorName(String motorName) {
            motorName = motorName;
            return this;
        }

        public Builder direction(DcMotorSimple.Direction direction) {
            direction = direction;
            return this;
        }

        public IntakeSubsystem build() {
            return new IntakeSubsystem(this);
        }
    }

    public void startIntaking() {
        intakeMotor.setPower(1.0);
    }

    public void startEjecting() {
        intakeMotor.setPower(-1.0);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}