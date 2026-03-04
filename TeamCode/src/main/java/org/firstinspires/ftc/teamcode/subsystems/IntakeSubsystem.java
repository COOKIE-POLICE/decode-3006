package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final DcMotor secondaryIntakeMotor;

    private IntakeSubsystem(Builder builder) {
        intakeMotor = builder.hardwareMap.get(DcMotor.class, builder.motorName);
        secondaryIntakeMotor = builder.hardwareMap.get(DcMotor.class, builder.secondaryMotorName);
        intakeMotor.setDirection(builder.direction);
    }

    public static final class Builder {
        private HardwareMap hardwareMap;
        private String motorName = Preferences.INTAKE_MOTOR;
        private String secondaryMotorName = "intakeMotor2";
        private DcMotorSimple.Direction direction = Preferences.Intake.DIRECTION;

        public Builder(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        public Builder motorName(String motorName) {
            this.motorName = motorName;
            return this;
        }

        public Builder direction(DcMotorSimple.Direction direction) {
            this.direction = direction;
            return this;
        }

        public IntakeSubsystem build() {
            return new IntakeSubsystem(this);
        }
    }

    public void startIntaking() {
        secondaryIntakeMotor.setPower(-1.0);
        intakeMotor.setPower(1.0);
    }

    public void startEjecting() {
        secondaryIntakeMotor.setPower(1.0);
        intakeMotor.setPower(-1.0);
    }

    public void stop() {
        secondaryIntakeMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
    }

    public void setPower(double power) {
        secondaryIntakeMotor.setPower(-power);
        intakeMotor.setPower(power);
    }
}