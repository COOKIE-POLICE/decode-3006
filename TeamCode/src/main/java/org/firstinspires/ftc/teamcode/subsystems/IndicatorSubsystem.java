package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IndicatorSubsystem extends SubsystemBase {

    private static final double GREEN = 0.500;
    private static final double OFF = 0.0;

    private final Servo indicator;

    private IndicatorSubsystem(Builder builder) {
        this.indicator = builder.hardwareMap.get(Servo.class, builder.servoName);
    }

    public static final class Builder {
        private final HardwareMap hardwareMap;
        private String servoName = "indicator";

        public Builder(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        public Builder servoName(String servoName) {
            this.servoName = servoName;
            return this;
        }

        public IndicatorSubsystem build() {
            return new IndicatorSubsystem(this);
        }
    }

    public void turnGreen() { indicator.setPosition(GREEN); }
    public void turnOff() { indicator.setPosition(OFF); }
}