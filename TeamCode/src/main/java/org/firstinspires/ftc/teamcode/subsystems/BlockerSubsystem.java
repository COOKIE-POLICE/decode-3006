package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class BlockerSubsystem extends SubsystemBase {

    private final Servo blockerServo;
    private final double blockPosition;
    private final double admitPosition;
    private boolean blocking = true;

    private BlockerSubsystem(Builder builder) {
        this.blockerServo = builder.hardwareMap.get(Servo.class, builder.servoName);
        this.blockPosition = builder.blockPosition;
        this.admitPosition = builder.admitPosition;
    }

    public static final class Builder {
        private final HardwareMap hardwareMap;
        private String servoName = "blockerServo";
        private double blockPosition = 1.0;
        private double admitPosition = 0.570;

        public Builder(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        public Builder servoName(String servoName) {
            this.servoName = servoName;
            return this;
        }

        public Builder blockPosition(double blockPosition) {
            this.blockPosition = blockPosition;
            return this;
        }

        public Builder admitPosition(double admitPosition) {
            this.admitPosition = admitPosition;
            return this;
        }

        public BlockerSubsystem build() {
            return new BlockerSubsystem(this);
        }
    }

    public void block() {
        blockerServo.setPosition(blockPosition);
        blocking = true;
    }

    public void admit() {
        blockerServo.setPosition(admitPosition);
        blocking = false;
    }

    public boolean isBlocking() { return blocking; }
}