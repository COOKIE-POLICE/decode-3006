package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class BlockerSubsystem extends SubsystemBase {
    private final Servo blockerServo;
    private boolean blocking = true;

    public BlockerSubsystem(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, Preferences.BLOCKER_SERVO);
    }
    public void block() {

        blockerServo.setPosition(Preferences.BlockerServo.BLOCK_POSITION);
        blocking = true;
    }
    public void admit() {
        blockerServo.setPosition(Preferences.BlockerServo.ADMIT_POSITION);
        blocking = false;
    }
    public boolean getBlocking() {
        return blocking;
    }
}