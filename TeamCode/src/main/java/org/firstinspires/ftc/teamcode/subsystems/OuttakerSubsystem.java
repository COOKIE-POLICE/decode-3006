package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class OuttakerSubsystem extends SubsystemBase {
    private final Servo leftOuttakeServo;
    private final Servo rightOuttakeServo;

    public OuttakerSubsystem(HardwareMap hardwareMap) {
        leftOuttakeServo = hardwareMap.get(Servo.class, Preferences.LEFT_OUTTAKE_SERVO);
        rightOuttakeServo = hardwareMap.get(Servo.class, Preferences.RIGHT_OUTTAKE_SERVO);
        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);
    }
    public void push() {
        leftOuttakeServo.setPosition(1.0);
        rightOuttakeServo.setPosition(1.0);
    }
    public void hold() {
        leftOuttakeServo.setPosition(0.0);
        rightOuttakeServo.setPosition(0.0);
    }
}
