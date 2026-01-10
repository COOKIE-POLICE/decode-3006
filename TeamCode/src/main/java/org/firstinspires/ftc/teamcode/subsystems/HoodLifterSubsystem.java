package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class HoodLifterSubsystem extends SubsystemBase {
    private final Servo leftHoodServo;
    private final Servo rightHoodServo;

    public HoodLifterSubsystem(HardwareMap hardwareMap) {
        leftHoodServo = hardwareMap.get(Servo.class, Preferences.LEFT_HOOD_SERVO);
        rightHoodServo = hardwareMap.get(Servo.class, Preferences.RIGHT_HOOD_SERVO);

        if (Preferences.Hood.INVERT_SERVO) {
            leftHoodServo.setDirection(Servo.Direction.REVERSE);
            rightHoodServo.setDirection(Servo.Direction.REVERSE);
        }
    }

    public void setPosition(double position) {
        leftHoodServo.setPosition(position);
        rightHoodServo.setPosition(position);
    }

    public void setUp() {
        setPosition(Preferences.Hood.POSITION_UP);
    }

    public void setDown() {
        setPosition(Preferences.Hood.POSITION_DOWN);
    }

    public double getPosition() {
        return leftHoodServo.getPosition();
    }
}
