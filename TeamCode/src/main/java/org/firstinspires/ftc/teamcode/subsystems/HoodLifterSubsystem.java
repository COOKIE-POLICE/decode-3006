package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class HoodLifterSubsystem extends SubsystemBase {
    private final CRServo leftHoodServo;
    private final CRServo rightHoodServo;

    public HoodLifterSubsystem(HardwareMap hardwareMap) {
        leftHoodServo = hardwareMap.get(CRServo.class, Preferences.LEFT_HOOD_SERVO);
        rightHoodServo = hardwareMap.get(CRServo.class, Preferences.RIGHT_HOOD_SERVO);
    }

    public void setUp() {
        leftHoodServo.setPower(-1.0);
        rightHoodServo.setPower(-1.0);
    }

    public void setDown() {

        leftHoodServo.setPower(1.0);
        rightHoodServo.setPower(1.0);
    }
}
