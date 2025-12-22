package org.firstinspires.ftc.teamcode.modules.blinkin;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class BlinkinExtension {
    private RevBlinkinLedDriver driver;
    public BlinkinExtension(HardwareMap hardwareMap, String name) {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, name);
    }
    public BlinkinExtension setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        driver.setPattern(pattern);
        return this;
    }
}
