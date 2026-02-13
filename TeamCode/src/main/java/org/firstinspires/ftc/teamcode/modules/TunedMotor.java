package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class TunedMotor {
    private final DcMotorEx motor;
    private final PIDFController controller;
    private double targetVelocity;

    public TunedMotor(DcMotorEx motor, double p, double i, double d, double f) {
        this.motor = motor;
        this.controller = new PIDFController(p, i, d, f);
        this.targetVelocity = 0.0;
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }
    public double getVelocity() {
        return this.motor.getVelocity();
    }

    public void setPIDF(double p, double i, double d, double f) {
        controller.setPIDF(p, i, d, f);
    }

    public void setTolerance(double positionTolerance) {
        controller.setTolerance(positionTolerance);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void update(double currentVelocity) {
        double absoluteTarget = Math.abs(targetVelocity);
        double absoluteCurrent = Math.abs(currentVelocity);
        double direction = Math.signum(targetVelocity);

        double power = controller.calculate(absoluteCurrent, absoluteTarget);
        motor.setPower(power * direction);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public boolean atSetPoint() {
        return controller.atSetPoint();
    }
}