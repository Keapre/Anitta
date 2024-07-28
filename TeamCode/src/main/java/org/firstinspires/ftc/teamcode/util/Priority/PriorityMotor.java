package org.firstinspires.ftc.teamcode.util.Priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.Utils;

public class PriorityMotor extends PriorityDevice{
    double lastPower = 0;
    public double power = 0;
    public CachingDcMotorEx[] motor;
    public String name;

    private double minPowerToOvercomeFriction = 0.0;

    public PriorityMotor(CachingDcMotorEx motor,String name,double basePriority) {
        this(new CachingDcMotorEx[] {motor},name,basePriority);
    }


    public PriorityMotor(CachingDcMotorEx[] motor, String name, double basePriority) {
        super(basePriority, name);
        this.motor = motor;
        this.name = name;



        callLengthMillis = 1.6;

    }

    public void setMinimumPowerToOvercomeFriction (double value) {
        minPowerToOvercomeFriction = value;
    }

    public void setTargetPower(double power) {
        if (power == 0) {
            this.power = 0;
            return;
        }
        power = Utils.minMaxClip(power, -1.0, 1.0);
        this.power = power;
    }
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setZeroPowerBehavior(behavior);
        }
    }
    public void setPowerForced(double power) {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power);
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
    public double getVelocity(int i) {
        return motor[i].getVelocity();
    }
    public double getPower() {
        return power;
    }
    @Override
    protected double getPriority(double timeRemaining) {
        if (power-lastPower == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        if (timeRemaining * 1000.0 <= callLengthMillis * (motor.length-1) + callLengthMillis/2.0) {
            return 0;
        }

        return basePriority;
    }

    @Override
    protected void update() {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power);
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
}
