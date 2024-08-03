package org.firstinspires.ftc.teamcode.util.control;

public class EricPid {
    public double kP;
    public double kI;
    public double kD;

    double integralSumLimit = 0;
    public double target;
    public double lastTarget;

    double error= 0;
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;
    double loopTime = 0;


    public EricPid(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        integralSumLimit = 0.25 / kI;
    }

    public void setTarget(double target){
        this.target = target;
    }

    public double update(double currentPos) {
        if(lastTarget!=target) {
            integral = 0;
        }
        error = target - currentPos;

        if(counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();

        loopTime = (currentTime - lastLoopTime) / 1000000000.0;
        lastLoopTime = currentTime;

        double proportional = error * kP;
        integral += error * kI * loopTime;
        double derivative = (error - lastError) * kD / loopTime;
        if (integral > integralSumLimit) {
            integral = integralSumLimit;
        }
        if (integral < -integralSumLimit) {
            integral = -integralSumLimit;
        }
        lastError = error;
        counter++;

        lastTarget = target;
        return proportional + integral + derivative;
    }

    public void updatePid(double p,double i,double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }
}