package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoTypeShi {
    public double basePos;
    public double min,max;
    public double minAng,maxAng;

    public double power = 0;
    public double targetAngle = 0,currentAngle = 0;
    private boolean reachedIntermediate = false;
    private double  lastUpdatedTargetAngle = 0, currentIntermediateTargetAngle = 0;

    private long lastUpdateTime = System.nanoTime();
    private long lastLoopTime = System.nanoTime();

    public enum servoType {
        AXON_MINI(0.1784612002049795, 5.6403953024772129),
        AXON_MAX(0.1775562245447108, 6.5830247235911042);

        public double positionPerRadian;
        public double speed;

        servoType(double positionPerRadian,double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public servoType type;
    public Servo servo;



    public ServoTypeShi(Servo servo, servoType type,double basePos,double min,double max) {
        this.servo = servo;
        this.type = type;

        this.basePos = basePos;

        min = Math.min(min,max);
        max = Math.max(min,max);

        minAng = convertPosToAngle(min);
        maxAng = convertPosToAngle(max);

    }

    public double convertPosToAngle(double pos){
        pos -= basePos;
        pos /= type.positionPerRadian;
        return pos;
    }
    public double convertAngleToPos(double ang){
        ang *= type.positionPerRadian;
        ang += basePos;
        return ang;
    }


    public void setTargetAngle(double targetAngle, double power){
        this.power = power;
        this.targetAngle = Math.max(Math.min(targetAngle,maxAng),minAng);
    }


    public void updateVal() {
        long currentTime = System.currentTimeMillis();
        double looptime = ((double)(currentTime - lastLoopTime))/1000;
        double error = currentIntermediateTargetAngle - currentAngle;
        double newAngle = looptime * type.speed * power * Math.signum(error);
        reachedIntermediate = (Math.abs(newAngle) > Math.abs(error));

        if(reachedIntermediate) {
            newAngle = error;
        }

        currentAngle += newAngle;
        lastLoopTime = currentTime;
    }

    public void updatePosition() {
        long currentTime = System.nanoTime();
        double looptime = ((double)(currentTime - lastUpdateTime))/1000000000;
        lastUpdatedTargetAngle = targetAngle;
        double error = lastUpdatedTargetAngle - currentAngle;
        double newAngle = looptime * type.speed * power * Math.signum(error);

        if(Math.abs(newAngle) > Math.abs(error)) {
            newAngle = error;
        }

        currentIntermediateTargetAngle+= newAngle;

        if(power == 1) {
            currentIntermediateTargetAngle = targetAngle;
        }

        servo.setPosition(convertAngleToPos(currentIntermediateTargetAngle));
        lastUpdateTime = currentTime;
    }
}
