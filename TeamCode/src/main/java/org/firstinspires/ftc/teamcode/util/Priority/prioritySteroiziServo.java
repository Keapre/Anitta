package org.firstinspires.ftc.teamcode.util.Priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;

public class prioritySteroiziServo extends PriorityDevice {
    public enum ServoType {
        AXON_MINI(0.1784612002049795, 5.6403953024772129),
        AXON_MAX(0.1775562245447108, 6.5830247235911042);

        public double positionPerRadian;
        public double speed;

        ServoType(double positionPerRadian,double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public ServoType type;
    public CachingServo[] servo;

    public String name;
    public double currentPos;
    public double min,max;
    public double minAng,maxAng;

    public double power = 0;
    public double targetAngle = 0,currentAngle = 0;
    private boolean reachedIntermediate = false;
    private double  lastUpdatedTargetAngle = 0, currentIntermediateTargetAngle = 0;

    private long lastUpdateTime = System.nanoTime();
    private long lastLoopTime = System.nanoTime();
    protected boolean reversed = false;

    public prioritySteroiziServo(CachingServo servo, String name, double basePriority,ServoType type,double currentPos,double lastPos,boolean reversed) {
        this(new CachingServo[] {servo},name,basePriority,type,currentPos,lastPos,reversed);
    }


    public prioritySteroiziServo(CachingServo[] servo, String name, double basePriority,ServoType type,double currentPos,double lastPos,boolean reversed) {
        super(basePriority, name);
        this.servo = servo;

        this.type = type;
        this.currentPos = currentPos;
        this.reversed = reversed;
        if(reversed) {
            this.type.positionPerRadian *=-1;
        }
        min = Math.min(min,max);
        max = Math.max(min,max);

        minAng = convertPosToAngle(min);
        maxAng = convertPosToAngle(max);
        servo[0].setPosition(currentPos);
        callLengthMillis = 1.0;
    }
    public double convertPosToAngle(double pos){
        pos -= currentPos;
        pos /= type.positionPerRadian;
        return pos;
    }
    public double convertAngleToPos(double ang){
        ang *= type.positionPerRadian;
        ang += currentPos;
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
    public double getPos() {
        return servo[0].getPosition();
    }
    public void setPosition(double pos) {
        this.currentPos = pos;
    }    public boolean inPosition(){
        return Math.abs(targetAngle-currentAngle) < Math.toRadians(0.01);
    }
    public void setCurrentAngle(double currentAngle) {
        this.currentAngle = currentAngle;
    }
    @Override
    public double getPriority(double timeRemaining) {
        if (isUpdated) {
            return 0;
        }

        updateVal();

        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }

        double priority = ((reachedIntermediate && currentIntermediateTargetAngle != targetAngle) ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0;

        if (priority == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        return priority;
    }
    public double getTargetPosition () {
        return convertAngleToPos(currentIntermediateTargetAngle);
    }
    public double getTargetAngle() {
        return currentIntermediateTargetAngle;
    }
    @Override
    public void update() {
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

        for(CachingServo servos : servo) {
            servos.setPosition(convertAngleToPos(currentIntermediateTargetAngle));
        }
        lastUpdateTime = currentTime;
    }
}
