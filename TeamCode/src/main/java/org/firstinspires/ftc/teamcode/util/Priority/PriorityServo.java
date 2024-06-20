package org.firstinspires.ftc.teamcode.util.Priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;

public class PriorityServo extends PriorityDevice {
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
    public double basePriority;

    public double currentPos = 0;
    public double lastPos = 0;
    private long lastLoopTime = System.nanoTime();
    protected boolean reversed = false;

    public PriorityServo(CachingServo servo, String name, double basePriority,ServoType type,double currentPos,double lastPos,boolean reversed) {
        this(new CachingServo[] {servo},name,basePriority,type,currentPos,lastPos,reversed);
    }


    public PriorityServo(CachingServo[] servo, String name, double basePriority,ServoType type,double currentPos,double lastPos,boolean reversed) {
        super(basePriority, name);
        this.servo = servo;

        this.type = type;
        this.currentPos = currentPos;
        this.lastPos = lastPos;
        this.reversed = reversed;
        if(reversed) {
            this.type.positionPerRadian *=-1;
        }
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

    public double getPos() {
        return servo[0].getPosition();
    }
    public void setPosition(double pos) {
        this.currentPos = pos;
    }
    @Override
    protected double getPriority(double timeRemaining) {
        if(currentPos - lastPos == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        return basePriority;
    }
    @Override
    protected void update() {

        for(Servo servos : servo) {
            servos.setPosition(currentPos);
        }
        lastUpdateTime = System.nanoTime();
        lastPos = currentPos;
    }
}
