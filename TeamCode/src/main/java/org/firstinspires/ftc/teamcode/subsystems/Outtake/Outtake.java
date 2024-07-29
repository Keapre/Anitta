package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;


@Config
public class Outtake {

    public enum outtakeState {
        IDLE,
        CHANGING_ROTATE,
        TRANSFER_INTAKE,

        DROP_LEFT,
        DROP_RIGHT,
        OUTTAKE_POSITION,
        DROP
    }

    boolean busy = false;

    public static double releaseTime = 250;
    public static double timer = 0;
    outtakeState currentState = outtakeState.IDLE;
    public final PriorityServo clawLeft, clawRight;
    public final PriorityServo rotateServo;

    public final PriorityServo outtakebar;

    public static boolean dropLeftPixel = false;
    public static boolean dropRightPixel = false;
    public final PriorityServo servoArmLeft, servoArmRight;

    public final double clawOpen = 0.7;
    public final double clawClosed = 0.3;

    public double defaultRotatePos = 0.3;

    public double currentRotatePos = defaultRotatePos;

    public double defaultOuttakeBarPos = 0.5;

    public double defaultArmLeft = 0.2;

    public double defaultArmRight = 0.8;

    public double scoringOuttakeBarPose = 0.7;
    public double scoringArmLeft = 0.7;

    public double deltaRotateValue = 0.1;
    public double scoringArmRight = 0.3;

    public Outtake(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        clawLeft = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "clawLeft")),
                "clawLeft", 0.2, PriorityServo.ServoType.AXON_MINI, clawClosed, clawClosed, false);
        clawRight = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "clawRight")),
                "clawRight", 0.2, PriorityServo.ServoType.AXON_MINI, clawClosed, clawClosed, false);

        rotateServo = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "rotateServo")),
                "rotateServo", 0.3, PriorityServo.ServoType.AXON_MINI, defaultRotatePos, defaultRotatePos, false);

        outtakebar = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "outtakebar")),
                "outtakebar", 0.35, PriorityServo.ServoType.AXON_MINI, defaultOuttakeBarPos, defaultOuttakeBarPos, false);

        servoArmLeft = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "servoArmLeft")),
                "servoArmLeft", 0.4, PriorityServo.ServoType.AXON_MINI, defaultArmLeft, defaultArmLeft, false);

        servoArmRight = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "servoArmRight")),
                "servoArmRight", 0.4, PriorityServo.ServoType.AXON_MINI, defaultArmRight, defaultArmRight, false);

        currentState = outtakeState.IDLE;
        hardwareQueue.addDevice(clawLeft);
        hardwareQueue.addDevice(rotateServo);
        hardwareQueue.addDevice(outtakebar);
        hardwareQueue.addDevice(servoArmLeft);
        hardwareQueue.addDevice(servoArmRight);
    }

    public boolean isBusy() {
        return busy;
    }

    public boolean readyToRetract() {
        return Globals.NUM_PIXELS <= 0 && !busy;
    }

    public void setOuttakeState(outtakeState newState) {
        currentState = newState;
    }

    public void addRotate() {
        currentRotatePos += deltaRotateValue;
    }

    public void subRotate() {
        currentRotatePos -= deltaRotateValue;
    }

    boolean releasingTwo = false;

    public void releaseOne() {
        if (!busy) {
            if (Globals.NUM_PIXELS == 2) {
                clawLeft.setPosition(clawClosed);
                releaseTime = 250;
            } else {
                clawLeft.setPosition(clawClosed);
                releaseTime = 125;
            }
            Globals.NUM_PIXELS--;
            busy = true;
            timer = System.currentTimeMillis();
        }
    }

    public void releaseOne2() {
        if (!busy) {
            if (Globals.NUM_PIXELS == 2) {
                clawRight.setPosition(clawClosed);
                releaseTime = 250;
            } else {
                clawRight.setPosition(clawClosed);
                releaseTime = 125;
            }
            Globals.NUM_PIXELS--;
            busy = true;
            timer = System.currentTimeMillis();
        }
    }

    public void releaseTwo() {

        releaseOne();
        releasingTwo = true;
    }

    public void updateRelease() {
        if (System.currentTimeMillis() - timer >= releaseTime) {
            busy = false;
            if (releasingTwo) {
                releaseOne2();
                releasingTwo = false;
            }
        }
    }

    public void setPositionClaw(double target) {
        clawLeft.setPosition(target);
        clawRight.setPosition(target);
    }

    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case CHANGING_ROTATE:
                rotateServo.setPosition(currentRotatePos);
                setOuttakeState(outtakeState.IDLE);
                break;
            case OUTTAKE_POSITION:
                setPositionClaw(clawClosed);
                servoArmLeft.setPosition(scoringArmLeft);
                servoArmRight.setPosition(scoringArmRight);
                outtakebar.setPosition(scoringOuttakeBarPose);
                setOuttakeState(outtakeState.DROP);
                break;
            case DROP_RIGHT:
                clawRight.setPosition(clawOpen);
                setOuttakeState(outtakeState.IDLE);
                break;
            case DROP:
                releaseTwo();
                setOuttakeState(outtakeState.TRANSFER_INTAKE);
            case DROP_LEFT:
                clawLeft.setPosition(clawOpen);
                setOuttakeState(outtakeState.IDLE);
                break;
            case TRANSFER_INTAKE:
                setPositionClaw(clawOpen);
                servoArmLeft.setPosition(defaultArmLeft);
                servoArmRight.setPosition(defaultArmRight);
                outtakebar.setPosition(defaultOuttakeBarPos);
                setPositionClaw(clawClosed);
                setOuttakeState(outtakeState.IDLE);
                break;
        }
    }

}
