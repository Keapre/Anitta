package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;
import org.firstinspires.ftc.teamcode.util.Utils;


@Config
public class Outtake {

    public enum FourBarState {
        IDLE,
        CHANGING_ROTATE,
        TRANSFER_INTAKE,
        OUTTAKE_POSITION,
    }

    public enum ClawState {
        OPEN,
        CLOSE,

        LEFT_CLOSE,
        RIGHT_CLOSE,

        LEFT_OPEN,
        RIGHT_OPEN
    }

    boolean busy = false;

    public static double releaseTime = 250;
    public static double timer = 0;
    public FourBarState currentState = FourBarState.IDLE;
    public FourBarState lastImportant = FourBarState.TRANSFER_INTAKE;

    public ClawState clawState = ClawState.OPEN;
    public final PriorityServo clawLeft, clawRight;
    public final PriorityServo rotateServo;

    public final PriorityServo outtakebar;

    public  boolean dropLeftPixel = false;
    public  boolean dropRightPixel = false;
    public final PriorityServo servoArmLeft, servoArmRight;

    Robot2 robot;
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

    public Outtake(HardwareMap hardwareMap, HardwareQueue hardwareQueue,Robot2 robot) {
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

        currentState = FourBarState.TRANSFER_INTAKE;
        clawState = ClawState.CLOSE;
        this.robot = robot;
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

    public void setOuttakeState(FourBarState newState) {
        if(currentState!=newState)
            currentState = newState;
    }
    public void setClawState(ClawState newState) {

        if(clawState!=newState) clawState = newState;
    }
    public void addRotate() {

        currentRotatePos += deltaRotateValue;
        Utils.minMaxClip(currentRotatePos, 0, 1);

    }

    public void subRotate() {
        currentRotatePos -= deltaRotateValue;
        Utils.minMaxClip(currentRotatePos, 0, 1);
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

    public void pickupLeft() {
        clawLeft.setPosition(clawOpen);
    }

    public void pickupRight() {
        clawRight.setPosition(clawOpen);
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
            case OUTTAKE_POSITION:
                robot.slides.checkForIntake();
                setPositionClaw(clawClosed);
                setClawState(ClawState.CLOSE);
                servoArmLeft.setPosition(scoringArmLeft);
                servoArmRight.setPosition(scoringArmRight);
                outtakebar.setPosition(scoringOuttakeBarPose);
                lastImportant = FourBarState.OUTTAKE_POSITION;
                setOuttakeState(FourBarState.IDLE);
                break;
            case TRANSFER_INTAKE:
                robot.slides.checkForIntake();
                servoArmLeft.setPosition(defaultArmLeft);
                servoArmRight.setPosition(defaultArmRight);
                outtakebar.setPosition(defaultOuttakeBarPos);
                lastImportant = FourBarState.TRANSFER_INTAKE;
                setPositionClaw(clawOpen);
                setClawState(ClawState.OPEN);
                setOuttakeState(FourBarState.IDLE);
                break;
        }
        switch (clawState) {
            case OPEN:
                releaseTwo();
                break;
            case CLOSE:
                clawLeft.setPosition(clawClosed);
                clawRight.setPosition(clawClosed);
                break;
            case LEFT_CLOSE:
                clawLeft.setPosition(clawClosed);
                break;
            case RIGHT_CLOSE:
                clawRight.setPosition(clawClosed);
                break;
            case LEFT_OPEN:
                clawLeft.setPosition(clawOpen);
                break;
            case RIGHT_OPEN:
                clawRight.setPosition(clawOpen);
                break;
        }
        rotateServo.setPosition(currentRotatePos);
    }

}
