package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;
import org.firstinspires.ftc.teamcode.util.Utils;


@Config
public class Outtake2 {

    public enum FourBarState {
        IDLE,
        CHANGING_ROTATE,
        TRANSFER_IDLE,
        INTAKE_POSITION,
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
    public FourBarState lastImportant = FourBarState.TRANSFER_IDLE;

    public ClawState clawState = ClawState.OPEN;
    public final PriorityServo clawLeft, clawRight;
    public final PriorityServo rotateServo;

    public final PriorityServo outtakebar;

    public  boolean dropLeftPixel = false;
    public  boolean dropRightPixel = false;
    public final PriorityServo servoArmLeft, servoArmRight;

    Robot3 robot;
    public double defaultRotatePos = 0.56;
    public double currentRotatePos = defaultRotatePos;

    public double defaultOuttakeBarPos = 0.24;
    public double defaultArmLeft = 0.25;
    public double defaultArmRight = 0.75;
    public double intakeArmRight = 0.85;
    public double intakeArmLeft = 0.15;
    public double intakeTilt = 0.24;

    public double clawLeftOpen = 0.01;
    public double clawRightOpen = 0.02;
    public double clawLeftClosed = 0.22;
    public double clawRightClosed = 0.16;


    public double scoringOuttakeBarPose = 0.92;
    public double scoringArmLeft = 0.55;
    public double scoringArmRight = 0.45;
    public double deltaRotateValue = 0.1;


    public Outtake2(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Robot3 robot) {
        clawLeft = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "clawLeft")),
                "clawLeft", 0.2, PriorityServo.ServoType.AXON_MINI, clawLeftClosed, clawLeftClosed, false);
        clawRight = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "clawRight")),
                "clawRight", 0.2, PriorityServo.ServoType.AXON_MINI, clawRightClosed, clawRightClosed, false);

        rotateServo = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "rotateouttake")),
                "rotateServo", 0.3, PriorityServo.ServoType.AXON_MINI, defaultRotatePos, defaultRotatePos, false);

        outtakebar = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "tiltouttake")),
                "outtakebar", 0.35, PriorityServo.ServoType.AXON_MINI, defaultOuttakeBarPos, defaultOuttakeBarPos, false);

        servoArmLeft = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "leftservo")),
                "servoArmLeft", 0.4, PriorityServo.ServoType.AXON_MINI, defaultArmLeft, defaultArmLeft, false);

        servoArmRight = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "rightservo")),
                "servoArmRight", 0.4, PriorityServo.ServoType.AXON_MINI, defaultArmRight, defaultArmRight, false);

        currentState = FourBarState.TRANSFER_IDLE;
        clawState = ClawState.CLOSE;
        this.robot = robot;
        hardwareQueue.addDevice(clawLeft);
        hardwareQueue.addDevice(clawRight);
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
                clawLeft.setPosition(clawLeftClosed);
                releaseTime = 250;
            } else {
                clawLeft.setPosition(clawRightClosed);
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
                clawRight.setPosition(clawRightClosed);
                releaseTime = 250;
            } else {
                clawRight.setPosition(clawRightClosed);
                releaseTime = 125;
            }
            Globals.NUM_PIXELS--;
            busy = true;
            timer = System.currentTimeMillis();
        }
    }

    public void pickupLeft() {
        clawLeft.setPosition(clawLeftOpen);
    }

    public void pickupRight() {
        clawRight.setPosition(clawRightOpen);
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

    public void clawOpenFunc() {
        clawLeft.setPosition(clawLeftOpen);
        clawRight.setPosition(clawRightOpen);
    }

    public void clawCloseFunc() {
        clawLeft.setPosition(clawLeftClosed);
        clawRight.setPosition(clawRightClosed);
    }

    public Action clawOpen() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(clawLeft,clawLeftOpen),
                new ActionUtil.ServoPositionAction(clawRight,clawRightOpen)
        );
    }
    public Action clawClosed() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(clawLeft,clawLeftClosed),
                new ActionUtil.ServoPositionAction(clawRight,clawRightClosed)
        );
    }

    public Action scoringPos() {
        setOuttakeState(FourBarState.OUTTAKE_POSITION);
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(servoArmLeft,scoringArmLeft),
                new ActionUtil.ServoPositionAction(servoArmRight,scoringArmRight),
                new ActionUtil.ServoPositionAction(outtakebar,scoringOuttakeBarPose)
        );
    }

    public Action transferPos() {
        setOuttakeState(FourBarState.TRANSFER_IDLE);
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(servoArmRight,defaultArmRight),
                new ActionUtil.ServoPositionAction(servoArmLeft,defaultArmLeft),
                new ActionUtil.ServoPositionAction(outtakebar,defaultOuttakeBarPos)
        );
    }
    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case OUTTAKE_POSITION:
//                robot.slides.checkForIntake();
                servoArmLeft.setPosition(scoringArmLeft);
                servoArmRight.setPosition(scoringArmRight);
                outtakebar.setPosition(scoringOuttakeBarPose);
                break;
            case TRANSFER_IDLE:
//                robot.slides.checkForIntake();
                servoArmLeft.setPosition(defaultArmLeft);
                servoArmRight.setPosition(defaultArmRight);
                outtakebar.setPosition(defaultOuttakeBarPos);
                break;
            case INTAKE_POSITION:
                servoArmLeft.setPosition(intakeArmLeft);
                servoArmRight.setPosition(intakeArmRight);
                outtakebar.setPosition(intakeTilt);
                break;
        }
        switch (clawState) {
            case OPEN:
                clawLeft.setPosition(clawLeftOpen);
                clawRight.setPosition(clawRightOpen);
                break;
            case CLOSE:
                clawLeft.setPosition(clawLeftClosed);
                clawRight.setPosition(clawRightClosed);
                break;
            case LEFT_CLOSE:
                clawLeft.setPosition(clawLeftClosed);
                break;
            case RIGHT_CLOSE:
                clawRight.setPosition(clawRightClosed);
                break;
            case LEFT_OPEN:
                clawLeft.setPosition(clawLeftOpen);
                break;
            case RIGHT_OPEN:
                clawRight.setPosition(clawRightOpen);
                break;
        }
        rotateServo.setPosition(currentRotatePos);
    }
    private class ChangeClawState implements Action {

        ClawState state;

        public ChangeClawState(ClawState state) {
            this.state = state;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawState = state;
            return false;
        }
    }
    public Action changeClawState(ClawState state) {
        return new ChangeClawState(state);
    }
    private class ChangeArmState implements Action {

        FourBarState state;
        public ChangeArmState(FourBarState state) {
            this.state = state;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentState = state;
            return false;
        }
    }
    public Action changeArmState(FourBarState state) {
        return new ChangeArmState(state);
    }
}
