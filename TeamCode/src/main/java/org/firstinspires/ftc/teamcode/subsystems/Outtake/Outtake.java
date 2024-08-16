package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import android.provider.Settings;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServoImplEx;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Utils;


@Config
public class Outtake implements Subsystem {

    public enum FourBarState {
        IDLE,
        CHANGING_ROTATE,
        TRANSFER_IDLE,
        INTAKE_POSITION,

        INTAKE_OUTTAKE,
        TRANSFER_AUTO,
        OUTTAKE_POSITION,
    }

    public enum ROTATESTATE {
        //previous, current, next

        LEFT90(left90){
            @Override
            public ROTATESTATE previous() {
                return this;
            }
        },
        LEFT45(left45),
        DEFAULT(defaultRotate),
        RIGHT45(right45),


        RIGHT90(right90){
            @Override
            public ROTATESTATE next() {
                return this;
            }
        };

        public final double pos;

        ROTATESTATE(double pos) {
            this.pos = pos;
        }

        public ROTATESTATE previous() {
            return values()[ordinal() - 1];
        }

        public ROTATESTATE next() {
            return values()[ordinal() + 1];
        }
    }

    public enum ClawState {
        OPEN,
        CLOSE,

        LEFT_CLOSE,
        RIGHT_CLOSE,

        LEFT_OPEN,
        RIGHT_OPEN
    }

    public static double[] rotateValues = new double[]{0.85, 0.7, 0.56, 0.41,0.26};

    public static double defaultRotate = 0.56;
    public static double left45 = 0.7;
    public static double right45 = 0.41;
    public static double left90 = 0.85;
    public static double right90 = 0.26;
    boolean busy = false;

    public static double releaseTime = 250;
    public static double timer = 0;
    public FourBarState currentState = FourBarState.IDLE;

    public long lastintakeAuto = -1;
    public FourBarState lastImportant = FourBarState.OUTTAKE_POSITION;

    public ClawState clawState = ClawState.OPEN;
    public final CachingServoImplEx clawLeft, clawRight;
    public final CachingServoImplEx rotateServo;

    public final CachingServoImplEx outtakebar;

    public  boolean dropLeftPixel = false;
    public  boolean dropRightPixel = false;
    public final CachingServoImplEx servoArmLeft, servoArmRight;

    Robot robot;
    public double defaultRotatePos = 0.56;
    public double currentRotatePos = defaultRotatePos;


    public static int rotateIndex = 2;
    public static double defaultOuttakeBarPos = 0.14;
    public static double defaultArmLeft = 0.3;
    public static double defaultArmRight = 0.7;
    long lastTransferTime = -1;
    public static double intakeArmRight = 0.85;
    public static double intakeArmLeft = 0.15;
    public static double intakeTilt = 0.14;

    public ROTATESTATE rotateState = ROTATESTATE.DEFAULT;
    public static double preIntakeArmLeft = 0.19;

    public static double capacTransferTimer = -1;
    public static double prePreIntakeArmRight = 0.81;
    public static double preIntakeTilt = 0.20;
    public static double clawLeftClosed = 0.4;
    public static double clawRightClosed = 0.15;
    public static double clawLeftOpen = 0.6;
    public static double clawRightOpen = 0;


    public static double scoringOuttakeBarPose = 0.80;
    public static double scoringArmLeft = 0.60;
    public static double scoringArmRight = 0.40;
    public double deltaRotateValue = 0.1;


    public Outtake(HardwareMap hardwareMap, Robot robot) {
        clawLeft = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "clawLeft"));
        clawRight = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "clawRight"));
        rotateServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "rotateOuttake"));

        outtakebar = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "tiltOuttake"));
        servoArmLeft = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "leftServo"));

        servoArmRight = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "rightServo"));

        currentState = FourBarState.TRANSFER_IDLE;
        clawState = ClawState.CLOSE;
        rotateState = ROTATESTATE.DEFAULT;
        this.robot = robot;
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
        rotateState = rotateState.next();
    }

    public void subRotate() {

        rotateState = rotateState.previous();
    }
    public void hangMode(){
        servoArmRight.setPwmDisable();
        servoArmLeft.setPwmDisable();
        clawLeft.setPwmDisable();
        clawRight.setPwmDisable();
        rotateServo.setPwmDisable();
        outtakebar.setPwmDisable();
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

    public int getRotateIndex() {
        return rotateIndex;
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

//    public Action clawOpen() {
//        return new ParallelAction(
//                new ActionUtil.ServoPositionAction(clawLeft,clawLeftOpen),
//                new ActionUtil.ServoPositionAction(clawRight,clawRightOpen)
//        );
//    }
//    public Action clawClosed() {
//        return new ParallelAction(
//                new ActionUtil.ServoPositionAction(clawLeft,clawLeftClosed),
//                new ActionUtil.ServoPositionAction(clawRight,clawRightClosed)
//        );
//    }

//    public Action scoringPos() {
//        setOuttakeState(FourBarState.OUTTAKE_POSITION);
//        return new ParallelAction(
//                new ActionUtil.ServoPositionAction(servoArmLeft,scoringArmLeft),
//                new ActionUtil.ServoPositionAction(servoArmRight,scoringArmRight),
//                new ActionUtil.ServoPositionAction(outtakebar,scoringOuttakeBarPose)
//        );
//    }

//    public Action transferPos() {
//        setOuttakeState(FourBarState.TRANSFER_IDLE);
//        return new ParallelAction(
//                new ActionUtil.ServoPositionAction(servoArmRight,defaultArmRight),
//                new ActionUtil.ServoPositionAction(servoArmLeft,defaultArmLeft),
//                new ActionUtil.ServoPositionAction(outtakebar,defaultOuttakeBarPos)
//        );
//    }

    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case OUTTAKE_POSITION:
                robot.intake.capacPos = Intake.CapacPos.DOWN;
//                robot.slides.checkForIntake();
                lastTransferTime = -1;
                servoArmLeft.setPosition(scoringArmLeft);
                servoArmRight.setPosition(scoringArmRight);
                outtakebar.setPosition(scoringOuttakeBarPose);
                break;
            case TRANSFER_IDLE:
                robot.intake.capacPos = Intake.CapacPos.DOWN;
                clawState = ClawState.CLOSE;
//                robot.slides.checkForIntake();
                lastTransferTime = -1;
                rotateState = ROTATESTATE.DEFAULT;
                servoArmLeft.setPosition(defaultArmLeft);
                servoArmRight.setPosition(defaultArmRight);
                outtakebar.setPosition(defaultOuttakeBarPos);
                break;
            case TRANSFER_AUTO:
                if(capacTransferTimer == -1) {
                    capacTransferTimer = System.currentTimeMillis();
                    robot.intake.capacPos = Intake.CapacPos.UP;
                    clawState = ClawState.CLOSE;
                }
                if(System.currentTimeMillis() - capacTransferTimer > 400) {
                    servoArmLeft.setPosition(intakeArmLeft);
                    servoArmRight.setPosition(intakeArmRight);
                    outtakebar.setPosition(intakeTilt);
                    if(lastintakeAuto == -1) {
                        lastintakeAuto = System.currentTimeMillis();
                    }
                    if(System.currentTimeMillis() - lastintakeAuto > 600) {
                        clawState = ClawState.OPEN;
                    }
                    if(System.currentTimeMillis() - lastintakeAuto > 900) {
                        lastintakeAuto = -1;
                        capacTransferTimer = -1;
                        currentState = FourBarState.INTAKE_OUTTAKE;
                    }
                }
                break;
            case INTAKE_OUTTAKE:
                servoArmLeft.setPosition(scoringArmLeft);
                servoArmRight.setPosition(scoringArmRight);
                if(lastTransferTime == -1) {
                    lastTransferTime = System.currentTimeMillis();
                }
                if(System.currentTimeMillis() - lastTransferTime > 650) {
                    outtakebar.setPosition(scoringOuttakeBarPose);
                }
                if(System.currentTimeMillis() - lastTransferTime > 800) {
                    lastTransferTime = -1;
                    currentState = FourBarState.OUTTAKE_POSITION;
                }
                break;
            case INTAKE_POSITION:
                rotateState = ROTATESTATE.DEFAULT;
                outtakebar.setPosition(intakeTilt);
                servoArmLeft.setPosition(intakeArmLeft);
                servoArmRight.setPosition(intakeArmRight);
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
        switch (rotateState) {
            case DEFAULT:
                rotateServo.setPosition(rotateValues[2]);
                break;
            case LEFT45:
                rotateServo.setPosition(rotateValues[1]);
                break;
            case LEFT90:
                rotateServo.setPosition(rotateValues[0]);
                break;
            case RIGHT45:
                rotateServo.setPosition(rotateValues[3]);
                break;
            case RIGHT90:
                rotateServo.setPosition(rotateValues[4]);
                break;
        }

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
