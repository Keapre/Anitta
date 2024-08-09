package org.firstinspires.ftc.teamcode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;
import org.firstinspires.ftc.teamcode.util.Utils;

import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake2 implements Subsystem {

    public enum TiltState {
        LOW,
        HIGH,
        STACK2,
        STACK3,
        STACK4,
        STACK5;

        public TiltState previous() {
            return values()[ordinal() - 1];
        }

        public TiltState next() {
            return values()[ordinal() + 1];
        }
    }


    public enum CapacPos {
        UP,
        DOWN
    }

    public enum IntakeState {
        FORWARD,
        REVERSE,
        REVERSE_SLOW,
        REVERSE_FOR_TIME,
        IDLE
    }

    final CachingServo tilt;
    public static boolean findPixels = false;
    public IntakeUpdate Update;
    public double timeReverse = 0;
    public boolean debug = false;

    public long reverseTimeStart = 0;

    public TiltState tiltPos = TiltState.LOW;
    public CapacPos capacPos = CapacPos.DOWN;
    Robot3 robot2;

    public IntakeState intakeState = IntakeState.IDLE, lastIntakeState = IntakeState.IDLE;

    public static int indexTilt = 0;
    public static int rotateIndex = 2;
    public double currentTilt = 0, lastTilt = 0;
    public static double[] tiltPositions = new double[]{0.6, 0.62, 0.64, 0.66, 0.68, 0.72};
    public static double[] capacPositions = new double[]{0.7, 0}; // 0 - open ,1 - closed
    public static double[] motorSpeed = new double[]{1, -1, -0.3, 0.0};

    final CachingServo capac;
    public boolean started = false;

    final CachingDcMotorEx intakeMotor;

    public Intake2(HardwareMap hardwareMap, Robot3 robot2) {
        findPixels = false;
        started = false;
        Update = new IntakeUpdate(robot2);
        tilt = new CachingServo(hardwareMap.get(Servo.class, "tilt"));
        capac = new CachingServo(hardwareMap.get(Servo.class, "capac"));
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        intakeMotor.setPower(0);
        //intakeMotor.motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capacPos = CapacPos.DOWN;
        tiltPos = TiltState.LOW;
        intakeState = IntakeState.IDLE;

        this.robot2 = robot2;
    }

    public void updateIndexTiltLow() {
        indexTilt = 0;
    }

    public void updateIndexTiltHigh() {
        indexTilt = 5;
    }

    public void resetRotateIndex() {
        rotateIndex = 2;
    }

    public void update() {
        tilt.setPosition(tiltPositions[indexTilt]);
        switch (capacPos) {
            case UP:
                capac.setPosition(capacPositions[0]);
                break;
            case DOWN:
                capac.setPosition(capacPositions[1]);
                break;
            default:
                break;
        }

        switch (intakeState) {
            case FORWARD:
                intakeMotor.setPower(1);
                break;
            case REVERSE:
                intakeMotor.setPower(-1);
                break;
            case REVERSE_SLOW:
                intakeMotor.setPower(motorSpeed[2]);
                break;
            case REVERSE_FOR_TIME:
                long elapsed = System.currentTimeMillis() - reverseTimeStart;
                if (System.currentTimeMillis() < reverseTimeStart + timeReverse) {
                    debug = true;
                    intakeMotor.setPower(motorSpeed[1]);
                } else {
                    debug = false;
                    intakeMotor.setPower(motorSpeed[3]);
                    intakeState = IntakeState.IDLE;
                }
                break;
            case IDLE:
                intakeMotor.setPower(motorSpeed[3]);
                break;
            default:
                break;
        }
    }


    public boolean getDebug() {
        return debug;
    }

    public void reverseForTime(double time) {
        this.timeReverse = time;
        reverseTimeStart = System.currentTimeMillis();
        intakeState = IntakeState.REVERSE_FOR_TIME;
    }


    public void intakeOff() {
        intakeState = IntakeState.IDLE;
    }

    public void slowReverse() {
        intakeState = IntakeState.REVERSE_SLOW;
    }

    public void intakeReverse() {
        intakeState = IntakeState.REVERSE;
    }

    public void intakeForward() {
        intakeState = IntakeState.FORWARD;
    }

    public void intakeForceOff() {
        intakeMotor.setPower(0);
        intakeState = IntakeState.IDLE;
    }

    public double reverseTimeStarting = 0;

    public void addTilt() {
        indexTilt++;
        Utils.minMaxClip(indexTilt,0,5);
    }

    public void substractTilt() {
        indexTilt--;
        Utils.minMaxClip(indexTilt,0,5);
    }

    public int getIndexTilt() {
        return indexTilt;
    }
//    public Action setTiltHeight(int setPoint) {
//        return new ActionUtil.ServoPositionAction(tilt,tiltPositions[setPoint]);
//    }

//    public Action intakeMotor() {
//        return new ActionUtil.DcMotorExPowerAction(intakeMotor,motorSpeed[0]);
//    }

//    public Action reverseMotor(double t) {
//        return new reverseForTime(t);
//    }

//    public Action offMotor() {
//        return new ActionUtil.DcMotorExPowerAction(intakeMotor,motorSpeed[4]);
//    }

//    public Action capacDeschis() {
//        capacPos = CapacPos.UP;
//        return new ActionUtil.ServoPositionAction(capac,capacPositions[0]);
//    }

//    public Action capacInchis() {
//        capacPos = CapacPos.DOWN;
//        return new ActionUtil.ServoPositionAction(capac,capacPositions[1]);
//    }

    public void intakeReverseInstant() {

        intakeMotor.setPower(motorSpeed[1]);
    }

    public void LowTilt() {
        tiltPos = TiltState.LOW;
    }

    public void checkForPixels(GamePadController g1) {
        if (Globals.NUM_PIXELS == 2 && !findPixels) {
            if (!started) {
                reverseTimeStarting = System.currentTimeMillis();
                started = true;
            } else {
                if (System.currentTimeMillis() > reverseTimeStarting + 1400) {
                    g1.rumble(500);
                    findPixels = true;
                    reverseForTime(1000);
                    capacPos = CapacPos.UP;
                    findPixels = true;
                }
            }

        }
        if (Globals.NUM_PIXELS != 2) {
            started = false;
            findPixels = false;
        }
    }

    public void HighTilt() {
        tiltPos = TiltState.HIGH;
    }

//    private class IntakeCount implements  Action {
//        private long waitUntil;
//        private long finalTime;
//        private int pixelCount;
//        private boolean done;
//        private boolean jammed;
//        public IntakeCount(boolean fast) {
//            this.waitUntil = System.currentTimeMillis() + 300;
//            this.finalTime = System.currentTimeMillis() + (fast ? 3000 : 6000);
//
//        }
//        public Action intakeCount(boolean fast) {
//            return new SequentialAction(
//                    new IntakeCount(fast),
//                    reverseMotor(300)
//            );
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(done) {
//                return System.currentTimeMillis() < waitUntil;
//            }
//
//            if(jammed) {
//                if(!intakeMotor.isOverCurrent()) {
//                    this.jammed = false;
//                    return System.currentTimeMillis() < finalTime;
//                }
//                if (System.currentTimeMillis() >= waitUntil) {
//                    intakeReverseInstant();
//                    return System.currentTimeMillis() < finalTime;
//                }
//            }
//            if (intakeMotor.isOverCurrent()) {
//                this.jammed = true;
//                return System.currentTimeMillis() < finalTime;
//            }
//            pixelCount = robot2.sensors.pixelCounter();
//            if (pixelCount >= 2) {
//                done = true;
//                return true;
//            }
//            return System.currentTimeMillis() < finalTime;
//        }
//    }
//    private class changeTiltState implements Action {
//
//        TiltState state;
//
//        public changeTiltState(TiltState state) {
//            this.state = state;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            tiltPos = state;
//            return false;
//        }
//    }
//    public Action changeState(TiltState state) {
//        return new changeTiltState(state);
//    }
//    private class changeIntakeMotorState implements Action {
//
//        IntakeState state;
//
//        public changeIntakeMotorState(IntakeState state) {
//            this.state = state;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            intakeState = state;
//            return false;
//        }
//    }
//    public Action changeintakeState(IntakeState state) {
//        return new changeIntakeMotorState(state);
//    }
//
//    private class reverseForTime implements Action {
//
//        private double finishTime = 0;
//        private boolean first = true;
//
//        public reverseForTime(double t) {
//            finishTime = System.currentTimeMillis() + t;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(first) {
//                intakeMotor.setPower(motorSpeed[1]);
//                first = false;
//            }
//
//            if(System.currentTimeMillis() >= finishTime) {
//                intakeForceOff();
//                return false;
//            }
//            return true;
//        }
//    }
//}
}