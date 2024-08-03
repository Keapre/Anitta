package org.firstinspires.ftc.teamcode.subsystems.Intake;

import android.provider.Settings;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public enum TiltPos {
        UP,
        DOWN,
        MANUAL,
        MIDDLE
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
    final PriorityServo tilt;
    public IntakeUpdate Update;
    public double timeReverse = 0;

    public long reverseTimeStart = 0;

    public TiltPos tiltPos;
    public CapacPos capacPos;
    Robot2 robot2;

    public IntakeState intakeState, lastIntakeState;

    public double currentTilt = 0,lastTilt = 0;
    double[] tiltPositions = new double[]{0.0, 0.5, 1.0};
    double[] capacPositions = new double[]{0.89, 0.19}; // 0 - open ,1 - closed

    double[] motorSpeed = new double[]{0.7, -0.7,-0.3, 0.0};

    final PriorityServo capac;

    final PriorityMotor intakeMotor;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Robot2 robot2) {
        tilt = new PriorityServo(new CachingServo(hardwareMap.get(Servo.class, "intakeTilt")),
                "intakeTilt",
                3,
                PriorityServo.ServoType.AXON_MINI,
                tiltPositions[0],
                tiltPositions[0],
                false
        );
        capac = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "intakeTilt")),
                "intakeCapac",
                2,
                PriorityServo.ServoType.AXON_MINI,
                capacPositions[0],
                capacPositions[0],
                false
        );
        intakeMotor = new PriorityMotor(
                new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor")),
                "intakeMotor",
                4
        );
        intakeMotor.setPowerForced( 0);

        intakeMotor.motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capacPos = CapacPos.UP;
        tiltPos = TiltPos.UP;
        intakeState = IntakeState.IDLE;

        this.robot2 = robot2;
        hardwareQueue.addDevice(tilt);
        hardwareQueue.addDevice(capac);
        hardwareQueue.addDevice(intakeMotor);
        Update = new IntakeUpdate(robot2);
    }


    public void update() {
        switch (tiltPos) {
            case UP:
                tilt.setPosition(tiltPositions[0]);
                break;
            case MIDDLE:
                tilt.setPosition(tiltPositions[1]);
                break;
            case DOWN:
                tilt.setPosition(tiltPositions[2]);
                break;
            default:
                break;
        }

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
                intakeMotor.setTargetPower(motorSpeed[0]);
                break;
            case REVERSE:
                intakeMotor.setTargetPower(motorSpeed[1]);
                break;
            case REVERSE_SLOW:
                intakeMotor.setTargetPower(motorSpeed[2]);
                break;
            case REVERSE_FOR_TIME:
                long elapsed = System.currentTimeMillis() - reverseTimeStart;
                if(elapsed < timeReverse) {
                    intakeMotor.setTargetPower((motorSpeed[1] + motorSpeed[2]) / 2);
                } else {
                    intakeState = IntakeState.IDLE;
                }
            case IDLE:
                intakeMotor.setTargetPower(motorSpeed[3]);
                break;
            default:
                break;
        }
    }


    public void reverseForTime(double time) {
        this.timeReverse = time;
        reverseTimeStart = System.currentTimeMillis();
        if(intakeState != IntakeState.REVERSE_FOR_TIME) {
            lastIntakeState = intakeState;
        }
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
        intakeMotor.setPowerForced(0);
        intakeState = IntakeState.IDLE;
    }
    public void addTilt(double adding) {
        currentTilt+=adding;
        currentTilt = Math.min(1.0, Math.max(0.0, currentTilt));
        tilt.setPosition(currentTilt);
    }

    public Action setTiltHeight(int setPoint) {
        return new ActionUtil.ServoPositionAction(tilt,tiltPositions[setPoint]);
    }

    public Action intakeMotor() {
        return new ActionUtil.DcMotorExPowerAction(intakeMotor,motorSpeed[0]);
    }

    public Action reverseMotor(double t) {
        return new reverseForTime(t);
    }

    public Action offMotor() {
        return new ActionUtil.DcMotorExPowerAction(intakeMotor,motorSpeed[4]);
    }

    public Action capacDeschis() {
        capacPos = CapacPos.UP;
        return new ActionUtil.ServoPositionAction(capac,capacPositions[0]);
    }

    public Action capacInchis() {
        capacPos = CapacPos.DOWN;
        return new ActionUtil.ServoPositionAction(capac,capacPositions[1]);
    }

    public void intakeReverseInstant() {

        intakeMotor.motor[0].setPower(motorSpeed[1]);
    }

    public void LowTilt() {
        tiltPos = TiltPos.DOWN;
    }

    public void MiddleTilt() {
        tiltPos = TiltPos.MIDDLE;
    }

    public void HighTilt() {
        tiltPos = TiltPos.UP;
    }

    private class IntakeCount implements  Action {
        private long waitUntil;
        private long finalTime;
        private int pixelCount;
        private boolean done;
        private boolean jammed;
        public IntakeCount(boolean fast) {
            this.waitUntil = System.currentTimeMillis() + 300;
            this.finalTime = System.currentTimeMillis() + (fast ? 3000 : 6000);

        }
        public Action intakeCount(boolean fast) {
            return new SequentialAction(
                    new IntakeCount(fast),
                    reverseMotor(300)
            );
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(done) {
                return System.currentTimeMillis() < waitUntil;
            }

            if(jammed) {
                if(!intakeMotor.motor[0].isOverCurrent()) {
                    this.jammed = false;
                    return System.currentTimeMillis() < finalTime;
                }
                if (System.currentTimeMillis() >= waitUntil) {
                    intakeReverseInstant();
                    return System.currentTimeMillis() < finalTime;
                }
            }
            if (intakeMotor.motor[0].isOverCurrent()) {
                this.jammed = true;
                return System.currentTimeMillis() < finalTime;
            }
            pixelCount = robot2.sensors.pixelCounter();
            if (pixelCount >= 2) {
                done = true;
                return true;
            }
            return System.currentTimeMillis() < finalTime;
        }
    }
    private class changeTiltState implements Action {

        TiltPos state;

        public changeTiltState(TiltPos state) {
            this.state = state;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            tiltPos = state;
            return false;
        }
    }
    public Action changeState(TiltPos state) {
        return new changeTiltState(state);
    }
    private class changeIntakeMotorState implements Action {

        IntakeState state;

        public changeIntakeMotorState(IntakeState state) {
            this.state = state;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeState = state;
            return false;
        }
    }
    public Action changeintakeState(IntakeState state) {
        return new changeIntakeMotorState(state);
    }

    private class reverseForTime implements Action {

        private double finishTime = 0;
        private boolean first = true;

        public reverseForTime(double t) {
            finishTime = System.currentTimeMillis() + t;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first) {
                intakeMotor.motor[0].setPower(motorSpeed[1]);
                first = false;
            }

            if(System.currentTimeMillis() >= finishTime) {
                intakeForceOff();
                return false;
            }
            return true;
        }
    }
}
