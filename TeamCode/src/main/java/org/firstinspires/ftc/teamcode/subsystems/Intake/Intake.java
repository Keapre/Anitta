package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public enum TiltPos {
        UP,
        DOWN,
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
    public double timeReverse = 0;

    public long reverseTimeStart = 0;
    public TiltPos tiltPos;
    public CapacPos capacPos;

    public IntakeState intakeState, lastIntakeState;

    public double currentTilt = 0,lastTilt = 0;
    double[] tiltPositions = new double[]{0.0, 0.5, 1.0};
    double[] capacPositions = new double[]{0.0, 1.0};

    double[] motorSpeed = new double[]{0.7, -0.7,-0.3, 0.0};

    final PriorityServo capac;

    final PriorityMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
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
        intakeMotor.setTargetPower(0);

        capacPos = CapacPos.UP;
        tiltPos = TiltPos.UP;
        intakeState = IntakeState.IDLE;
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
                    intakeState = lastIntakeState;
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
        tilt.setPosition(tilt.getPos() + adding);
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

}
