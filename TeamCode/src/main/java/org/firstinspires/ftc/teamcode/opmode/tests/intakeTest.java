package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.ArrayList;

@Config
@TeleOp(name = "intakeTest")
public class intakeTest extends LinearOpMode {
    Robot2 robot;
    double powerExtendo = 0;
    GamePadController g1;
    boolean pixelsFound = false;

    DcMotorEx intakeMotor;
    DcMotorEx extendoMotor;
    Servo capac,tilt;
    public enum ExtendoState {
        MANUAL,
        IDLE,
    }

    public enum CapacState {
        CLOSE,
        OPEN
    }


    public enum TiltState {
        LOW,
        HIGH,
        STACK2,
        STACK3,
        STACK4,
        STACK5
    }

    public enum MotorState {
        REVERSE,
        IDLE,
        FORWARD
    }

    ExtendoState extendoState = ExtendoState.IDLE;
    ArrayList<TiltState> stack = new ArrayList<>();
    ArrayList<Double> stackPos = new ArrayList<>();
    public static double capacInchis = 0;
    public static double capacDeschis = 0.7;
    double[] manualPowers = new double[]{0.8, -0.8, 0.0};

    public static int indexManualPowers = 2;
    public static double indexTilt = 0;
    public static double forwardPower = 1;
    public static double reversePower = -0.6;

    public static double tiltLow = 0.55;
    public static double tiltHigh = 0.7;

    MotorState motorState = MotorState.IDLE;
    CapacState capacState = CapacState.CLOSE;
    TiltState tiltState = TiltState.LOW;

    DigitalChannel poluluLeft,poluluRight;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        capac = hardwareMap.get(Servo.class,"capac");
        tilt = hardwareMap.get(Servo.class,"tilt");
        extendoMotor = hardwareMap.get(DcMotorEx.class,"extendo");

        poluluLeft = hardwareMap.get(DigitalChannel.class,"poluluLeft");
        poluluRight = hardwareMap.get(DigitalChannel.class,"poluluRight");

        poluluLeft.setMode(DigitalChannel.Mode.INPUT);
        poluluRight.setMode(DigitalChannel.Mode.INPUT);
        g1 = new GamePadController(gamepad1);

        while(opModeInInit()) {
            intakeMotor.setPower(0);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            extendoMotor.setPower(0);
            extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            extendoState = ExtendoState.IDLE;
            capac.setPosition(capacInchis);
            capacState = CapacState.CLOSE;
            tilt.setPosition(tiltLow);
            tiltState = TiltState.LOW;
            addStack();
        }
        waitForStart();

        while(opModeIsActive()) {
            g1.update();
            if(g1.aOnce()) {
                if(motorState == MotorState.FORWARD) motorState = MotorState.IDLE;
                else motorState = MotorState.FORWARD;
            }
            if(g1.xOnce()) {
                if(motorState == MotorState.REVERSE) motorState = MotorState.IDLE;
                else motorState = MotorState.REVERSE;
            }


            if(g1.bOnce()) {
                if(capacState == CapacState.CLOSE) capacState = CapacState.OPEN;
                else capacState = CapacState.CLOSE;
            }
            if(g1.yOnce()) {
                if(tiltState!=TiltState.HIGH) {
                    tiltState = TiltState.HIGH;
                }else {
                    tiltState = TiltState.LOW;
                }
            }
            if(g1.dpadUpOnce()) {
                indexTilt++;
                indexTilt = Math.min(indexTilt,stack.size()-1);
                tiltState = stack.get((int)indexTilt);
            }
            if(g1.dpadDownOnce()) {
                indexTilt--;
                indexTilt = Math.max(indexTilt,0);
                tiltState = stack.get((int)indexTilt);
            }
            powerExtendo = -g1.left_trigger + g1.right_trigger;
            if(pixelCheck() && !pixelsFound) {
                pixelsFound = true;
                g1.rumble(250);
                tiltState = TiltState.HIGH;
                motorState = MotorState.IDLE;
                capacState = CapacState.OPEN;
            }else if(!pixelCheck()) {
                pixelsFound = false;
            }

//            if(intakeMotor.isOverCurrent()) {
//                tiltState = TiltState.HIGH;
//                motorState = MotorState.REVERSE;
//            }

            update();

            extendoMotor.setPower(powerExtendo);
            telemetry.addData("tiltPos",tilt.getPosition());
            telemetry.addData("capacPos",capac.getPosition());
            telemetry.addData("intakeSpeed",capac.getPosition());
            telemetry.addData("motorState",motorState);
            telemetry.addData("capacState",capacState);
            telemetry.addData("tiltState",tiltState);
            telemetry.addData("extendoState",extendoState);
            telemetry.addData("indexTilt",indexTilt);
            telemetry.addData("motor voltage",intakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("pixelCheck",pixelCheck());
            telemetry.addData("extendoPower",powerExtendo);
            telemetry.addData("Left",poluluLeft.getState());
            telemetry.addData("Right",poluluRight.getState());
            telemetry.addData("manualPowers",manualPowers[indexManualPowers]);
            telemetry.update();
        }
    }
    void addStack() {
        stack.add(TiltState.LOW);
        stack.add(TiltState.STACK2);
        stack.add(TiltState.STACK3);
        stack.add(TiltState.STACK4);
        stack.add(TiltState.HIGH);

        stackPos.add(tiltLow);
        stackPos.add(0.56);
        stackPos.add(0.58);
        stackPos.add(0.6);
        stackPos.add(0.61);
        stackPos.add(tiltHigh);

    }
    public void manual_extend() {
        extendoState = ExtendoState.MANUAL;
        indexManualPowers = 0;
    }

    public void manual_retract() {
        extendoState = ExtendoState.MANUAL;
        indexManualPowers = 1;
    }

    public void manual_IDLE() {
        indexManualPowers = 2;
        extendoState = ExtendoState.IDLE;
    }
    void update() {
        switch (tiltState) {
            case LOW:
                tilt.setPosition(tiltLow);
                break;
            case HIGH:
                tilt.setPosition(tiltHigh);
                break;
            case STACK2:
                tilt.setPosition(stackPos.get(1));
                break;
            case STACK3:
                tilt.setPosition(stackPos.get(2));
                break;
            case STACK4:
                tilt.setPosition(stackPos.get(3));
                break;
            case STACK5:
                tilt.setPosition(stackPos.get(4));
                break;
            default:
                break;
        }

        switch (capacState) {
            case CLOSE:
                capac.setPosition(capacInchis);
                break;
            case OPEN:
                capac.setPosition(capacDeschis);
                break;
            default:
                break;
        }

        switch (motorState) {
            case FORWARD:
                intakeMotor.setPower(forwardPower);
                break;
            case REVERSE:
                intakeMotor.setPower(reversePower);
                break;
            case IDLE:
                intakeMotor.setPower(0);
                break;
            default:
                break;
        }

        switch (extendoState) {
            case MANUAL:
                extendoMotor.setPower(manualPowers[indexManualPowers]);
                break;
            case IDLE:
                extendoMotor.setPower(0);
                break;
            default:
                break;
        }
    }

    boolean pixelCheck() {
        int pixels = 0;
        if(!poluluLeft.getState()) pixels++;
        if(!poluluRight.getState()) pixels++;
        return pixels == 2;
    }
}
