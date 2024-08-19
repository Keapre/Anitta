package org.firstinspires.ftc.teamcode.opmode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.control.EricPid;

@Config
@TeleOp(name = "Elevator State")
public class ElevatorTest extends LinearOpMode {

    public enum ElevatorState{
        EXTEND,
        IDLE,
        RETRACT
    }
    DcMotorEx elevatorMotor1;
    DcMotorEx elevatorMotor2;
    GamePadController g1;
    ElevatorState elevatorState = ElevatorState.IDLE;

    public static double idlePower = 0;
    public static double elevatorPower = 1;

    public static double retractPower = -1;

    public static double targetLength = 0;
    public static double currentLength = 0;
    public static double kP = 0,kI= 0,kD = 0;
    public EricPid pid;
    @Override
    public void runOpMode() throws InterruptedException {
        elevatorMotor1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        elevatorMotor2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        g1 = new GamePadController(gamepad1);
        waitForStart();

        double power = 0;
        initiliazeMotor();
        pid = new EricPid(kP,kI,kD);

        while(opModeIsActive()) {
            g1.update();
            if(g1.rightBumperOnce()) {
                if(elevatorState == ElevatorState.EXTEND){
                    elevatorState = ElevatorState.IDLE;
                }else {
                    elevatorState = ElevatorState.EXTEND;
                }
            }
            if (g1.leftBumperOnce()) {
                if(elevatorState == ElevatorState.RETRACT) {
                    elevatorState = ElevatorState.IDLE;
                }else {
                    elevatorState = ElevatorState.RETRACT;
                }
            }
            update();

        }
    }

    void initiliazeMotor() {
        elevatorMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    void update() {
//        updatePid();
//        currentLength = elevatorMotor1.getCurrentPosition();
//        pid.setTarget(targetLength);
//        double power = pid.update(currentLength) + idlePower;
//        elevatorMotor2.setPower(power);
//        elevatorMotor1.setPower(power);
        switch (elevatorState){
            case EXTEND:
                elevatorMotor1.setPower(1);
                elevatorMotor2.setPower(1);
                break;
            case RETRACT:
                elevatorMotor1.setPower(-1);
                elevatorMotor2.setPower(-1);
                break;
            case IDLE:
                Log.w("debug", "setting idle");
                elevatorMotor1.setPower(idlePower);
                elevatorMotor2.setPower(idlePower);
                break;
        }
    }

    void updatePid() {
        pid.updatePid(kP,kI,kD);
    }
}
