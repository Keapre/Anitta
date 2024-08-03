package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.GamePadController;

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

    public static double elevatorPower = 0.8;

    public static double retractPower = -0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        elevatorMotor1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        elevatorMotor2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        g1 = new GamePadController(gamepad1);
        waitForStart();

        initiliazeMotor();

        while(opModeIsActive()) {
            g1.update();
            if(g1.yOnce()) {
                if(elevatorState == ElevatorState.IDLE) {
                    elevatorState = ElevatorState.EXTEND;
                } else {
                    elevatorState = ElevatorState.IDLE;
                }
            }
            if(g1.aOnce()) {
                if(elevatorState == ElevatorState.IDLE) {
                    elevatorState = ElevatorState.RETRACT;
                } else {
                    elevatorState = ElevatorState.IDLE;
                }
            }
            update();
            telemetry.addData("Elevator State", elevatorState);
            telemetry.update();
        }
    }

    void initiliazeMotor() {
        elevatorMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        elevatorMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        elevatorMotor1.setPower(0);
        elevatorMotor2.setPower(0);
    }

    void update() {
        switch (elevatorState) {
            case EXTEND:
                elevatorMotor1.setPower(elevatorPower);
                elevatorMotor2.setPower(elevatorPower);
                break;
            case IDLE:
                elevatorMotor1.setPower(0);
                elevatorMotor2.setPower(0);
                break;
            case RETRACT:
                elevatorMotor1.setPower(retractPower);
                elevatorMotor2.setPower(retractPower);
                break;
        }
    }
}
