package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
@TeleOp(name = "Extendo Test")
public class ExtendoTest extends LinearOpMode {

    public enum ExtendoState{
        EXTEND,
        IDLE,
        RETRACT
    }

    public double signum(double val) {
        return (val != 0) ? Math.signum(val) : +1;
    }

    public static double ff = 0.07;

    DcMotorEx extendoMotor1;
    GamePadController g1;
    ExtendoState extendoState = ExtendoState.IDLE;

    public static double extendoPower = 1;

    public static double power = 0;

    public static double retractPower = -1;

    public static double kP = 0.3,kI = 0.002,kD = 0.001; /// kP = 0.0395,kI = 0.002,kD = 0.001;

    public static double maxLen = 700;
    public static PIDCoefficients coef = new PIDCoefficients(kP, kI, kD);
    //    public static PIDCoefficients coef = new PIDCoefficients(0, 0, 0);
    public static PIDController pid = new PIDController(coef.p, coef.i, coef.d);
    public static double target = 0,current = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        extendoMotor1 = hardwareMap.get(DcMotorEx.class, "extendo");

        g1 = new GamePadController(gamepad1);
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initiliazeMotor();

        while(opModeIsActive()) {
            g1.update();

            if(g1.rightTrigger() ) extendoState = ExtendoState.EXTEND;
            else if(g1.leftTrigger()) extendoState = ExtendoState.RETRACT;
            else {
                extendoState = ExtendoState.IDLE;
            }
            update();
            telemetry.addData("Current Pos",current);
            telemetry.addData("Target Pos",target);
            telemetry.update();
        }
    }
    //max - 2200
    //
    void initiliazeMotor() {

        extendoMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void update() {
//        updatePid();
//        pid.setTolerance(3);
//        pid.setSetPoint(target);
//        current = extendoMotor1.getCurrentPosition();
//        double power = pid.calculate(current);
//
//        power+=ff * signum(power);
//        extendoMotor1.setPower(power);
        switch (extendoState){
            case EXTEND:
                extendoMotor1.setPower(1);
                break;
            case RETRACT:
                extendoMotor1.setPower(-1);
                break;
            case IDLE:
                extendoMotor1.setPower(-0.05);
                break;
        }
    }

    void updatePid() {
        pid.setPID(kP,kI,kD);
    }
}
