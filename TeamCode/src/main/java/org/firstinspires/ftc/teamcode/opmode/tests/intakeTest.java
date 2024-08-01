package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.util.GamePadController;

@Config
@TeleOp(name = "intakeTest")
public class intakeTest extends LinearOpMode {
    Robot2 robot;
    GamePadController g1;

    DcMotorEx intakeMotor;
    Servo capac,tilt;

    public static double capacInchis = 0;
    public static double capacDeschis = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        capac = hardwareMap.get(Servo.class,"capac");
        tilt = hardwareMap.get(Servo.class,"tilt");
        g1 = new GamePadController(gamepad1);
        while(opModeInInit()) {
            intakeMotor.setPower(0);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            capac.setPosition(capacInchis);
        }
        waitForStart();

        while(opModeIsActive()) {
            g1.update();
            if(g1.aOnce()) {
                if(intakeMotor.getPower() == 1) {
                    intakeMotor.setPower(0);
                }else {
                    intakeMotor.setPower(1);
                }
            }

            if(g1.bOnce()) {
                if(capac.getPosition() == capacDeschis) {
                    capac.setPosition(capacInchis);
                }else {
                    capac.setPosition(capacDeschis);
                }
            }
        }
    }
}
