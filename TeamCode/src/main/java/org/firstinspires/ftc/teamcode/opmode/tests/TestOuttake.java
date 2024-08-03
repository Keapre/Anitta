package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TestOuttake extends LinearOpMode {

    Servo fourbar;
    Servo claw1,claw2;
    Servo arm1,arm2;
    @Override
    public void runOpMode() throws InterruptedException {

        fourbar = hardwareMap.get(Servo.class,"fourbar");
        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        arm1 = hardwareMap.get(Servo.class,"arm1");
        arm2 = hardwareMap.get(Servo.class,"arm2");
        waitForStart();


    }
}
