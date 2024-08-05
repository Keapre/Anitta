package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.util.GamePadController;

@Config
@TeleOp(name = "ServoPose")
public class ServoPoseTester extends LinearOpMode {
    ServoImplEx left,right;
    ServoImplEx tilt,rotate;
    ServoImplEx clawLeft,clawRight;
    GamePadController g1;
    public static double leftPos = 0.5;
    public static double rightPos = 0.5;

    public static double rotatePos = 0.5;
    public static double tiltpos = 0.5;

    public static double clawLeftPos = 0.5;
    public static double clawRightPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(ServoImplEx.class,"leftservo");
        right = hardwareMap.get(ServoImplEx.class,"rightservo");
        rotate = hardwareMap.get(ServoImplEx.class,"rotateouttake");
        tilt = hardwareMap.get(ServoImplEx.class,"tiltouttake");
        clawLeft = hardwareMap.get(ServoImplEx.class,"clawLeft");
        clawRight = hardwareMap.get(ServoImplEx.class,"clawRight");
        g1 = new GamePadController(gamepad1);
        left.setPosition(leftPos);
        right.setPosition(rightPos);
        tilt.setPosition(tiltpos);
        rotate.setPosition(rotatePos);
        clawLeft.setPosition(clawLeftPos);
        clawRight.setPosition(clawRightPos);
        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            if(g1.xOnce()) {
                left.setPosition(leftPos);
                right.setPosition(rightPos);
                tilt.setPosition(tiltpos);
                rotate.setPosition(rotatePos);
                clawRight.setPosition(clawRightPos);
                clawLeft.setPosition(clawLeftPos);
            }
        }
    }
}
