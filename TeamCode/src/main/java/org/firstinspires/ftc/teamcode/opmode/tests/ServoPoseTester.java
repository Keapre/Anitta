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
@TeleOp(name = "TESTSPM")
public class ServoPoseTester extends LinearOpMode {

    ServoImplEx tilt,capac;
    GamePadController g1;

    public static double capacPos = 0;
    public static double tiltPos = 0.556;

    @Override
    public void runOpMode() throws InterruptedException {
        tilt = hardwareMap.get(ServoImplEx.class,"tilt");
        capac = hardwareMap.get(ServoImplEx.class,"capac");

        tilt.setPosition(tiltPos);
        capac.setPosition(capacPos);

        waitForStart();

        while(opModeIsActive()) {
            tilt.setPosition(tiltPos);
            capac.setPosition(capacPos);
        }
    }

}
