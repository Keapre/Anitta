package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Polulu Test")
public class PoluluTest extends LinearOpMode {


    DigitalChannel distanceSensor;
    DigitalChannel distanceSensor2;
    boolean test = false;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DigitalChannel.class,"poluluLeft");
        distanceSensor2 = hardwareMap.get(DigitalChannel.class,"poluluRight");

        distanceSensor.setMode(DigitalChannel.Mode.INPUT);
        distanceSensor2.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while(opModeIsActive()) {

            test = distanceSensor.getState();
            telemetry.addData("Status",test);
            telemetry.addData("Status2",distanceSensor2.getState());
            telemetry.update();
        }
    }
}
