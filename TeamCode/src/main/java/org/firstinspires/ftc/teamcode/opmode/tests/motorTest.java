package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

@TeleOp(name = "Test Roti")
public class motorTest extends LinearOpMode {
    MecanumDrive drive;
    HardwareQueue hardwareQueue;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareQueue = new HardwareQueue();
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0),hardwareQueue);

        waitForStart();
        while(opModeIsActive()){
            drive.rightFront.setPowerForced(1);
        }
    }
}
