package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

@TeleOp(name = "Test Roti")
public class motorTest extends LinearOpMode {
    MecanumDrive drive;
    HardwareQueue hardwareQueue;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"rightBack");
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class,"leftBack");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            motor.setPower(1);
            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(1);
        }
    }
}
