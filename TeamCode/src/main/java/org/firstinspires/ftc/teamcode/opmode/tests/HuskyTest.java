package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Vision.Husky;

@TeleOp(name = "Husky")
public class HuskyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Husky husky = new Husky(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            int location = husky.getLocation(true);
            telemetry.addData("location",location);
            telemetry.update();
        }
    }
}
