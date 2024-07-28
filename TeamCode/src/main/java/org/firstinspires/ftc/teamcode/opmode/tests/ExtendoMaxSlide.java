package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

public class ExtendoMaxSlide extends LinearOpMode {

    HardwareQueue hardwareQueue;
    Extendo extendo;
    Sensors sensors;
    double maxLength = 0;
    GamePadController g1;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        hardwareQueue = new HardwareQueue();
        g1 = new GamePadController(gamepad1);
        sensors = new Sensors(hardwareMap,hardwareQueue);
        extendo = new Extendo(hardwareMap, hardwareQueue,sensors);
        while (opModeIsActive()) {
            if(g1.dpadUpOnce()) extendo.manual_extend();
            else if(g1.dpadDownOnce()){
                extendo.manual_retract();
            }else {
                extendo.manual_IDLE();
            }
            if(extendo.getLength() > maxLength) {
                maxLength = extendo.getLength();
            }
            extendo.update();
            hardwareQueue.update();
            telemetry.addData("Max Length", maxLength);
        }
    }
}
