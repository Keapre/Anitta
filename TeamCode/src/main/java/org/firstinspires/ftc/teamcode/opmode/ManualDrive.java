package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.SmartGameTimer;

@TeleOp(name = "TeleOP")
public class ManualDrive extends LinearOpMode {
    public static double TURN_SPEED = 0.75;
    public static double DRIVE_SPEED = 1;
    public static double SLOW_TURN_SPEED = 0.3;
    public static double D2_SLOW_TURN = 0.25;
    public static double SLOW_DRIVE_SPEED = 0.3;
    public static double VISION_RANGE = 20;
    public static double VISION_CLOSE_DIST = 5;
    private SmartGameTimer smartGameTimer;
    private GamePadController g1, g2;
    private Robot robot;
    private long lastLoopFinish = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("initializing...");
        telemetry.update();

        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);

        g1.update();

        robot = new Robot(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            g1.update();
            robot.update();

            robot.wolfPackDrive.driveFromController(g1);
        }


    }
}
