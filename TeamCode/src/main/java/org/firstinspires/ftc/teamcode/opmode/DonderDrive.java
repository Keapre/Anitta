package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.SmartGameTimer;


/*
TODO:
TO TUNE:
1.outtake positions(ASAP)
2.Slides (inch per tick)
3.intake positions
4.LOCALIZATION / rr 1.0
5.extendo

 */

@TeleOp
public class DonderDrive extends LinearOpMode {

    public static double TURN_SPEED = 0.75;
    public static double DRIVE_SPEED = 1;
    public static double SLOW_TURN_SPEED = 0.3;
    public static double D2_SLOW_TURN = 0.25;
    public static double SLOW_DRIVE_SPEED = 0.3;
    public static double VISION_RANGE = 20;
    public static double VISION_CLOSE_DIST = 5;
    private SmartGameTimer smartGameTimer;
    private GamePadController g1;
    private Robot2 robot;

    int pixelCount = 0;
    private long lastLoopFinish = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("initializing...");
        telemetry.update();

        g1 = new GamePadController(gamepad1);


        robot = new Robot2(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            g1.update();
            robot.update();

            intakeUpdate();
            updateDriveTrain();
            updatePlane();
            updateExtendo();
            updateSlides();
            outtakeUpdate();
        }
    }

    public void outtakeUpdate() {

        if(g1.xOnce()) {
            if(robot.outtake.clawState == Outtake.ClawState.OPEN || robot.outtake.clawState == Outtake.ClawState.LEFT_OPEN || robot.outtake.clawState == Outtake.ClawState.RIGHT_OPEN) {
                robot.outtake.clawState = Outtake.ClawState.CLOSE;
            } else {
                robot.outtake.clawState = Outtake.ClawState.RIGHT_OPEN;
            }
        }
        if(g1.bOnce()) {
            if(robot.outtake.clawState == Outtake.ClawState.OPEN || robot.outtake.clawState == Outtake.ClawState.LEFT_OPEN || robot.outtake.clawState == Outtake.ClawState.RIGHT_OPEN) {
                robot.outtake.clawState = Outtake.ClawState.CLOSE;
            } else {
                robot.outtake.clawState = Outtake.ClawState.LEFT_OPEN;
            }
        }

        if(g1.yOnce()) {
            if(robot.outtake.lastImportant == Outtake.FourBarState.TRANSFER_INTAKE) {
                robot.outtake.currentState = Outtake.FourBarState.OUTTAKE_POSITION;
            } else {
                robot.outtake.currentState = Outtake.FourBarState.TRANSFER_INTAKE;
            }
        }
    }

    public void intakeUpdate() {
        pixelCount = robot.sensors.pixelCounter();
        if(g1.aOnce()) {
            if(robot.intake.intakeState == Intake.IntakeState.IDLE) {
                robot.intake.capacPos = Intake.CapacPos.DOWN;
                robot.intake.intakeState = Intake.IntakeState.FORWARD;
            } else if(robot.intake.intakeState == Intake.IntakeState.FORWARD) {
                robot.intake.intakeState = Intake.IntakeState.IDLE;
            }
        }
        if(pixelCount == 2 || (pixelCount == 1 && robot.drivetrain.isBusy())) { // maybe 1 pixel works?
            robot.intake.reverseForTime(700);
            robot.intake.capacPos = Intake.CapacPos.UP;
        }
        if(g1.dpadDownOnce()) {
            robot.intake.addTilt(-0.1);
        }
        if(g1.dpadUpOnce()) {
            robot.intake.addTilt(0.1);
        }

        robot.intake.update();
    }

    public void updateSlides(){
        if(g1.leftTrigger()) {
            robot.slides.manual_retract();
        } else if(g1.rightTrigger()) {
            robot.slides.manual_extend();
        } else {
            robot.slides.manual_IDLE();
        }

        robot.slides.update();
    }

    public void updateExtendo() {
        if(g1.leftBumper()) {
            robot.extendo.manual_extend();
        }else if(g1.rightBumper()) {
            robot.extendo.manual_retract();
        } else {
            robot.extendo.manual_IDLE();
        }

        robot.extendo.update();
    }
    void updateDriveTrain() {
        if(g1.leftStickButtonOnce()) {
            robot.wolfPackDrive.SLOW_MODE = !robot.wolfPackDrive.SLOW_MODE;
        }

        if(g1.rightStickButtonOnce()) {
            robot.wolfPackDrive.resetYaw();
        }

        robot.wolfPackDrive.driveFromController(g1);
    }

    void updatePlane() {
        if(g1.startOnce()) {
            robot.plane.setOut();
        }

        robot.plane.update();
    }



}
