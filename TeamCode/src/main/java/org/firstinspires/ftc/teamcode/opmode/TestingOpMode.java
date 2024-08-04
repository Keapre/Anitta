package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot3;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake2;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;

@Config
@TeleOp(name = "teleOpConfig")
public class TestingOpMode extends LinearOpMode {
    Robot3 robot2;
    GamePadController g1;
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = Perioada.TELEOP;
        robot2= new Robot3(hardwareMap);
        g1 = new GamePadController(gamepad1);

        waitForStart();

        while(opModeIsActive()) {
            g1.update();

            intakeUpdate();
            extendoUpdate();
            robot2.drive.driveGamepad(g1);
            updateTelemetry();
            robot2.update();
        }
    }

    private void intakeUpdate() {
        if(g1.aOnce()) {
            if(robot2.intake.intakeState == Intake2.IntakeState.IDLE) {
                robot2.intake.intakeState = Intake2.IntakeState.FORWARD;
            }else {
                robot2.intake.intakeState = Intake2.IntakeState.IDLE;
            }
        }
        if(g1.yOnce()) {
            if(robot2.intake.tiltPos !=Intake2.TiltState.HIGH) {
                robot2.intake.updateIndexTiltHigh();
            }else {
                robot2.intake.updateIndexTiltLow();
            }
        }
        if(g1.bOnce()) {
            if(robot2.intake.capacPos == Intake2.CapacPos.DOWN) {
                robot2.intake.capacPos = Intake2.CapacPos.UP;
            }else {
                robot2.intake.capacPos = Intake2.CapacPos.DOWN;
            }
        }
        if(g1.xOnce()) {
            if(robot2.intake.intakeState == Intake2.IntakeState.IDLE) {
                robot2.intake.intakeState = Intake2.IntakeState.REVERSE;
            }else {
                robot2.intake.intakeState = Intake2.IntakeState.IDLE;
            }
        }
        if(g1.dpadUpOnce()) robot2.intake.addTilt();
        if(g1.dpadDownOnce()) robot2.intake.substractTilt();

        robot2.intake.checkForPixels(g1);
    }
    private void extendoUpdate() {
        robot2.extendo.updatePower(-g1.left_trigger + g1.right_trigger);
    }
    private void updateTelemetry() {
        telemetry.addData("capacPose",robot2.intake.capacPos);
        telemetry.addData("tiltPos",robot2.intake.currentTilt);
        telemetry.addData("indexTilt",robot2.intake.getIndexTilt());
        telemetry.addData("speed Motor",robot2.intake.intakeState);
        telemetry.addData("extendoPower",robot2.extendo.getPower());
        telemetry.addData("extendoState",robot2.extendo.extendoState);
        telemetry.addData("pixelLeft",robot2.sensors.getLeftDistance());
        telemetry.addData("pixelRight",robot2.sensors.getRightDistance());
        telemetry.addData("Pixels",Globals.NUM_PIXELS);
        telemetry.addData("Debug",robot2.intake.getDebug());
        telemetry.update();
    }
}
