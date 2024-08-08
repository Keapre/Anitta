package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot3;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo2;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake2;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake2;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides2;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;

@Config
@TeleOp(name = "teleOpConfig")
public class TestingOpMode extends LinearOpMode {
    Robot3 robot2;
    GamePadController g1;
    private long lastLoopFinish = 0;
    public static boolean usePoluu = false;

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
            outtakeUpdate();
            robot2.drive.driveFromController(g1);
            slidesUpdate();
            updateTelemetry();
            robot2.update();
        }
    }

    private void intakeUpdate() {
        if(g1.aOnce()) {
            if(robot2.intake.intakeState == Intake2.IntakeState.IDLE) {
                robot2.intake.intakeState = Intake2.IntakeState.FORWARD;
            }else if(robot2.intake.intakeState == Intake2.IntakeState.FORWARD){
                robot2.intake.intakeState = Intake2.IntakeState.REVERSE;
            }else {
                robot2.intake.intakeState = Intake2.IntakeState.IDLE;
            }
        }

        if(g1.xOnce()) { //guide
            if(robot2.intake.capacPos == Intake2.CapacPos.DOWN) {
                robot2.intake.capacPos = Intake2.CapacPos.UP;
            }else {
                robot2.intake.capacPos = Intake2.CapacPos.DOWN;
            }
        }

        if(g1.dpadUpOnce()) robot2.intake.addTilt();
        if(g1.dpadDownOnce()) robot2.intake.substractTilt();

        if(usePoluu) robot2.intake.checkForPixels(g1);
    }

    private void extendoUpdate() {
        if(g1.rightBumper()) {
            robot2.extendo.extendoState = Extendo2.ExtendoState.EXTEND;
            return;
        }
        if(g1.leftBumper()) {
            robot2.extendo.extendoState = Extendo2.ExtendoState.RETRACT;
            return;
        }
        robot2.extendo.extendoState= Extendo2.ExtendoState.IDLE;


    }
    private void slidesUpdate() {
        robot2.slides.updatePower(-g1.left_trigger + g1.right_trigger);
    }
    private void outtakeUpdate() {
        if(robot2.outtake.currentState == Outtake2.FourBarState.PRE_INTAKE) {
            robot2.outtake.currentState = Outtake2.FourBarState.INTAKE_POSITION;
        }
        if(g1.bOnce()) {
            if(robot2.outtake.clawState == Outtake2.ClawState.OPEN) {
                robot2.outtake.clawState = Outtake2.ClawState.CLOSE;
            }else {
                robot2.outtake.clawState = Outtake2.ClawState.OPEN;
            }
        }
        if(g1.yOnce()) {
            if(robot2.outtake.currentState == Outtake2.FourBarState.INTAKE_POSITION) {
                robot2.outtake.lastImportant = robot2.outtake.currentState;
                robot2.outtake.currentState = Outtake2.FourBarState.TRANSFER_IDLE;
            }else if(robot2.outtake.currentState == Outtake2.FourBarState.TRANSFER_IDLE) {
                if(robot2.outtake.lastImportant == Outtake2.FourBarState.INTAKE_POSITION) {
                    robot2.outtake.lastImportant = robot2.outtake.currentState;
                    robot2.outtake.currentState = Outtake2.FourBarState.OUTTAKE_POSITION;
                }else {
                    robot2.outtake.lastImportant = robot2.outtake.currentState;
                    robot2.outtake.currentState = Outtake2.FourBarState.INTAKE_POSITION;
                }
            }else {
                robot2.outtake.lastImportant = robot2.outtake.currentState;
                robot2.outtake.currentState = Outtake2.FourBarState.TRANSFER_IDLE;
            }
        }
    }
    private void updateTelemetry() {
        long finish = System.currentTimeMillis();
        telemetry.addLine("                     INTAKE");
        telemetry.addData("capacPose",robot2.intake.capacPos);
        telemetry.addData("tiltPos",robot2.intake.currentTilt);
        telemetry.addData("indexTilt",robot2.intake.getIndexTilt());
        telemetry.addData("speed Motor",robot2.intake.intakeState);
        telemetry.addData("extendoPower",robot2.extendo.getPower());
        telemetry.addData("extendoState",robot2.extendo.extendoState);
//        telemetry.addData("pixelLeft",robot2.sensors.getLeftDistance());
//        telemetry.addData("pixelRight",robot2.sensors.getRightDistance());
        telemetry.addData("Pixels",Globals.NUM_PIXELS);
        telemetry.addLine("                     OUTTAKE");
        telemetry.addData("OuttakeState",robot2.outtake.currentState);
        telemetry.addData("OuttakeLast",robot2.outtake.lastImportant);
        telemetry.addData("ClawState",robot2.outtake.clawState);
        telemetry.addData("rotatePos",robot2.outtake.currentRotatePos);

        telemetry.addData("SLides state",robot2.slides.slidesState);

        telemetry.addLine("                     MISC");
        telemetry.addData("Runtime",getRuntime());

        telemetry.addData("Sample Rate (Hz) ",1/((double)(finish - lastLoopFinish)/1000.0));
        lastLoopFinish = finish;
        telemetry.update();
    }
}
