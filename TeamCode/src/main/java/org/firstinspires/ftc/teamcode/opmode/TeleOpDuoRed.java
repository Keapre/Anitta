package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;

@Config
@TeleOp(name = "Duo TeleOp Red")
public class TeleOpDuoRed extends OpMode {
    Robot robot2;
    GamePadController g1,g2;
    //g1 - drivetrain,extendo,intake

    private long lastLoopFinish = 0;
    public static boolean usePoluu = false;
    public static boolean hang = false;
    MultipleTelemetry telemetry;
//    DcMotorEx sMotor1,sMotor2;
//

    private void hangUpdate() {

        if(g1.backOnce()) {
            if(!hang) {
                hang = true;
                robot2.drive.hangMode();
                robot2.outtake.hangMode();
                robot2.extendo.hangMode();
                robot2.intake.hangMode();
                robot2.plane.hangMode();
            }
        }
        telemetry.addData("hang",hang);
    }
    private void intakeUpdate() {
        robot2.intake.updatePower(g1);


        if(robot2.intake.intakeSpeed == 1) {
            robot2.intake.capacPos = Intake.CapacPos.DOWN;
        }
        if (g1.aOnce()) {
            if (robot2.intake.tiltPos == Intake.TiltState.LOW) {
                robot2.intake.tiltPos = Intake.TiltState.HIGH;
            } else {
                robot2.intake.tiltPos = Intake.TiltState.LOW;
            }


            if (usePoluu) robot2.intake.checkForPixels(g1);
        }
    }

    private void extendoUpdate() {
        double power = -g1.left_trigger + g1.right_trigger;
        robot2.extendo.updatePower(power);
    }
    private void slidesUpdate() {
        if(gamepad2.right_trigger > 0.1) {
            robot2.slides.updatePower(gamepad2.right_trigger);
        }else if(gamepad2.left_trigger > 0.1) {
            robot2.slides.updatePower(-gamepad2.left_trigger);
        }else {
            robot2.slides.updatePower(robot2.slides.idlePower);
        }
    }
    private void outtakeUpdate() {
        robot2.drive.slow_mode = (robot2.outtake.currentState == Outtake.FourBarState.OUTTAKE_POSITION);
//        if(robot2.outtake.currentState == Outtake2.FourBarState.OUTTAKE_POSITION) {
//            robot2.intake.capacPos = Intake2.CapacPos.DOWN;
//        }

        if(g2.leftBumperOnce()) {
            if(robot2.outtake.currentState == Outtake.FourBarState.TRANSFER_IDLE) robot2.outtake.currentState = Outtake.FourBarState.OUTTAKE_POSITION;
            else {
                robot2.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;
            }
        }
        if(g2.rightBumperOnce()) {
            if(robot2.outtake.currentState == Outtake.FourBarState.TRANSFER_IDLE) robot2.outtake.currentState = Outtake.FourBarState.INTAKE_POSITION;
            else {
                robot2.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;
            }
        }
        if(robot2.outtake.currentState == Outtake.FourBarState.TRANSFER_IDLE && robot2.outtake.clawState == Outtake.ClawState.OPEN) {
            g2.rumble(150);
        }
        if(g2.bOnce()) {
            if(robot2.outtake.clawState == Outtake.ClawState.CLOSE) {
                robot2.outtake.clawState = Outtake.ClawState.OPEN;
            }else if(robot2.outtake.rotateState != Outtake.ROTATESTATE.LEFT90 && robot2.outtake.rotateState!= Outtake.ROTATESTATE.RIGHT90){
                robot2.outtake.clawState = Outtake.ClawState.CLOSE;
            }else if(robot2.outtake.rotateState == Outtake.ROTATESTATE.LEFT90) {
                if(robot2.outtake.clawState == Outtake.ClawState.OPEN) {
                    robot2.outtake.clawState = Outtake.ClawState.LEFT_CLOSE;
                }else if(robot2.outtake.clawState == Outtake.ClawState.LEFT_CLOSE) {
                    robot2.outtake.clawState = Outtake.ClawState.CLOSE;
                }
            }else {
                if(robot2.outtake.clawState == Outtake.ClawState.OPEN) {
                    robot2.outtake.clawState = Outtake.ClawState.RIGHT_CLOSE;
                }else if(robot2.outtake.clawState == Outtake.ClawState.RIGHT_CLOSE) {
                    robot2.outtake.clawState = Outtake.ClawState.CLOSE;
                }
            }
        }
        if(g2.yOnce()) {
            if(robot2.outtake.currentState == Outtake.FourBarState.TRANSFER_IDLE) {
                robot2.outtake.currentState = Outtake.FourBarState.TRANSFER_AUTO;
            }else {
                robot2.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;
            }
        }

        if(g2.dpadLeftOnce()) {
            robot2.outtake.addRotate();
        }
        if(g2.dpadRightOnce()){
            robot2.outtake.subRotate();
        }
    }

    public void planeUpdate() {
        if(g1.backOnce()) {
            robot2.plane.setOut();
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
        telemetry.addData("pixelLeft",robot2.sensors.getLeftDistance());
        telemetry.addData("pixelRight",robot2.sensors.getRightDistance());
        telemetry.addData("Encoder Extendo",robot2.extendo.getEnoder());
        //telemetry.addData("Slide Pos",robot2.slides.getEncoder());
        telemetry.addData("Pixels",Globals.NUM_PIXELS);
        telemetry.addLine("                     OUTTAKE");
        telemetry.addData("OuttakeState",robot2.outtake.currentState);
        telemetry.addData("OuttakeLast",robot2.outtake.lastImportant);
        telemetry.addData("ClawState",robot2.outtake.clawState);
        telemetry.addData("rotatePos",robot2.outtake.rotateState);

        telemetry.addData("SLides state",robot2.slides.slidesState);

        telemetry.addLine("                     MISC");
        telemetry.addData("Runtime",getRuntime());

        telemetry.addData("Sample Rate (Hz) ",1/((double)(finish - lastLoopFinish)/1000.0));
        lastLoopFinish = finish;
        telemetry.update();
    }

    @Override
    public void init() {
        Globals.RUNMODE = Perioada.TELEOP;
        Globals.isRed = true;
        robot2= new Robot(this,false);
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void updateDrive() {
        //if(robot2.drive.pose.position.x >=20 && robot2.drive.pose.position.y < 0) robot2.Wdrive.SLOW_MODE = true;
        //robot2.Wdrive.driveFromController(g1);
    }
    @Override
    public void start() {
        robot2.start();
    }
    @Override
    public void loop() {
        g1.update();
        g2.update();
        hangUpdate();
        intakeUpdate();
        planeUpdate();
        robot2.Wdrive.driveFromController(g1);
        extendoUpdate();
        slidesUpdate();
        outtakeUpdate();
        updateTelemetry();
    }
}
