//package org.firstinspires.ftc.teamcode.opmode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//import org.firstinspires.ftc.teamcode.Robot3;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo2;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake2;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake2;
//import org.firstinspires.ftc.teamcode.util.GamePadController;
//import org.firstinspires.ftc.teamcode.util.Globals;
//import org.firstinspires.ftc.teamcode.util.Perioada;
//
//@Config
//@TeleOp(name = "TeleOp single")
//public class TestingOpModeSingle extends OpMode {
//    Robot3 robot2;
//    GamePadController g1;
//    private long lastLoopFinish = 0;
//    public static boolean usePoluu = false;
//    MultipleTelemetry telemetry;
//
//
//    private void intakeUpdate() {
//
//        if(g1.xOnce()) { //guide
//            if(robot2.intake.capacPos == Intake2.CapacPos.DOWN) {
//                robot2.intake.capacPos = Intake2.CapacPos.UP;
//            }else {
//                robot2.intake.capacPos = Intake2.CapacPos.DOWN;
//            }
//        }
//
//        robot2.intake.updatePower(g1);
//
//        if(usePoluu) robot2.intake.checkForPixels(g1);
//    }
//
//    private void extendoUpdate() {
//        if(robot2.outtake.currentState == Outtake2.FourBarState.INTAKE_POSITION) {
//            robot2.extendo.extendoState = Extendo2.ExtendoState.RETRACT_INTAKE;
//            return;
//        }
//
//        if(g1.rightTrigger()) {
//            robot2.extendo.extendoState = Extendo2.ExtendoState.EXTEND;
//            return;
//        }
//        if(g1.leftTrigger()) {
//            robot2.extendo.extendoState = Extendo2.ExtendoState.RETRACT;
//            return;
//        }
//        robot2.extendo.extendoState= Extendo2.ExtendoState.IDLE;
//
//
//    }
//    private void slidesUpdate() {
//        double power = -g1.left_trigger + g1.right_trigger;
//        if(power == 0) power = 0.05;
//        robot2.slides.updatePower(power);
//    }
//    private void outtakeUpdate() {
//        if(robot2.outtake.currentState == Outtake2.FourBarState.TRANSFER_IDLE) {
//            if(Globals.NUM_PIXELS == 2 && usePoluu) {
//                robot2.intake.capacPos = Intake2.CapacPos.UP;
//            }
//            robot2.drive.slow_mode = false;
//        }
//        if(robot2.outtake.currentState == Outtake2.FourBarState.OUTTAKE_POSITION) {
//            robot2.intake.capacPos = Intake2.CapacPos.DOWN;
//        }
//        if(g1.bOnce()) {
//            if(robot2.outtake.clawState == Outtake2.ClawState.OPEN) {
//                robot2.outtake.clawState = Outtake2.ClawState.CLOSE;
//            }else {
//                robot2.outtake.clawState = Outtake2.ClawState.OPEN;
//            }
//        }
//        if(g1.yOnce()) {
//            if(robot2.outtake.currentState != Outtake2.FourBarState.TRANSFER_IDLE) {
//                robot2.outtake.lastImportant = robot2.outtake.currentState;
//                robot2.outtake.currentState = Outtake2.FourBarState.TRANSFER_IDLE;
//                robot2.drive.slow_mode = false;
//                robot2.outtake.clawState = Outtake2.ClawState.OPEN;
//                //robot2.intake.capacPos = Intake2.CapacPos.DOWN; //might not work
//
//            }else {
//                if(robot2.outtake.lastImportant == Outtake2.FourBarState.INTAKE_POSITION) {
//                    robot2.outtake.lastImportant = robot2.outtake.currentState;
//                    robot2.drive.slow_mode = true;
//                    robot2.outtake.currentState = Outtake2.FourBarState.OUTTAKE_POSITION;
//                }else {
//                    robot2.outtake.lastImportant = robot2.outtake.currentState;
//                    robot2.outtake.clawState = Outtake2.ClawState.OPEN;
//                    robot2.intake.capacPos = Intake2.CapacPos.UP;
//                    robot2.drive.slow_mode = false;
//                    robot2.outtake.currentState = Outtake2.FourBarState.INTAKE_POSITION;
//                }
//            }
//        }
//        if(g1.dpadLeftOnce()) {
//            robot2.outtake.addRotate();
//        }
//        if(g1.dpadRightOnce()){
//            robot2.outtake.subRotate();
//        }
//    }
//    private void updateTelemetry() {
//        long finish = System.currentTimeMillis();
//        telemetry.addLine("                     INTAKE");
//        telemetry.addData("capacPose",robot2.intake.capacPos);
//        telemetry.addData("tiltPos",robot2.intake.currentTilt);
//        telemetry.addData("indexTilt",robot2.intake.getIndexTilt());
//        telemetry.addData("speed Motor",robot2.intake.intakeState);
//        telemetry.addData("extendoPower",robot2.extendo.getPower());
//        telemetry.addData("extendoState",robot2.extendo.extendoState);
//        telemetry.addData("pixelLeft",robot2.sensors.getLeftDistance());
//        telemetry.addData("pixelRight",robot2.sensors.getRightDistance());
//        telemetry.addData("Pixels",Globals.NUM_PIXELS);
//        telemetry.addLine("                     OUTTAKE");
//        telemetry.addData("OuttakeState",robot2.outtake.currentState);
//        telemetry.addData("OuttakeLast",robot2.outtake.lastImportant);
//        telemetry.addData("ClawState",robot2.outtake.clawState);
//        telemetry.addData("rotatePos",robot2.outtake.currentRotatePos);
//
//        telemetry.addData("SLides state",robot2.slides.slidesState);
//
//        telemetry.addLine("                     MISC");
//        telemetry.addData("Runtime",getRuntime());
//
//        telemetry.addData("Sample Rate (Hz) ",1/((double)(finish - lastLoopFinish)/1000.0));
//        lastLoopFinish = finish;
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        Globals.RUNMODE = Perioada.TELEOP;
//        robot2= new Robot3(this,false);
//        g1 = new GamePadController(gamepad1);
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
//    }
//
//    @Override
//    public void start() {
//        robot2.start();
//    }
//    @Override
//    public void loop() {
//        g1.update();
//        intakeUpdate();
//        extendoUpdate();
//        slidesUpdate();
//        outtakeUpdate();
//        robot2.drive.driveFromController(g1);
//        updateTelemetry();
//    }
//}
