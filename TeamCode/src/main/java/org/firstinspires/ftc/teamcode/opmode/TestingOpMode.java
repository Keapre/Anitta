package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;

@Config
@TeleOp
public class TestingOpMode extends LinearOpMode {
    Robot2 robot2;
    GamePadController g1;
    @Override
    public void runOpMode() throws InterruptedException {
        robot2 = new Robot2(hardwareMap);
        g1 = new GamePadController(gamepad1);
        Globals.RUNMODE = Perioada.TELEOP;
        waitForStart();

        while(opModeIsActive()) {
            g1.update();

            intakeUpdate();
            extendoUpdate();
            robot2.update();
        }
    }

    private void intakeUpdate() {
        if(g1.aOnce()) {
            if(robot2.intake.intakeState == Intake.IntakeState.IDLE) {
                robot2.intake.intakeState = Intake.IntakeState.FORWARD;
            }else {
                robot2.intake.intakeState = Intake.IntakeState.IDLE;
            }
        }
        if(g1.yOnce()) {
            if(robot2.intake.tiltPos !=Intake.TiltState.HIGH) {
                robot2.intake.tiltPos = Intake.TiltState.HIGH;
            }else {
                robot2.intake.tiltPos = Intake.TiltState.LOW;
            }
        }

        if(g1.dpadUpOnce()) robot2.intake.addTilt();
        if(g1.dpadDownOnce()) robot2.intake.substractTilt();

        robot2.intake.checkForPixels(g1);
    }
    private void extendoUpdate() {
        robot2.extendo.updatePower(-g1.left_trigger + g1.right_trigger);
    }
}
