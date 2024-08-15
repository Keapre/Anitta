package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.auto.trajectories.Center0Red;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;

@Config
@Autonomous(name = "Red 2+0")
public class Redplus2 extends LinearOpMode {
    public Robot robot;
    Center0Red traj;


    AutoActionScheduler scheduler;
    ElapsedTime trajectoryTimer = null;
    public static int Case = 2; // 0-left,1-mid,2-right
    void solvePurplePixel() {
        robot.intake.tiltPos = Intake.TiltState.HIGH;
        robot.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;

        //robot.sleep(0.4);
        Action fllw = traj.YellowPixel(robot.drive,Case);
        scheduler.addAction(
                new SequentialAction(
                        fllw,
                        robot.intake.changeintakeState(Intake.IntakeState.REVERSE_SLOW),
                        robot.outtake.changeArmState(Outtake.FourBarState.TRANSFER_AUTO)
                )

        );
        scheduler.run();
        robot.sleep(0.5);
//        trajectoryTimer.reset();
//
//        while(robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//            System.out.println(robot.drive.isBusy());
//            if(50 < trajectoryTimer.milliseconds() && trajectoryTimer.milliseconds() < 150) {
//                robot.intake.tiltPos = Intake.TiltState.HIGH;
//                robot.intake.intakeState = Intake.IntakeState.REVERSE_SLOW;
//            }
//            robot.sleep(0.01);
//        }
        Action fllw1 = traj.toBackBoard(robot.drive,Case);
//        trajectoryTimer.reset();
        scheduler.addAction(
                new SequentialAction(
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.intake.changeintakeState(Intake.IntakeState.IDLE),
                                fllw1,
                                new SleepAction(0.3)
                        )
                )

        );
        scheduler.run();
        //
        Action fllw2 = traj.getParking(robot.drive);
        robot.outtake.clawState = Outtake.ClawState.CLOSE;
        robot.sleep(0.5);
        scheduler.addAction(new ParallelAction(
                fllw2,
                new SleepAction(0.5),
                robot.outtake.changeArmState(Outtake.FourBarState.TRANSFER_IDLE)
        ));

        scheduler.run();
//        boolean done = false;
//        while(robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//            if(trajectoryTimer.milliseconds() < 500 && !done) {
//                robot.outtake.currentState = Outtake.FourBarState.TRANSFER_AUTO;
//                done = true;
//            }
//            robot.sleep(0.01);
//        }

    }

    void intakeOuttakeMovement() {
        robot.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;
        robot.sleep(0.3);
        robot.outtake.currentState = Outtake.FourBarState.TRANSFER_AUTO;
        robot.sleep(0.5);
        //robot.slides.slidesState = Slides.SlidesState.FIRST_THRESHOLD;
        robot.sleep(0.2);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        traj = new Center0Red();
        Globals.RUNMODE = Perioada.AUTO;
        Globals.startPose = traj.start;
        Globals.isRed = true;
        robot = new Robot(this,true);


        scheduler = new AutoActionScheduler();
        trajectoryTimer = new ElapsedTime();
        robot.start();
        telemetry.addData("is busy",robot.drive.isBusy());
        telemetry.update();
        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        if (isStopRequested()) {
            robot.stop();
        }
        solvePurplePixel();
    }
}