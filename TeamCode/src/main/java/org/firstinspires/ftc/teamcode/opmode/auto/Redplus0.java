package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.auto.trajectories.Center0Red;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TeamPropDetectionRed;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red 2+0")
public class Redplus0 extends LinearOpMode {
    public Robot robot;
    Center0Red traj;
    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionRed teamPropDetectionRed;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = -1;

    AutoActionScheduler scheduler;
    ElapsedTime trajectoryTimer = null;
    public static int Case = 1; // 0-left,1-mid,2-right

    void solvePurplePixel() {
        robot.intake.tiltPos = Intake.TiltState.HIGH;
        robot.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;
        //robot.intake.capacPos = Intake.CapacPos.UP; //asta trebuie comentat for before version
        robot.sleep(0.25);
        Action fllw = traj.YellowPixel(robot.drive, Case);
//        scheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                fllw,
//                                new SequentialAction(
//                                        new SleepAction(0.7),
//                                        robot.intake.changeintakeState(Intake.IntakeState.REVERSE_SLOW),
//                                        robot.outtake.changeArmState(Outtake.FourBarState.TRANSFER_AUTO)
//                                )
//                        ),
//                        new SleepAction(0.3)
//                )
//        );
        scheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                fllw,
                                new SequentialAction(
                                        new SleepAction(0.8),
                                        robot.outtake.changeArmState(Outtake.FourBarState.TRANSFER_AUTO)
                                )
                        ),
                        new SleepAction(0.3),
                        robot.intake.changeintakeState(Intake.IntakeState.REVERSE_SLOW)
                )
        );
        scheduler.run();
        robot.sleep(0.3);
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
        Action fllw1 = traj.toBackBoard(robot.drive, Case);
//        trajectoryTimer.reset();
        scheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                robot.intake.changeintakeState(Intake.IntakeState.IDLE),
                                fllw1,
                                new SleepAction(0.2)
                        )
                )

        );
        scheduler.run();
        //

        Action back = traj.goBackAbit(robot.drive);
        robot.outtake.clawState = Outtake.ClawState.CLOSE;
        scheduler.addAction(new SequentialAction(
                new SleepAction(0.4),
                back
        ));

        scheduler.run();
        Action fllw2 = traj.getParking(robot.drive);
        scheduler.addAction(new ParallelAction(
                fllw2,
                new SleepAction(0.4),
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

    //    int cameraTeamProp() {
//        int readFromCamera = noDetectionFlag;
//
//        teamPropDetectionRed = new TeamPropDetectionRed();
//
//        telemetry.addData("Webcam 1", "Initing");
//        telemetry.update();
//
//        visionPortalTeamProp = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(1920, 1080))
//                .addProcessor(teamPropDetectionRed)
//                .enableLiveView(true)
//                .build();
//
//        telemetry.setMsTransmissionInterval(50);
//
//        if (visionPortalTeamProp.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Webcam 1", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortalTeamProp.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                telemetry.addData("Webcam 1", "Waiting");
//                telemetry.addData("State", visionPortalTeamProp.getCameraState());
//                telemetry.update();
//                sleep(50);
//            }
//            telemetry.addData("Webcam 1", "Ready");
//            telemetry.update();
//        }
//        if (isStopRequested()) {
//            robot.stop();
//            return robotStopFlag;
//        }
//
//        while (!isStarted()) {
//            readFromCamera = teamPropDetectionRed.getTeamProp();
//            telemetry.addData("Case", readFromCamera);
//            telemetry.addData("left", teamPropDetectionRed.leftValue());
//            telemetry.addData("cent", teamPropDetectionRed.centreValue());
//            telemetry.addData("thresh", teamPropDetectionRed.threshold);
//            telemetry.update();
//        }
//
//        visionPortalTeamProp.stopStreaming();
//
//        return readFromCamera;
//    }
    void intakeOuttakeMovement() {
        robot.outtake.currentState = Outtake.FourBarState.TRANSFER_IDLE;
        robot.sleep(0.3);
        robot.outtake.currentState = Outtake.FourBarState.TRANSFER_AUTO;
        robot.sleep(0.5);
        //robot.slides.slidesState = Slides.SlidesState.FIRST_THRESHOLD;
        robot.sleep(0.2);

    }

    public void getLocation() {
        Case = robot.sensors.hky.getLocation(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        traj = new Center0Red();
        Globals.RUNMODE = Perioada.AUTO;
        Globals.startPose = traj.start;
        Globals.isRed = true;
        robot = new Robot(this, true);

        scheduler = new AutoActionScheduler();

        trajectoryTimer = new ElapsedTime();

        telemetry.addData("is busy", robot.drive.isBusy());
        telemetry.update();
        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        robot.start();
        if (isStopRequested()) {
            robot.stop();
        }
        getLocation();
        solvePurplePixel();
    }
}