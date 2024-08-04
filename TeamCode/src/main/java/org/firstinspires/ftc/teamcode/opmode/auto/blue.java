//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//
//import org.firstinspires.ftc.teamcode.opmode.auto.trajectories.BlueBackBoard;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
//
//public class blue extends AutoBase{
//
//
//    BlueBackBoard blue;
//    public static Pose2d start = BlueBackBoard.start;
//    @Override
//    protected Pose2d getStartPose() {
//        return start;
//    }
//
//    @Override
//    protected void printDescription() {
//        telemetry.addData("Description","Sa ti o trag de 0 in albastru");
//    }
//
//    @Override
//    protected void onRun() {
//        scoreTeamProp();
//        scorePreload();
//        park();
//    }
//
//    void scoreTeamProp() {
//        Action toDrive = blue.getTrajectorToYellowPixel(robot.drivetrain,spike);
//
//        sched.addAction(new SequentialAction(
//                toDrive,
//                robot.intake.changeintakeState(Intake.IntakeState.REVERSE_FOR_TIME)
//        ));
//
//        sched.run();
//    }
//
//    void scorePreload() {
//        Action fllw = blue.getTrajectorToRedBackBoardPreload(robot.drivetrain,spike);
//        sched.addAction(robot.outtake.changeClawState(Outtake.ClawState.CLOSE));
//        sched.addAction(new ParallelAction(
//                fllw,
//                robot.outtake.changeArmState(Outtake.FourBarState.OUTTAKE_POSITION)
//        ));
//        sched.addAction(
//                robot.outtake.changeClawState(Outtake.ClawState.OPEN)
//        );
//        sched.run();
//    }
//
//    void park() {
//        Action fllw = blue.getParking(robot.drivetrain);
//
//        sched.addAction(fllw);
//        sched.run();
//    }
//}
