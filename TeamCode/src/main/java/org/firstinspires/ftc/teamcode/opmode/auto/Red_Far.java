package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.trajectories.RedBackBoard;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;

public class Red_Far extends AutoBase {

    RedBackBoard inf;
    public static Pose2d start = RedBackBoard.start;

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description","Sa ti o trag de 0 in rosu");
    }


    @Override
    protected void onRun() {
        scoreTeamProp();
        scorePreload();
        park();

    }

    void scoreTeamProp() {
        Action toDrive = inf.getTrajectorToYellowPixel(robot.drivetrain,spike);

        sched.addAction(new SequentialAction(
                toDrive,
                robot.intake.changeintakeState(Intake.IntakeState.REVERSE_FOR_TIME)
        ));

        sched.run();
    }

    void scorePreload() {
        Action fllw = inf.getTrajectorToRedBackBoardPreload(robot.drivetrain,spike);
        sched.addAction(robot.outtake.changeClawState(Outtake.ClawState.CLOSE));
        sched.addAction(new ParallelAction(
                fllw,
                robot.outtake.changeArmState(Outtake.FourBarState.OUTTAKE_POSITION)
        ));
        sched.addAction(
                robot.outtake.changeClawState(Outtake.ClawState.OPEN)
        );
        sched.run();
    }
    void park() {
        Action fllw = inf.getParking(robot.drivetrain);

        sched.addAction(fllw);
        sched.run();
    }

}
