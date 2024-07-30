package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.trajectories.RedBackBoard;

public class Red_Far extends AutoBase {
    RedBackBoard trajector;

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
        scorePreload();
        park();
    }

    void scorePreload() {
        Action actionToYellow = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                .strafeToLinearHeading(trajector.redBackBoardPreloadPos.get(spike), trajector.headingPreload.get(spike))
                .afterDisp(1,()->{
                    robot.intake.reverseForTime(0.6);
                })
                .build();

        sched.addAction(actionToYellow);
        sched.run();
    }

    void park() {

    }

}
