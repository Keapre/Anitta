package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.trajectories.RedBackBoard;

public class Red_Far extends AutoBase {

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

    }

    void park() {

    }

}
