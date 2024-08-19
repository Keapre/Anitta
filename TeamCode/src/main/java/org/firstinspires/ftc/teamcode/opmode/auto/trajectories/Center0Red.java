package org.firstinspires.ftc.teamcode.opmode.auto.trajectories;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
public class Center0Red {

    public  Pose2d start = new Pose2d(12,-63,Math.toRadians(90));
    public  Pose2d currentPose = new Pose2d(15,-63,Math.toRadians(90));

    public static Pose2d START_POSE = new Pose2d(-37, -65, Math.toRadians(270));


    public static Vector2d Lane = new Vector2d(23,-60);
    public static Vector2d followLane = new Vector2d(-24,-60);
    public static Pose2d stackPosition = new Pose2d(-59,-40,Math.toRadians(160));


    Trajectory trajectorToRedBackBoardPreload;
    Trajectory trajectorToYellowPixel;
    Trajectory trajectorToParking;

    public Pose2d parkingPose = new Pose2d(46,-60,Math.toRadians(180));
    public Center0Red() {
    }

    public Action goBackAbit(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(drive.pose.position.x-3.7,drive.pose.position.y))
                .build();
    }


    public Action toBackBoard(MecanumDrive drive,int Case) {
        if(Case == 1) {
            return drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(51, -36), Math.toRadians(180))
                    .build();
        }
        else if(Case == 0) {
            return drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(50, -26.2))
                    .build();
        }
        return  drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(49.3 ,-43))
                 .build();
    }

    public Action YellowPixel(MecanumDrive drive, int Case) {
        Action action;
        if (Case == 1) {
            return drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(12, -35))
                    .build();
        }
        if(Case == 0) {
            return  drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(8, -32),Math.toRadians(180))
                    .build();

        }

        return   drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(33.5, -32),Math.toRadians(180))
                    .build();
    }
    public Action getCycle(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineTo(Lane,Math.toRadians(180))
                .splineToConstantHeading(followLane,Math.toRadians(180))
                .splineToLinearHeading(stackPosition,Math.toRadians(90))
                .build();

    }
    public Action goToLane(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineTo(Lane,Math.toRadians(180))
                .build();
    }
    public Action followLane(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .splineToConstantHeading(followLane,Math.toRadians(180))
                .build();
    }

    public Action goToStack(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .splineToSplineHeading(stackPosition,Math.toRadians(90))
                .build();
    }
    public Action getParking(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(parkingPose.position.x,parkingPose.position.y),parkingPose.heading.toDouble())
                .build();
    }
}
