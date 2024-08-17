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
public class BlueBackBoard {

    public  Pose2d start = new Pose2d(12,63,-Math.toRadians(90));
    public  Pose2d currentPose = new Pose2d(15,-63,Math.toRadians(90));

    public static Pose2d START_POSE = new Pose2d(-37, -65, Math.toRadians(270));


    public static Vector2d Lane = new Vector2d(23,-60);
    public static Vector2d followLane = new Vector2d(-24,-60);
    public static Pose2d stackPosition = new Pose2d(-56,-41,Math.toRadians(150));


    Trajectory trajectorToRedBackBoardPreload;
    Trajectory trajectorToYellowPixel;
    Trajectory trajectorToParking;

    public Vector2d parkingPose = new Vector2d(46,60);
    public BlueBackBoard() {
    }

    public Action goBackAbit(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(drive.pose.position.x-3.7,drive.pose.position.y))
                .build();
    }


    public Action toBackBoard(MecanumDrive drive,int Case) {
        if(Case == 1) {
            return drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(52.7 , 37), -Math.toRadians(180))
                    .build();
        }
        else if(Case == 0) {
            return drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(52.7, 30.5))
                    .build();
        }
        return  drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(52.7,42.5))
                .build();
    }

    public Action YellowPixel(MecanumDrive drive, int Case) {
        Action action;
        if (Case == 1) {
            return drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(13, 35))
                    .build();
        }
        if(Case == 0) {
            return  drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(9.0, 30.6),-Math.toRadians(180))
                    .build();

        }

        return   drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(32, 29),-Math.toRadians(180))
                .build();
    }
    public Action getCycle(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineTo(Lane,Math.toRadians(180))
                .splineToConstantHeading(followLane,Math.toRadians(180))
                .waitSeconds(0.2)
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
                .strafeTo(parkingPose)
                .build();
    }
}
