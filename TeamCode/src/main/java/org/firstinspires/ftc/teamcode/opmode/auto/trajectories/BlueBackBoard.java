package org.firstinspires.ftc.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;

import java.util.ArrayList;

public class BlueBackBoard {
    public static Pose2d start = new Pose2d(11.5,-56.5,Math.toRadians(90));

    public ArrayList<Pose2d> blueBackBoardPreloadPos;
    public ArrayList<Pose2d> yellowPixelPos;

    public ArrayList<Double> headingPreload = new ArrayList<>();
    public ArrayList<Double> headingPixel = new ArrayList<>();
    Trajectory trajectorToRedBackBoardPreload;
    Trajectory trajectorToYellowPixel;
    Trajectory trajectorToParking;

    public Vector2d parkingPose;
    public BlueBackBoard() {
        blueBackBoardPreloadPos = new ArrayList<>();
        yellowPixelPos = new ArrayList<>();
    }

    public void add() {
        parkingPose = new Vector2d(48,-60);
        blueBackBoardPreloadPos.add(new Pose2d(11.5,35,Math.toRadians(180)));//left
        blueBackBoardPreloadPos.add(new Pose2d(11.5,35,Math.toRadians(90)));//center
        blueBackBoardPreloadPos.add(new Pose2d(35,30,Math.toRadians(180)));//right
        yellowPixelPos.add(new Pose2d(48,29,Math.toRadians(180)));
        yellowPixelPos.add(new Pose2d(48,38,Math.toRadians(180)));
        yellowPixelPos.add(new Pose2d(48,44,Math.toRadians(180)));
    }


    public Action getTrajectorToRedBackBoardPreload(MecanumDrive drive, int Case) {
        return drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(blueBackBoardPreloadPos.get(Case).position.x,blueBackBoardPreloadPos.get(Case).position.y),blueBackBoardPreloadPos.get(Case).heading.toDouble())
                    .build();
    }

    public Action getTrajectorToYellowPixel(MecanumDrive drive, int Case) {
        return drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(yellowPixelPos.get(Case).position.x,yellowPixelPos.get(Case).position.y),yellowPixelPos.get(Case).heading.toDouble())
                .build();
    }
    public Action getParking(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .strafeTo(parkingPose)
                .build();
    }
}
