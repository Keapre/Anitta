package org.firstinspires.ftc.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;

import java.util.ArrayList;

public class RedBackBoard {
    public static Pose2d start = new Pose2d(11.5,-56.5,Math.toRadians(90));

    public ArrayList<Vector2d> redBackBoardPreloadPos;
    public ArrayList<Vector2d> yellowPixelPos;

    public ArrayList<Double> fromBackboardtoStack = new ArrayList<>();
    public ArrayList<Double> fromStacktoBoardPos = new ArrayList<>();

    Trajectory trajectorToRedBackBoardPreload;
    Trajectory trajectorToYellowPixel;
    Trajectory trajectorToParking;

    public Vector2d parkingPose;
    public RedBackBoard() {
        redBackBoardPreloadPos = new ArrayList<>();
        yellowPixelPos = new ArrayList<>();
    }

    public void add() {
        parkingPose = new Vector2d(48,-60);
        redBackBoardPreloadPos.add(new Vector2d(11.5,-35));
        redBackBoardPreloadPos.add(new Vector2d(11.5,-35));
        redBackBoardPreloadPos.add(new Vector2d(35,-30));
        yellowPixelPos.add(new Vector2d(48,-29));
        yellowPixelPos.add(new Vector2d(48,-38));
        yellowPixelPos.add(new Vector2d(48,-44));
    }


    public Action getTrajectorToRedBackBoardPreload(MecanumDrive drive, int Case) {
        Action trajectory = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(redBackBoardPreloadPos.get(Case), headingPreload.get(Case))
                .build();
        return trajectory;
    }

    public Action getTrajectorToYellowPixel(MecanumDrive drive, int Case) {
        return drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(yellowPixelPos.get(Case), headingPixel.get(0))
                .build();
    }
    public Action getParking(MecanumDrive drive) {
        return drive.actionBuilder(drive.pose)
                .strafeTo(parkingPose)
                .build();
    }
}
