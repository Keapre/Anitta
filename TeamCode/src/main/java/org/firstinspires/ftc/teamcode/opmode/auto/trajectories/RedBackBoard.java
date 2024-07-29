package org.firstinspires.ftc.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.ArrayList;

public class RedBackBoard {
    public ArrayList<Vector2d> redBackBoardPreloadPos;
    public ArrayList<Vector2d> yellowPixelPos;

    public ArrayList<Double> headingPreload = new ArrayList<>();
    public ArrayList<Double> headingPixel = new ArrayList<>();
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
        headingPreload.add(Math.toRadians(90));
        headingPreload.add(Math.toRadians(180));
        headingPreload.add(Math.toRadians(270));

        headingPixel.add(Math.toRadians(-90));
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
}
