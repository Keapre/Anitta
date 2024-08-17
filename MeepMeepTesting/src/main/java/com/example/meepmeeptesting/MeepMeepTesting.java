package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.sun.org.apache.xpath.internal.compiler.PsuedoNames;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import javax.swing.ActionMap;

public class MeepMeepTesting {
    //CENTER CONE RED BACKBOARD SIDE
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d start = new Pose2d(15, -63, Math.toRadians(90));
        Vector2d Lane = new Vector2d(23,-60);
        Vector2d followLane = new Vector2d(-24,-60);
        Pose2d stackPosition = new Pose2d(-56,-41,Math.toRadians(150));
        Vector2d rightPrompt = new Vector2d(11.5,-35);
        Vector2d rightBack = new Vector2d(48,-35);
        List<Action> trajectories = new ArrayList<>();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        Action scorePreload = myBot.getDrive().actionBuilder(stackPosition)
                .splineToLinearHeading(new Pose2d(followLane,Math.toRadians(180)),Math.toRadians(180))
                .splineToConstantHeading(Lane,Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(start.position.x,start.position.y),Math.toRadians(180))



                .build();
//        Action goToPixels1 = myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(44,-45),Math.toRadians(180)))
//                .setTangent(Math.toRadians(180))
//                .splineTo(new Vector2d(23,-60),Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-24,-60),Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-56,-41,Math.toRadians(150)),Math.toRadians(90))
//                .waitSeconds(1)
//                .build();
        myBot.runAction(new SequentialAction(
                scorePreload
        ));
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}