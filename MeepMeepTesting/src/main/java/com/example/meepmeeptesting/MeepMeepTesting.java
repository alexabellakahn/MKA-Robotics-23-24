package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-35.25, 62.7, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(-40.25, 35.7, Math.toRadians(180)), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-33.75, 29.2), Math.toRadians(0))
//                        .strafeTo(new Vector2d(-33.7, 16.2))
                                .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-30.25, 9.7), Math.toRadians(0))
                        //9.7
                                .setReversed(false)
                        .lineToLinearHeading(new Pose2d(38.75, 9.7, Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(56.25, 38.2))
                        .lineToConstantHeading(new Vector2d(51.25, 10.2))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}