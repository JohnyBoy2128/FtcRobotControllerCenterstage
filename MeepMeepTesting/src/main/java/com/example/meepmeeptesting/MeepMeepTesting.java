package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.04, -60.18, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-39.44, -25.03), Math.toRadians(90.00))
                                .lineToSplineHeading(new Pose2d(-54.00, -36.00, Math.toRadians(90.00)))
                                .lineTo(new Vector2d(-54.00, -12.00))
                                .lineToConstantHeading(new Vector2d(24.00, -12.00))
                                .lineToLinearHeading(new Pose2d(50.13, -36.20, Math.toRadians(0.00)))
                                .lineToLinearHeading(new Pose2d(45, -36.20, Math.toRadians(0.00)))
                                .turn(Math.toRadians(90))
                                .forward(24)
                                .strafeRight(15)
                                .build());

























        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}