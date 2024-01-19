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
                .setConstraints(60, 50, 5.7, 5.7, 13.22)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(30,-35,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(47,-40,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(43,-20,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity myBotCycle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.7, 5.7, 13.22)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(50, -27, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(20,-11,Math.toRadians(180)))
                                .lineTo(new Vector2d(-5,-11))
                                .lineToLinearHeading(new Pose2d(20,-11,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -27, Math.toRadians(180)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotCycle)
                .start();
    }
}