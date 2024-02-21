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
                .followTrajectorySequence( drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                                .back(5)
                                .strafeLeft(5)
                                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y,Math.toRadians(ANGLE_SPIKE_LEFT)))
                                .setReversed(true)
                                .splineTo(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y), Math.toRadians(180))
                                .lineTo(new Vector2d(COLLECT_STACK_X, COLLECT_STACK_Y))
                                .setReversed(false)
                                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y))// se da cu spatele
                                .splineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y), Math.toRadians(180))
                                .lineTo(new Vector2d(COLLECT_STACK_X, COLLECT_STACK_Y))
                                .setReversed(false)
                                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y))// se da cu spatele
                                .splineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(PARK_RIGHT_X, PARK_RIGHT_Y,Math.toRadians(ANGLE_PARK_RIGHT)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

    static Pose2d startPose = new Pose2d(10, 62, Math.toRadians(270));
    public static double forwardRight = 6.75;
    public static  double PRELOAD_LEFT_X = 15;
    public static  double PRELOAD_LEFT_Y = 30;
    public static  double PRELOAD_ANGLE_LEFT = -50;

    public static  double PLACE_SPIKE_LEFT_X = 50;
    public static  double PLACE_SPIKE_LEFT_Y = 46;
    public static  double ANGLE_SPIKE_LEFT = 0;

    public static  double PARK_LEFT_X = 43;
    public static  double PARK_LEFT_Y = 20;
    public static  double ANGLE_PARK_LEFT = 0;

    public static  double PRELOAD_MID_X = 12;
    public static  double PRELOAD_MID_Y = 30;
    public static  double PRELOAD_ANGLE_MID = 0;

    public static  double PLACE_SPIKE_MID_X = 50;
    public static  double PLACE_SPIKE_MID_Y = 40;
    public static  double ANGLE_SPIKE_MID = 0;

    public static  double PARK_MID_X = 43;
    public static  double PARK_MID_Y = 20;
    public static  double ANGLE_PARK_MID = 0;

    public static  double PRELOAD_RIGHT_X = 7;
    public static  double PRELOAD_RIGHT_Y = 33;
    public static  double PRELOAD_ANGLE_RIGHT = -55;

    public static  double PLACE_SPIKE_RIGHT_X = 50;
    public static  double PLACE_SPIKE_RIGHT_Y = 32;
    public static double ANGLE_SPIKE_RIGHT = 0;

    public static double PARK_RIGHT_X = 44;
    public static double PARK_RIGHT_Y = 20;
    public static double ANGLE_PARK_RIGHT = 0;

    public static final double GO_TO_STACK_X = 27f;
    public static final double GO_TO_STACK_Y = 8.5;
    public static final double GO_TO_STACK_ANGLE_CYCLE1 = 0.2;
    public static final double GO_TO_STACK_ANGLE_CYCLE2 = 1;
    public static final double GO_TO_STACK_ANGLE_CYCLE3 = 1;
    public static final double COLLECT_STACK_X = -27f;
    public static final double COLLECT_STACK_Y = 8.5f;

    public static final double COLLECT_STACK_X_CYCLE1 = -27.5f;
    public static final double COLLECT_STACK_Y_CYCLE1 = 8.5f;

    public static final double COLLECT_STACK_X_CYCLE2 = -27.2f;
    public static final double COLLECT_STACK_Y_CYCLE2 = 8.5f;

    public static final double COLLECT_STACK_X_CYCLE3 = -26.5f;
    public static final double COLLECT_STACK_Y_CYCLE3 = 8.5f;

    public static final double PLACE_BB_LLH1_X = 20;
    public static final double PLACE_BB_LLH1_Y = 11;
    public static final int ANGLE_BB_LLH1 = 0;
    public static final double PLACE_BB_LLH2_X = 50.5;
    public static final double PLACE_BB_LLH2_Y = 33.5f;
    public static final int ANGLE_BB_LLH2 = 0;
    public static int timeout_1pixel = 3;
    public static int timeout_nopixel = 5;
}  