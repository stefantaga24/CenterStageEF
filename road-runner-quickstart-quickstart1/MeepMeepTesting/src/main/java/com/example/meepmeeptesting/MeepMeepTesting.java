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
                                .lineToSplineHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(FIRST_PIXEL_X,FIRST_PIXEL_Y,Math.toRadians(0)),Math.toRadians(180))
                                .back(5)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

    static Pose2d startPose = new Pose2d(-38, 62, Math.toRadians(270));
    public static double forwardRight = 6.75;

    public static  double PRELOAD_LEFT_X = -31;
    public static  double PRELOAD_LEFT_Y = 38;
    public static  double PRELOAD_ANGLE_LEFT = -45;

    public static  double PLACE_SPIKE_LEFT_X = 48;
    public static  double PLACE_SPIKE_LEFT_Y = 46;
    public static  double ANGLE_SPIKE_LEFT = 0;

    public static  double PARK_LEFT_X = 43;
    public static  double PARK_LEFT_Y = 20;
    public static  double ANGLE_PARK_LEFT = 0;

    public static  double PRELOAD_MID_X = 38;
    public static  double PRELOAD_MID_Y = 37;
    public static  double PRELOAD_ANGLE_MID = 270;

    public static  double PLACE_SPIKE_MID_X = 48;
    public static  double PLACE_SPIKE_MID_Y = 39;
    public static  double ANGLE_SPIKE_MID = 0;

    public static  double PARK_MID_X = 43;

    public static  double PARK_MID_Y = 20;
    public static  double ANGLE_PARK_MID = 0;

    public static  double PRELOAD_RIGHT_X = 36;
    public static  double PRELOAD_RIGHT_Y = 40;
    public static  double PRELOAD_ANGLE_RIGHT = -145;

    public static  double PLACE_SPIKE_RIGHT_X = 48;
    public static  double PLACE_SPIKE_RIGHT_Y = 32;
    public static double ANGLE_SPIKE_RIGHT = 0;

    public static double PARK_RIGHT_X = 43;
    public static double PARK_RIGHT_Y = 62;
    public static double ANGLE_PARK_RIGHT = 0;

    public static final double FIRST_PIXEL_X = -53;
    public static final double FIRST_PIXEL_Y = 12.5;

    public static final double GO_TO_STACK_X = 20f;
    public static final double GO_TO_STACK_Y = 57.5f;
    public static final double LINE_TO_STACK_X = -30f;
    public static final double LINE_TO_STACK_Y = 57.5f;
    public static final double GO_TO_BACK_1_X = -55f;
    public static final double GO_TO_BACK_1_Y =55.5f;
    public static final double GO_TO_BACK_2_X = 10f;
    public static final double GO_TO_BACK_2_Y =55.5f;
    public static final double ANGLE_COLECTARE=40f;
    public static final double COLLECT_STACK_X_CYCLE1_C2 = -45f;
    public static final double COLLECT_STACK_Y_CYCLE1_C2 = 50f;


    public static final double COLLECT_STACK_X_CYCLE2_C2 = -28f;

    public static final double GO_TO_STACK_ANGLE_CYCLE1_C1 = 0;
    public static final double GO_TO_STACK_ANGLE_CYCLE2_C1= 0;
    public static final double GO_TO_STACK_ANGLE_CYCLE3_C1 = 0;
    public static final double COLLECT_STACK_X_CYCLE1_C1 = -27.5f;
    public static final double COLLECT_STACK_Y_CYCLE1_C1 = 9.5f;
    public static final double COLLECT_STACK_X_CYCLE2_C1 = -28f;
    public static final double COLLECT_STACK_Y_CYCLE2_C1 = 7.5f;
    public static final double COLLECT_STACK_X_CYCLE3_C1 = -27f;
    public static final double COLLECT_STACK_Y_CYCLE3_C1 = 8.5f;


    public static final double GO_TO_STACK_ANGLE_CYCLE1_C2 = 0.45;
    public static final double GO_TO_STACK_ANGLE_CYCLE2_C2= 0.60;
    public static final double GO_TO_STACK_ANGLE_CYCLE3_C2 = 0;


    public static final double COLLECT_STACK_Y_CYCLE2_C2 = 7.5f;
    public static final double COLLECT_STACK_X_CYCLE3_C2 = -26.5f;
    public static final double COLLECT_STACK_Y_CYCLE3_C2 = 8.5f;

    public static final double GO_TO_STACK_ANGLE_CYCLE1_C3 = 0;
    public static final double GO_TO_STACK_ANGLE_CYCLE2_C3= 0.2;
    public static final double GO_TO_STACK_ANGLE_CYCLE3_C3 = 0;
    public static final double COLLECT_STACK_X_CYCLE1_C3 = -28f;
    public static final double COLLECT_STACK_Y_CYCLE1_C3 = 9.5f;
    public static final double COLLECT_STACK_X_CYCLE2_C3 = -28f;
    public static final double COLLECT_STACK_Y_CYCLE2_C3 = 8.5f;
    public static final double COLLECT_STACK_X_CYCLE3_C3 = -26.5f;
    public static final double COLLECT_STACK_Y_CYCLE3_C3 = 8.5f;


    public static final double PLACE_BB_LLH1_X = 20;
    public static final double PLACE_BB_LLH1_Y = 11;
    public static final int ANGLE_BB_LLH1 = 0;
    public static final double PLACE_BB_LLH2_X = 49f;
    public static final double PLACE_BB_LLH2_Y = 48f;
    public static final double ANGLE_BB_LLH2 = -25f;

}  