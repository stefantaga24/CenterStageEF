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
                                .lineToLinearHeading(new Pose2d(PRELOAD_MID_X, PRELOAD_MID_Y,Math.toRadians(PRELOAD_ANGLE_MID)))
                                .setReversed(true)
//                                .strafeRight(10)
                                .lineToLinearHeading(new Pose2d(FIRST_PIXEL_X,FIRST_PIXEL_Y,Math.toRadians(0)))
//                                .lineTo(new Vector2d(PLACE_BB_LLH1_X_PRELOAD,PLACE_BB_LLH1_Y_PRELOAD))
//                                .splineToConstantHeading(new Vector2d(LEFT_SPLINE_X, LEFT_SPLINE_Y),Math.toRadians(0))
//                                .lineTo(new Vector2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

    static Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));

    public static double forwardRight = 6.75;

    public static  double PRELOAD_RIGHT_X = -31.5;
    public static  double PRELOAD_RIGHT_Y = -38;
    public static  double PRELOAD_ANGLE_RIGHT = 42;

    public static  double PLACE_SPIKE_RIGHT_X = 49.5;
    public static  double PLACE_SPIKE_RIGHT_Y = -47;
    public static  double ANGLE_SPIKE__LEFT = 0;

    public static  double PRELOAD_MID_X = -38;
    public static  double PRELOAD_MID_Y = -33;
    public static  double PRELOAD_ANGLE_MID = 42;

    public static  double MID_SPLINE_X = 33.7;
    public static  double MID_SPLINE_Y = -40;
    public static  double PLACE_SPIKE_MID_X = 50.5;
    public static  double PLACE_SPIKE_MID_Y = -40;
    public static  double ANGLE_SPIKE_MID = 0;


    public static  double PRELOAD_LEFT_X = -43;
    public static  double PRELOAD_LEFT_Y = -40;
    public static  double PRELOAD_ANGLE_LEFT = 145;

    public static  double LEFT_SPLINE_X = 35;
    public static  double LEFT_SPLINE_Y = -33.5;
    public static  double PLACE_SPIKE_LEFT_X = 49.3;
    public static  double PLACE_SPIKE_LEFT_Y = -33.5;
    public static double ANGLE_SPIKE_LEFT = 0;

    public static double PARK_RIGHT_X = 43;
    public static double PARK_RIGHT_Y = -62;
    public static double ANGLE_PARK_RIGHT = 0;

    public static final double FIRST_PIXEL_X = -57.5;
    public static final double FIRST_PIXEL_Y = -11;
    public static final double FIRST_PIXEL_X_C1 =-59;



    public static final double GO_TO_STACK_X_C3 = 27f;
    public static final double GO_TO_STACK_Y_C3 = -8f;
    public static final double COLLECT_STACK_X_C3 = -27f;
    public static final double COLLECT_STACK_Y_C3 = -8f;

    public static final double GO_TO_STACK_X_C2 = 33f;
    public static final double GO_TO_STACK_Y_C2 = -9.5f;
    public static final double COLLECT_STACK_X_C2 = -27f;
    public static final double COLLECT_STACK_Y_C2 = -9.5f;

    public static final double GO_TO_STACK_X_C1 = 30f;
    public static final double GO_TO_STACK_Y_C1 = -9.5f;
    public static final double COLLECT_STACK_X_C1 = -29.5f;
    public static final double COLLECT_STACK_Y_C1 = -9.5f;

    public static final double COLLECT_STACK_ANGLE=0f;

    public static final double PLACE_BB_LLH1_X_CYCLES = 15;
    public static final double PLACE_BB_LLH1_Y_CYCLES = -11f;
    public static final double PLACE_BB_LLH2_X_CYCLES = 48.5;
    public static final double PLACE_BB_LLH2_Y_CYCLES = -41f;




    public static final double PLACE_BB_LLH1_X = 20;
    public static final double PLACE_BB_LLH1_Y = -11;
    public static final int ANGLE_BB_LLH1 = 0;
    public static final double PLACE_BB_LLH2_X = 49f;
    public static final double PLACE_BB_LLH2_Y = -50f;
    public static final double ANGLE_BB_LLH2 = -25f;

    public static final double PLACE_BB_LLH1_X_PRELOAD = 15;
    public static final double PLACE_BB_LLH1_Y_PRELOAD = -11;

    static Pose2d BBPose = new Pose2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y, Math.toRadians(0));

}  