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
                        drive.trajectorySequenceBuilder(new Pose2d(PLACE_BB_LLH2_X_CYCLES,PLACE_BB_LLH2_Y_CYCLES,Math.toRadians(PLACE_BB_LLH2_ANGLE_CYCLES)))
                                .setReversed(true)
                                .splineTo(new Vector2d(GO_TO_STACK_X_C1,GO_TO_STACK_Y_C1),Math.toRadians(180))
                                .lineTo(new Vector2d(-10,COLLECT_STACK_Y_C1))
                                .splineTo(new Vector2d(-30,-15),Math.toRadians(200))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10,COLLECT_STACK_Y_C1),Math.toRadians(0))
                                .lineTo(new Vector2d(PLACE_BB_LLH1_X_CYCLES,-11.5))
//                                .setReversed(true)
                                .splineTo(new Vector2d(PLACE_BB_LLH2_X_CYCLES,PLACE_BB_LLH2_Y_CYCLES),Math.toRadians(PLACE_BB_LLH2_ANGLE_CYCLES))
//                                .lineToLinearHeading(new Pose2d(GO_TO_BB_PRELOAD_X,GO_TO_BB_PRELOAD_Y, Math.toRadians(0)))
//                                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y))
//                                .splineTo(new Vector2d(PLACE_SPIKE_LEFT_X,PLACE_SPIKE_LEFT_Y),Math.toRadians(-30))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(GO_TO_STACK_X_C1, GO_TO_STACK_Y_C1),Math.toRadians(180))
//                                .lineTo(new Vector2d(COLLECT_STACK_X_C1, COLLECT_STACK_Y_C1))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

    static Pose2d startPose = new Pose2d(-38.5, -62, Math.toRadians(90));


    public static double forwardRight = 6.75;

    public static  double PRELOAD_RIGHT_X = -32.5;
    public static  double PRELOAD_RIGHT_Y = -36;
    public static  double PRELOAD_ANGLE_RIGHT = 39.5;

    public static  double PLACE_SPIKE_RIGHT_X = 47;
    public static  double PLACE_SPIKE_RIGHT_Y = -47;


    public static  double PRELOAD_MID_X = -38;
    public static  double PRELOAD_MID_Y = -33;
    public static  double PRELOAD_ANGLE_MID = 42;

    public static  double PLACE_SPIKE_MID_X = 47;
    public static  double PLACE_SPIKE_MID_Y = -42.5;
    public static  double ANGLE_SPIKE_MID = -35;


    public static  double PRELOAD_LEFT_X = -41.5;
    public static  double PRELOAD_LEFT_Y = -38;
    public static  double PRELOAD_ANGLE_LEFT = 145;

    public static  double PLACE_SPIKE_LEFT_X = 49;
    public static  double PLACE_SPIKE_LEFT_Y = -27.5;
    public static double ANGLE_SPIKE_LEFT = -30;

    public static double PARK_RIGHT_X = 43;
    public static double PARK_RIGHT_Y = -10;
    public static double ANGLE_PARK_RIGHT = 0;

    public static final double FIRST_PIXEL_X = -56;
    public static final double FIRST_PIXEL_Y = -10;
    public static final double FIRST_PIXEL_X_C1 =-56.5;
    public static final double FIRST_PIXEL_X_C2 =-51.5;
    public static final double FIRST_PIXEL_X_C3 =-35;
    public static final double FIRST_PIXEL_Y_C1 =-17.5;
    public static final double FIRST_PIXEL_Y_C2 =-16.8;
    public static final double FIRST_PIXEL_Y_C3 =-27.5;

    public static final double GO_TO_BB_PRELOAD_X =-40;
    public static final double GO_TO_BB_PRELOAD_Y=-12;
    public static final double GO_TO_STACK_X_C3 = 27f;
    public static final double GO_TO_STACK_Y_C3 = -12f;
    public static final double COLLECT_STACK_X_C3 = -29.5f;
    public static final double COLLECT_STACK_Y_C3 = -12f;

    public static final double GO_TO_STACK_X_C2 = 33f;
    public static final double GO_TO_STACK_Y_C2 = -10.5f;
    public static final double COLLECT_STACK_X_C2 = -30.6f;
    public static final double COLLECT_STACK_Y_C2 = -10.5f;

    public static final double GO_TO_STACK_X_C1 = 11.5f;
    public static final double GO_TO_STACK_Y_C1 = -11.5f;
    public static final double COLLECT_STACK_X_C1 = -25f;
    public static final double COLLECT_STACK_Y_C1 = -11.5f;

    public static final double COLLECT_STACK_ANGLE=0f;

    public static final double PLACE_BB_LLH1_X_CYCLES = 15;
    public static final double PLACE_BB_LLH1_Y_CYCLES = -14f;
    public static final double PLACE_BB_LLH2_X_CYCLES = 48.5;
    public static final double PLACE_BB_LLH2_Y_CYCLES = -29;
    public static final double PLACE_BB_LLH2_ANGLE_CYCLES = -30;




    public static final double PLACE_BB_LLH1_X = 10;
    public static final double PLACE_BB_LLH1_Y = -12;
    public static final double PLACE_BB_LLH2_X = 44f;
    public static final double PLACE_BB_LLH2_Y = -22f;
    public static final double ANGLE_BB_LLH2 = -40f;

    public static final double PLACE_BB_LLH1_X_PRELOAD = 15;
    public static final double PLACE_BB_LLH1_Y_PRELOAD = -14;


}  