package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;
@Config
public class CollectForbarController {
    public enum CollectStatus{
        INIT,
        JERK,
        PLAY,
        COLLECT_DRIVE,
        COLLECT_DRIVE_STACK,
        COLLECT_DRIVE_STACK_LOW,
        COLLECT_AUTO_STACK_5,
        COLLECT_AUTO_STACK_4,
        COLLECT_AUTO_STACK_LOW,
        AI,
        ONE_PIXEL,
        ONE_PIXEL_FAILSAFE,

    }

    public CollectStatus currentStatus = CollectStatus.INIT;
    public CollectStatus previousStatus = null;

    public static double initPosition = 1f;
    public static double collectPosition = 0.15f;
    public static double collectStackPosition = 0.38f; /// 0.75 - OK era 0.5
    public static double collectStackLower = 0.25f;//era 0.34
    public static double collectStackPixel5 = 0.41f;
    public static double collectStackPixel4 = 0.33f;
    public static double collectStackAutoLowPosition = 0.27f;
    public static double collectStackAutoAI = 0.39f;
    public static double onePixel = 0.42f;
    public static double onePixelFailsafe = 0.39f;
    public static double playPosition = 1f;
    public static double autoPosition = 1f;

    private Servo forbarIntake = null;

    public CollectForbarController(RobotMap robot)
    {
        forbarIntake = robot.forbarIntake;
    }

    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case INIT:
                {
                    forbarIntake.setPosition(initPosition);
                    break;
                }
                case PLAY:
                {
                    forbarIntake.setPosition(playPosition);
                    break;
                }
                case COLLECT_DRIVE:
                {
                    forbarIntake.setPosition(collectPosition);
                    break;
                }
                case COLLECT_DRIVE_STACK:
                {
                    forbarIntake.setPosition(collectStackPosition);
                    break;
                }
                case COLLECT_AUTO_STACK_5:
                {
                    forbarIntake.setPosition(collectStackPixel5);
                    break;
                }
                case COLLECT_AUTO_STACK_4:
                {
                    forbarIntake.setPosition(collectStackPixel4);
                    break;
                }
                case COLLECT_AUTO_STACK_LOW:
                {
                    forbarIntake.setPosition(collectStackAutoLowPosition);
                    break;
                }
                case COLLECT_DRIVE_STACK_LOW:
                {
                    forbarIntake.setPosition(collectStackLower);
                    break;
                }
                case AI:
                {
                    forbarIntake.setPosition(collectStackAutoAI);
                    break;
                }
                case ONE_PIXEL:
                {
                    forbarIntake.setPosition(onePixel);
                    break;
                }
                case ONE_PIXEL_FAILSAFE:
                {
                    forbarIntake.setPosition(onePixelFailsafe);
                    break;
                }
            }
        }
    }
}
