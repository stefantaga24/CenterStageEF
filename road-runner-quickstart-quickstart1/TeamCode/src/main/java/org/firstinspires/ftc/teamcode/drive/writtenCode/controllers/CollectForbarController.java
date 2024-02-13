package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class CollectForbarController {
    public enum CollectStatus{
        INIT,
        JERK,
        PLAY,
        COLLECT_DRIVE,
        COLLECT_DRIVE_STACK,
        COLLECT_AUTO_STACK_5,
        COLLECT_AUTO_STACK_4,
        COLLECT_AUTO_STACK_LOW
    }

    public CollectStatus currentStatus = CollectStatus.INIT;
    public CollectStatus previousStatus = null;

    public double initPosition = 1f;
    public double collectPosition = 0.278f;
    public double collectStackPosition = 0.73f; /// Trebuie sa modificati pozitia asta pt stack
    public double collectStackPixel5 = 0.74f;
    public double collectStackPixel4 = 0.71f;
    public double collectStackAutoLowPosition = 0.62f;
    public double playPosition = 0.5f;
    public double autoPosition = 1f;

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
            }
        }
    }
}
