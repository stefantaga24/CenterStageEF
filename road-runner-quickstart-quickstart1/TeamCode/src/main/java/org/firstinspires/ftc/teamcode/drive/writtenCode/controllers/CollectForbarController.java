package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class CollectForbarController {
    public enum CollectStatus{
        INIT,
        JERK,
        PLAY,
        COLLECT_DRIVE,
        COLLECT_DRIVE_STACK,
    }

    public CollectStatus currentStatus = CollectStatus.INIT;
    public CollectStatus previousStatus = null;

    public double initPosition = 1f;
    public double collectPosition = 0.59f;
    public double collectStackPosition = 0.77f; /// Trebuie sa modificati pozitia asta pt stack
    public double playPosition = 0.7f;
    public double jerkPosition = 0.9f;
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
                case JERK:
                {
                    forbarIntake.setPosition(jerkPosition);
                    break;
                }
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
            }
        }
    }
}
