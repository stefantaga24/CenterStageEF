package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class CollectForbarController {
    public enum CollectStatus{
        INIT,
        COLLECT_DRIVE,
    }

    public double currentStatus = CollectStatus.INIT;
    public double previousStatus = null;

    public double initPosition = 0.5; // De modificat
    public double collectPosition = 0.3; // De modificat

    private Servo forbarIntake = null;

    public CollectForbarController(RobotMap robot)
    {
        forbarIntake = robot.forbarIntake;
    }

    public update()
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
                case COLLECT_DRIVE:
                {
                    forbarIntake.setPosition(collectPosition);
                    break;
                }
            }
        }
    }
}
