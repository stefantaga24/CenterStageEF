package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class BoxController {
    public enum CollectStatus{
        COLLECT,
        TRANSFER,
    }

    public double currentStatus = CollectStatus.COLLECT;
    public double previousStatus = null;

    public double collectPosition = 0.5; // De modificat
    public double transferPosition = 0.3; // De modificat

    private Servo forbarCutieIntake = null;

    public BoxController(RobotMap robot)
    {
        forbarCutieIntake = robot.forbarCutieIntake;
    }

    public update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case COLLECT:
                {
                    forbarCutieIntake.setPosition(collectPosition);
                    break;
                }
                case TRANSFER:
                {
                    forbarCutieIntake.setPosition(transferPosition);
                    break;
                }
            }
        }
    }
}
