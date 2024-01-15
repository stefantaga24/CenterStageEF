package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class BoxController {
    public enum CollectStatus{
        COLLECT,
        TRANSFER,
    }

    public CollectStatus currentStatus = CollectStatus.COLLECT;
    public CollectStatus previousStatus = null;

    public double collectPosition = 0.5; // De modificat
    public double transferPosition = 0.3; // De modificat

    private Servo forbarCutieIntake = null;

    public BoxController(RobotMap robot)
    {
        forbarCutieIntake = robot.forbarCutieIntake;
    }

    public void update()
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
