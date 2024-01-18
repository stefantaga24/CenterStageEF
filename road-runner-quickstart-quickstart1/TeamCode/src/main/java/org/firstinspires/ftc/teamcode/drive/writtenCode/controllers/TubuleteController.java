package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class TubuleteController {
    public enum CollectStatus{
        COLECTARE,
        TRANSFER,
        BLOCARE
    }

    public CollectStatus currentStatus = CollectStatus.COLECTARE;
    public CollectStatus previousStatus = null;

    public double collectPosition1 = 0.13;
    public double collectPosition2 = 1-collectPosition1;

    public double transferPosition1 = 1;
    public double transferPosition2 = 1-transferPosition1;

    public double blocarePosition1 = 0.05;
    public double blocarePosition2 = 1- blocarePosition1;

    private Servo leftTransferServo = null;
    private Servo rightTransferServo = null;

    public TubuleteController(RobotMap robot)
    {
        leftTransferServo = robot.leftTransferServo;
        rightTransferServo = robot.rightTransferServo;
    }

    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case COLECTARE:
                {
                    leftTransferServo.setPosition(collectPosition1);
                    rightTransferServo.setPosition(collectPosition2);
                    break;
                }
                case TRANSFER:
                {
                    leftTransferServo.setPosition(transferPosition1);
                    rightTransferServo.setPosition(transferPosition2);
                    break;
                }
                case BLOCARE:
                {
                    leftTransferServo.setPosition(blocarePosition1);
                    rightTransferServo.setPosition(blocarePosition2);
                }
            }
        }
    }
}
