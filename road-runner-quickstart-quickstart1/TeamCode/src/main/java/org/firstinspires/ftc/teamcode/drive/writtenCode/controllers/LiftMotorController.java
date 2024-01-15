package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class LiftMotorController {
    public enum LiftStatus{
        INIT,
        LOW,
        MID,
        HIGH,
    }
    public LiftStatus currentStatus = LiftStatus.INIT;
    public LiftStatus previousStatus = null;
    public int initPosition = -20;
    public int lowPosition =450;
    public int midPosition =700;
    public int highPosition = 900;
    private int currentPosition = initPosition;
    public DcMotor liftMotor = null;
    public LiftMotorController(RobotMap robot)
    {
        this.liftMotor = robot.liftMotor;
    }
    public void update()
    {
        this.liftMotor.setTargetPosition(currentPosition);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor.setPower(1);
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case INIT:
                {
                    currentPosition = initPosition;
                    break;
                }
                case LOW:
                {
                    currentPosition = lowPosition;
                    break;
                }
                case MID:
                {
                    currentPosition = midPosition;
                    break;
                }
                case HIGH:
                {
                    currentPosition = highPosition;
                    break;
                }
            }
        }
    }
}
