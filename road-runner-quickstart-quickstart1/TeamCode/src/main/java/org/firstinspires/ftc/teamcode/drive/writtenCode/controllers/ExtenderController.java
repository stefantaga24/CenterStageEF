package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class ExtenderController {
    public enum ExtenderStatus{
        INIT,
        FIX,
        FAR,
        FAILSAFE
    }
    public ExtenderStatus currentStatus = ExtenderStatus.INIT;
    public ExtenderStatus previousStatus = null;
    public int extenderInit =-5; /// Pozitia de init a extenderului
    public int extenderFar = 965; /// Pozitia de extensie a extenderului
    public int currentPosition = -5;
    public int extenderFix = 50;
    public int extenderFailsafe = 300;

    private DcMotor leftExtension;
    private DcMotor rightExtension;
    public ExtenderController(RobotMap robot)
    {
        this.leftExtension = robot.leftExtension;
        this.rightExtension = robot.rightExtension;
    }

    public void update()
    {
        this.leftExtension.setTargetPosition(currentPosition);
        this.leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftExtension.setPower(1);

        this.rightExtension.setTargetPosition(currentPosition);
        this.rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightExtension.setPower(1);

        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case FIX:
                {
                    currentPosition = extenderFix;
                    break;
                }
                case INIT:
                {
                    currentPosition = extenderInit;
                    break;
                }
                case FAR:
                {
                    currentPosition = extenderFar;
                    break;
                }
                case FAILSAFE:
                {
                    currentPosition = extenderFailsafe;
                    break;
                }
            }
        }
    }
}
