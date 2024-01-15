package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class IntakeController {
    public enum IntakeStatus{
        STOP,
        COLLECT_DRIVE,
        REVERSE,
    }

    public IntakeStatus currentStatus = IntakeStatus.STOP;
    public IntakeStatus previousStatus = null;

    public double driveCollectPower = 0.8;
    public double reverseCollectPower = -0.8;
    private DcMotorEx intakeMotor = null;

    /**
     * Constructor pentru clasa IntakeController
     * Practic cand cream un nou controller trebuie sa pasam robotMap
     * @param robot - Clasa care contine tot hardware-ul robotului
     */
    public IntakeController(RobotMap robot)
    {
        intakeMotor = robot.intakeMotor;
    }

    public void update()
    {
        if (currentStatus != previousStatus )
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case STOP:
                {
                    intakeMotor.setPower(0);
                    break;
                }
                case COLLECT_DRIVE:
                {
                    intakeMotor.setPower(driveCollectPower);
                    break;
                }
                case REVERSE:
                {
                    intakeMotor.setPower(reverseCollectPower);
                    break;
                }
            }
        }
    }
}