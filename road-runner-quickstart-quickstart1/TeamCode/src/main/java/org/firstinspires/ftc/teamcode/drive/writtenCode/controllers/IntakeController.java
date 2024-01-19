package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class IntakeController {
    public enum IntakeStatus{
        STOP,
        COLLECT_DRIVE,
        REVERSE,
        REVERSE_AUTO,
    }

    public IntakeStatus currentStatus = IntakeStatus.STOP;
    public IntakeStatus previousStatus = null;

    public double driveCollectPower = 1;
    public double reverseCollectPower = -1;
    public double reverseAutoPower = -0.6;
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
                case REVERSE_AUTO:
                {
                    intakeMotor.setPower(reverseAutoPower);
                    break;
                }
            }
        }
    }
}