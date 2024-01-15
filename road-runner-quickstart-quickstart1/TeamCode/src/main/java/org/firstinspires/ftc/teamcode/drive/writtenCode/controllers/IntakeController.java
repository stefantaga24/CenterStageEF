package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class IntakeController {
    public enum IntakeStatus{
        STOP_COLLECT,
        COLLECT_DRIVE,
    }
    public static double driveCollectPower = 0.8;
    private DcMotorEx intakeMotor = null;

    /**
     * Constructor pentru clasa IntakeController
     * Practic cand cream un nou controller trebuie sa pasam robotMap
     * @param robot - Clasa care contine tot hardware-ul robotului
     */
    public IntakeController(RobotMap robot)
    {

    }
}
