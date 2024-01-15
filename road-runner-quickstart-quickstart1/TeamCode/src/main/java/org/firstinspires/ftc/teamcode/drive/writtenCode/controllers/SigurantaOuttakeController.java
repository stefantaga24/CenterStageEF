package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class SigurantaOuttakeController {
    public enum SigurantaOuttakeStatus{
        CLOSED,
        OPEN,
    }

    public SigurantaOuttakeStatus currentStatus = SigurantaOuttakeStatus.CLOSED;
    public SigurantaOuttakeStatus previousStatus = null;

    public double closedPosition = 0.5; // De modificat
    public double openPosition = 0.3; // De modificat

    private Servo sigurantaOuttake = null;

    public SigurantaOuttakeController(RobotMap robot)
    {
        sigurantaOuttake = robot.sigurantaOuttake;
    }

    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case CLOSED:
                {
                    sigurantaOuttake.setPosition(closedPosition);
                    break;
                }
                case OPEN:
                {
                    sigurantaOuttake.setPosition(openPosition);
                    break;
                }
            }
        }
    }
}
