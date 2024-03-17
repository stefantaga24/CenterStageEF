package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class Pixel2Controller {
    public enum Pixel2Status{
        CLOSED,
        OPEN,
    }

    public Pixel2Status currentStatus = Pixel2Status.CLOSED;
    public Pixel2Status previousStatus = null;

    public double closedPosition = 0; // De modificat
    public double openPosition = 0.35; // De modificat

    private Servo pixel2Outtake = null;

    public Pixel2Controller(RobotMap robot)
    {
        pixel2Outtake = robot.pixel2Outtake;
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
                    pixel2Outtake.setPosition(closedPosition);
                    break;
                }
                case OPEN:
                {
                    pixel2Outtake.setPosition(openPosition);
                    break;
                }
            }
        }
    }
}
