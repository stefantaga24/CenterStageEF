package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class RotateClawController {
    public enum RotateStatus{
        VERTICAL,
        HORIZONTAL,
    }

    public RotateStatus currentStatus = RotateStatus.VERTICAL;
    public RotateStatus previousStatus = null;

    public double verticalServoPosition = 0.115;
    public double horizontalServoPosition = 0.39;
        private Servo clawRotate = null;
    public RotateClawController(RobotMap robot)
    {
        clawRotate = robot.clawRotate;
    }
    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case VERTICAL:
                {
                    this.clawRotate.setPosition(verticalServoPosition);
                    break;
                }
                case HORIZONTAL:
                {
                    this.clawRotate.setPosition(horizontalServoPosition);
                    break;
                }
            }
        }
    }
}
