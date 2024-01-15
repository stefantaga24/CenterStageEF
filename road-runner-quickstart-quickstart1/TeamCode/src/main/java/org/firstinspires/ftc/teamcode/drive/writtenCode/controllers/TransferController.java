package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class TransferController {
    public enum TransferStatus{
        INIT,
        BLOCHEAZA_TUBULETE,
    }

    public double currentStatus = TransferStatus.INIT;
    public double previousStatus = null;
    private IntakeController intakeController = null;
    private TubuleteController tubuleteController = null;
    private Servo forbarCutieIntake = null;
    public TransferController(IntakeController intakeController,
                              TubuleteController tubuleteController,
                              RobotMap robot)
    {
        this.intakeController = intakeController;
        this.tubuleteController = tubuleteController;
        this.forbarCutieIntake = robot.forbarCutieIntake;
    }

    public update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case BLOCHEAZA_TUBULETE:
                {
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.BLOCARE;
                }
            }
        }
    }
}
