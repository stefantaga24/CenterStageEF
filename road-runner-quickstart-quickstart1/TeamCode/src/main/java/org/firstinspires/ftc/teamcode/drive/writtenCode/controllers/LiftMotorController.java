package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.WAIT_TO_LIFT_FORBAR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class LiftMotorController {
    public enum LiftStatus{
        INIT,
        LOW,
        MID,
        HIGH,
        WAIT_TO_LIFT_FORBAR
    }
    public LiftStatus currentStatus = LiftStatus.INIT;
    public LiftStatus previousStatus = null;
    public int initPosition = -10;
    public int lowPosition = -700;
    public int midPosition = -700;
    public int highPosition = -1630;

    public int retardPosition = 100;
    public int currentPosition = initPosition;
    private ForbarOuttakeController forbarOuttakeController = null;
    public DcMotor liftMotor = null;
    public MotorConfigurationType mct1;


    public LiftMotorController(ForbarOuttakeController forbarOuttakeController, RobotMap robot)
    {
        this.forbarOuttakeController = forbarOuttakeController;
        this.liftMotor = robot.liftMotor;
    }
    public void update()
    {
        mct1 = liftMotor.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        liftMotor.setMotorType(mct1);

        this.liftMotor.setTargetPosition(currentPosition);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor.setPower(1);
        double liftCurrentPosition = this.liftMotor.getCurrentPosition();
        if (currentStatus != previousStatus)
        {
            /// Atentie baieti! Delay-ul de la ridicare forbarului din outtake
            // merge doar daca liftul vine dintr-o pozitie mai mica decat 0 adica din INIT.
            switch (currentStatus)
            {
                case INIT:
                {
                    forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.GET_COLLECTED_PIXELS;
                    currentPosition = initPosition;
                    break;
                }
                case LOW:
                {
                    currentPosition = lowPosition;
                    if (liftCurrentPosition >=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    break;
                }
                case MID:
                {
                    currentPosition = midPosition;
                    if (liftCurrentPosition >=0)
                    {
                        forbarOuttakeController.currentStatus =  ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    break;
                }
                case HIGH:
                {
                    currentPosition = highPosition;
                    if (liftCurrentPosition >=0)
                    {
                        forbarOuttakeController.currentStatus =  ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    break;
                }
            }
            previousStatus = currentStatus;
        }
    }
}
