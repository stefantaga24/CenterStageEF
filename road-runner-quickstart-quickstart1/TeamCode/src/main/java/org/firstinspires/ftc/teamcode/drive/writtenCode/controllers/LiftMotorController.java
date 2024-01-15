package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.IDLE;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.WAIT_TO_LIFT_FORBAR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class LiftMotorController {
    public enum LiftStatus{
        IDLE,
        INIT,
        LOW,
        MID,
        HIGH,
        WAIT_TO_LIFT_FORBAR
    }
    public LiftStatus currentStatus = LiftStatus.INIT;
    public LiftStatus previousStatus = null;
    public int initPosition = -20;
    public int lowPosition =450;
    public int midPosition =700;
    public int highPosition = 900;
    public int currentPosition = initPosition;
    private ForbarOuttakeController forbarOuttakeController = null;
    public DcMotor liftMotor = null;



    public LiftMotorController(ForbarOuttakeController forbarOuttakeController, RobotMap robot)
    {
        this.forbarOuttakeController = forbarOuttakeController;
        this.liftMotor = robot.liftMotor;
    }
    public void update()
    {
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
                    currentStatus = IDLE;
                    break;
                }
                case LOW:
                {
                    currentPosition = lowPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    currentStatus = IDLE;
                    break;
                }
                case MID:
                {
                    currentPosition = midPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus =  ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    currentStatus = IDLE;
                    break;
                }
                case HIGH:
                {
                    currentPosition = highPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus =  ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    currentStatus = IDLE;
                    break;
                }
            }
            previousStatus = currentStatus;
        }
    }
}
