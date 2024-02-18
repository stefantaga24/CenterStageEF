package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.AUTO_CYCLE1_C23;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class LiftMotorController {
    public enum LiftStatus{
        INIT,
        GOING_DOWN,
        LOW,
        MID,
        HIGH,
        LOW_AUTO,
        AUTO_CYCLE2_C1,
        AUTO_CYCLE1_C1,
        AUTO_CYCLE1_C23,
        AUTO_CYCLE2_C23,
        AUTO_CYCLE3,
        liftMosaic,

    }
    public LiftStatus currentStatus = LiftStatus.INIT;
    public LiftStatus previousStatus = null;
    public static int initPosition = 10;
    public static int downPosition = -100;
    public static int lowPosition = -700;
    public static int midPosition = -700;
    public static int highPosition = -1630;

    public static int lowAuto = -360;
    public static int mosaicPosition = -317;
    public static int autoCycle1_C1 = -570;
    public static int autoCycle2_C1 = -900;
    public static int autoCycle1_C23 = -540;
    public static int autoCycle2_C23 = -800;
    public static int autoCycle3= -1000;

    public int retardPosition = 100;
    public int currentPosition = initPosition;
    private ForbarOuttakeController forbarOuttakeController = null;
    private ExtenderController extenderController;
    public DcMotor liftMotor = null;
    public MotorConfigurationType mct1;


    public LiftMotorController(ForbarOuttakeController forbarOuttakeController, ExtenderController extenderController, RobotMap robot)
    {
        this.extenderController = extenderController;
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
//                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
//                    if(liftCurrentPosition<=-200)
//                    {
//                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.DOWN;
//                    }
//                    else
//                    {
//                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.GET_COLLECTED_PIXELS;
//                    }
                    forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.GET_COLLECTED_PIXELS;
                    currentPosition = initPosition;
                    break;
                }
                case GOING_DOWN:
                {
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                    forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.DOWN;
                    currentPosition=downPosition;
                    break;
                }
                case LOW:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
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
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                    //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                  //  }
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
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
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
                case LOW_AUTO:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = lowAuto;
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
                case AUTO_CYCLE2_C1:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                   // {
                     //   extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle2_C1;
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
                case AUTO_CYCLE1_C1:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle1_C1;
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
                case AUTO_CYCLE1_C23:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                   // {
                     //   extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                  //  }
                    currentPosition = autoCycle1_C23;
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
                case AUTO_CYCLE2_C23:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                   // {
                   //     extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle2_C23;
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
                case AUTO_CYCLE3:
                {
                    //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    // {
                    //     extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle3;
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
                case liftMosaic:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                 //   {
                   //     extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                   // }
                    currentPosition = mosaicPosition;
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
            }

        }
            previousStatus = currentStatus;
        }
    }
