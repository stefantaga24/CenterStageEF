package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class ExtenderController {
    public enum ExtenderStatus{
        INIT,
        FIX,
        FAR,
        FAILSAFE
    }
    public ExtenderStatus currentStatus = ExtenderStatus.INIT;
    public ExtenderStatus previousStatus = null;
    public static int extenderInit =-5; /// Pozitia de init a extenderului
    public int extenderFar = 960; /// Pozitia de extensie a extenderului
    public static double Kp = 0; //0.00325;
    public static double Ki = 0; //0.0022;
    public static double Kd = 0;
    public int CurrentPosition = -5;
    public int extenderFix = 50;
    public static int extenderFailsafe = 300;
    public static double PowerCap = 1;
    public static double maxSpeed = 1;
    private DcMotor leftExtension;
    private DcMotor rightExtension;
    SimplePIDController MotorColectarePID = null;
    public ExtenderController(RobotMap robot)
    {
        this.leftExtension = robot.leftExtension;
        this.rightExtension = robot.rightExtension;
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue= extenderInit;
        MotorColectarePID.maxOutput = maxSpeed;
    }

    public void update(int ColectarePosition) {
        CurrentPosition = ColectarePosition;
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        this.leftExtension.setPower(powerColectare);
        this.rightExtension.setPower(powerColectare);

        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case FIX:
                {
                    MotorColectarePID.targetValue = extenderFix;
                    break;
                }
                case INIT:
                {
                    MotorColectarePID.targetValue = extenderInit;
                    break;
                }
                case FAR:
                {
                    MotorColectarePID.targetValue = extenderFar;
                    break;
                }
                case FAILSAFE:
                {
                    MotorColectarePID.targetValue = extenderFailsafe;
                    break;
                }
            }
        }
    }
}
