package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class ForbarOuttakeController {
    public enum ForbarStatus{
        GET_COLLECTED_PIXELS, // Cand putem efectua transfer
        PLACE_ON_BACKBOARD, // Cand pui pe backboard
        MOVE_TO_BACKBOARD_DELAY, // Cand vrei sa-l misti cu un delay
        PLACE_ON_BACKBOARD_WITH_DELAY,
    }
    public ForbarStatus currentStatus = ForbarStatus.GET_COLLECTED_PIXELS;
    public ForbarStatus previousStatus = null;
    public double getPixelsPosition = 0;
    public double placePixelsPosition =0.5;
    private ElapsedTime waitForTheLift = new ElapsedTime();
    private double timerPentruForbarOuttake = 0.3; // Modificati asta in functie de cat sa stea forbarul pana se ridica.
    private Servo forbarOuttake = null;
    public ForbarOuttakeController(RobotMap robot)
    {
        this.forbarOuttake = robot.forbarOuttake;
    }
    public void update()
    {
        /// Orice status care are timere in el trebuie sa apara in if.
        if (currentStatus != previousStatus || currentStatus == ForbarStatus.PLACE_ON_BACKBOARD_WITH_DELAY)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case GET_COLLECTED_PIXELS:
                {
                    this.forbarOuttake.setPosition(getPixelsPosition);
                    break;
                }
                case PLACE_ON_BACKBOARD:
                {
                    this.forbarOuttake.setPosition(placePixelsPosition);
                    break;
                }
                case MOVE_TO_BACKBOARD_DELAY:
                {
                    waitForTheLift.reset();
                    currentStatus = ForbarStatus.PLACE_ON_BACKBOARD_WITH_DELAY;
                    break;
                }
                case PLACE_ON_BACKBOARD_WITH_DELAY:
                {
                    if (waitForTheLift.seconds()>timerPentruForbarOuttake)
                    {
                        currentStatus = ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    break;
                }
            }
        }
    }
}
