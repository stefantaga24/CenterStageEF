package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.robot.Robot;

public class ScoringController {
    public enum ScoringStatus{
        INIT,
        DROP_BOTH_PIXELS,
        DROP_ONE_PIXEL
    }
    public ScoringStatus currentStatus = ScoringStatus.INIT;
    public ScoringStatus previousStatus = null;
    public Pixel2Controller pixel2Controller = null;
    public ParbrizController parbrizController = null;
    public SigurantaOuttakeController sigurantaOuttakeController = null;
    public ScoringController(Pixel2Controller pixel2Controller , SigurantaOuttakeController sigurantaOuttakeController,
                             ParbrizController parbrizController)
    {
        this.pixel2Controller = pixel2Controller;
        this.sigurantaOuttakeController = sigurantaOuttakeController;
        this.parbrizController = parbrizController;
    }
    public void update()
    {
        if (currentStatus!=previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case DROP_ONE_PIXEL:
                {
                    pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.CLOSED;

                }

            }
        }
    }
}
