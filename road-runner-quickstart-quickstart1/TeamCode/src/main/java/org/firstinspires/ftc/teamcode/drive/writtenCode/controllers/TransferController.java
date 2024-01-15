package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.FLIP_BOX;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.PLACE_BOX_IN_COLLECT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.TRANSFER_PIXELS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class TransferController {
    public enum TransferStatus{
        INIT,
        BLOCHEAZA_TUBULETE,
        FLIP_BOX,
        TRANSFER_PIXELS,
        PLACE_BOX_IN_COLLECT,
    }

    public TransferStatus currentStatus = INIT;
    public TransferStatus previousStatus = null;
    private IntakeController intakeController = null;
    private SigurantaOuttakeController sigurantaOuttakeController = null;
    private TubuleteController tubuleteController = null;
    private Servo forbarCutieIntake = null;

    public double timerFlipCutie = 0.8; /// Aici modifici cat timp ia sa-si dea flip cutia
    public double timerAsteaptaPixeli = 0.3;
    /// Aici modifici cat timp ia sa se duca pixelii din cutia de intake in cutia de outtake
    public double transferPosition = 0.294; // De schimbat
    public double collectPosition = 1; //  De schimbat
    public ElapsedTime asteaptaCutie = new ElapsedTime();
    public ElapsedTime asteaptaPixeli = new ElapsedTime();
    public TransferController(IntakeController intakeController,
                              TubuleteController tubuleteController,
                              SigurantaOuttakeController sigurantaOuttakeController,
                              RobotMap robot)
    {
        this.intakeController = intakeController;
        this.tubuleteController = tubuleteController;
        this.forbarCutieIntake = robot.forbarCutieIntake;
        this.sigurantaOuttakeController = sigurantaOuttakeController;
    }

    public void update()
    {
        if (currentStatus != previousStatus || currentStatus == TRANSFER_PIXELS || currentStatus == PLACE_BOX_IN_COLLECT)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case BLOCHEAZA_TUBULETE: /// Imi pune tubuletele si deschide siguranta la outtake
                {
                    sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.BLOCARE;
                    currentStatus = FLIP_BOX;
                    break;
                }
                case FLIP_BOX: // Imi da flip la cutia de intake
                {
                    forbarCutieIntake.setPosition(transferPosition);
                    // Cat timp isi da flip cutia va fi reversed intake ul
                    intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                    asteaptaCutie.reset(); /// start la timp
                    currentStatus = TRANSFER_PIXELS;
                    break;
                }
                case TRANSFER_PIXELS: // Astept sa dea flip cutia si apoi fac transferul cu tubuletele
                {
                    if (asteaptaCutie.seconds()>timerFlipCutie)
                    {
                        /// Dupa ce si-a dat flip cutia il opresc.
                        intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                        tubuleteController.currentStatus = TubuleteController.CollectStatus.TRANSFER;
                        currentStatus = PLACE_BOX_IN_COLLECT;
                        asteaptaPixeli.reset();
                    }
                    break;
                }
                case PLACE_BOX_IN_COLLECT: // Dupa ce s-a facut transferul , inchid siguranta si dau flip la cutia de intake.
                {
                    if (asteaptaPixeli.seconds()>timerAsteaptaPixeli)
                    {
                        sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.CLOSED;
                        forbarCutieIntake.setPosition(collectPosition);
                        currentStatus = INIT;
                    }
                    break;
                }
            }
        }
    }
}
