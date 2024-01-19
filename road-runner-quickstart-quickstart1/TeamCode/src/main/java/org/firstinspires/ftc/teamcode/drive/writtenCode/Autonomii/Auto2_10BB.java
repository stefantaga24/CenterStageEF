package org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.END_AUTO;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.GO_COLLECT_PIXELS;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.GO_TO_STACK_FIRST;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.PARK;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.PLACE_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.PLACE_STACK_PIXELS_BB;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.RETRACT_LIFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.WAIT_FOR_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.ANGLE_PARK_LEFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.ANGLE_PARK_MID;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.ANGLE_PARK_RIGHT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.ANGLE_SPIKE_LEFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.ANGLE_SPIKE_MID;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.ANGLE_SPIKE_RIGHT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PARK_LEFT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PARK_LEFT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PARK_MID_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PARK_MID_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PARK_RIGHT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PARK_RIGHT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PLACE_SPIKE_LEFT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PLACE_SPIKE_LEFT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PLACE_SPIKE_MID_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PLACE_SPIKE_MID_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PLACE_SPIKE_RIGHT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PLACE_SPIKE_RIGHT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PRELOAD_ANGLE_LEFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PRELOAD_LEFT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PRELOAD_MID_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PRELOAD_MID_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PRELOAD_RIGHT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoPreloadBB.PRELOAD_RIGHT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.AvionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CataratController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectForbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ExtenderController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ForbarOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ParbrizController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Pixel2Controller;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.RotateClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoringController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.SigurantaOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TubuleteController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Config
@Autonomous(group = "Auto")

public class Auto2_10BB extends LinearOpMode {

    public static final int GO_TO_STACK_X = 20;
    public static final int GO_TO_STACK_Y = -11;
    public static final int GO_TO_STACK_ANGLE = 180;
    public static final int COLLECT_STACK_X = -5;
    public static final int COLLECT_STACK_Y = -11;
    public static final int PLACE_BB_LLH1_X = 20;
    public static final int PLACE_BB_LLH1_Y = -11;
    public static final int ANGLE_BB_LLH1 = 180;
    public static final int PLACE_BB_LLH2_X = 50;
    public static final int PLACE_BB_LLH2_Y = -27;
    public static final int ANGLE_BB_LLH2 = 180;

    enum STROBOT
    {
        START,
        PLACE_PURPLE_PIXEL,
        WAIT_FOR_PURPLE_PIXEL,
        PLACE_SPIKE_BACKDROP,
        PARK,
        RETRACT_LIFT,
        GO_COLLECT_PIXELS,
        PLACE_STACK_PIXELS_BB,
        GO_TO_STACK_FIRST,
        END_AUTO,
    }
    public static double timePlacePixel = 0.2;
    public static double delayLift = 0.4;
    public static double waitTimeBackDrop = 0.3;
    public static double timeOpenSlides = 0.3;
    ElapsedTime timerPunerePixel = new ElapsedTime();
    ElapsedTime timerLift = new ElapsedTime();
    ElapsedTime timerSlides = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CataratController cataratController = new CataratController(robot);
        AvionController avionController = new AvionController(robot);
        IntakeController intakeController = new IntakeController(robot);
        CollectForbarController collectForbarController = new CollectForbarController(robot);
        TubuleteController tubuleteController = new TubuleteController(robot);
        Pixel2Controller pixel2Controller = new Pixel2Controller(robot);
        ParbrizController parbrizController = new ParbrizController(robot);
        SigurantaOuttakeController sigurantaOuttakeController = new SigurantaOuttakeController(robot);
        ForbarOuttakeController forbarOuttakeController = new ForbarOuttakeController(robot);
        RotateClawController rotateClawController = new RotateClawController(robot);
        ExtenderController extenderController = new ExtenderController(robot);
        LiftMotorController liftMotorController = new LiftMotorController(forbarOuttakeController,extenderController,robot);
        robot.forbarCutieIntake.setPosition(initPosition);


        TransferController transferController = new TransferController(
                intakeController,tubuleteController,sigurantaOuttakeController,robot);
        ScoringController scoringController = new ScoringController(pixel2Controller, sigurantaOuttakeController, parbrizController, rotateClawController);

        cataratController.update();
        avionController.update();
        extenderController.update();
        rotateClawController.update();
        forbarOuttakeController.update();
        liftMotorController.update();
        intakeController.update();
        collectForbarController.update();
        tubuleteController.update();
        transferController.update();
        pixel2Controller.update();
        parbrizController.update();
        sigurantaOuttakeController.update();
        scoringController.update();


        Pose2d startPose = new Pose2d(10, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        double nrCycles = 0;
        double howManyCycles = 1;
        TrajectorySequence PLACE_PRELOAD_LEFT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(AutoPreloadBB.PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                .build();
        TrajectorySequence PLACE_SPIKE_LEFT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_LEFT.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y,Math.toRadians(ANGLE_SPIKE_LEFT)))
                .build();



        TrajectorySequence PLACE_PRELOAD_MID = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_MID_X, PRELOAD_MID_Y,Math.toRadians(AutoPreloadBB.PRELOAD_ANGLE_MID)))
                .build();
        TrajectorySequence PLACE_SPIKE_MID = drive.trajectorySequenceBuilder(PLACE_PRELOAD_MID.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_MID_X, PLACE_SPIKE_MID_Y,Math.toRadians(ANGLE_SPIKE_MID)))
                .build();



        TrajectorySequence PLACE_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_RIGHT_X, PRELOAD_RIGHT_Y,Math.toRadians(ANGLE_SPIKE_RIGHT)))
                .build();
        TrajectorySequence PLACE_SPIKE_RIGHT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_RIGHT.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_RIGHT_X, PLACE_SPIKE_RIGHT_Y,Math.toRadians(ANGLE_SPIKE_RIGHT)))
                .build();
        TrajectorySequence GO_TO_STACK_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE)))
                .build();

        TrajectorySequence GO_TO_STACK_MID = drive.trajectorySequenceBuilder(PLACE_SPIKE_MID.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE)))
                .build();

        TrajectorySequence GO_TO_STACK_RIGHT = drive.trajectorySequenceBuilder(PLACE_SPIKE_RIGHT.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE)))
                .build();
        TrajectorySequence GO_COLLECT_STACK = drive.trajectorySequenceBuilder(
                new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE)))
                .lineTo(new Vector2d(COLLECT_STACK_X, COLLECT_STACK_Y))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD = drive.trajectorySequenceBuilder(GO_COLLECT_STACK.end())
                .lineToLinearHeading(new Pose2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y,Math.toRadians(ANGLE_BB_LLH1))) // se da cu spatele
                .lineToLinearHeading(new Pose2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y, Math.toRadians(ANGLE_BB_LLH2)))
                .build(); // te duce la backboard
        TrajectorySequence GO_TO_STACK_GENERAL = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE)))
                .build();
        TrajectorySequence PARK_ROBOT = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .lineToLinearHeading(new Pose2d(PARK_RIGHT_X, PARK_RIGHT_Y,Math.toRadians(ANGLE_PARK_RIGHT)))
                .build();

        double cazAuto = 1;
        while (!isStarted()&&!isStopRequested())
        {
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            switch (status)
            {
                case START:
                {
                    if (cazAuto == 1)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD_LEFT);
                    }
                    else if (cazAuto ==2)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD_MID);
                    }
                    else if (cazAuto == 3)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD_RIGHT);
                    }
                    status = STROBOT.PLACE_PURPLE_PIXEL;
                    break;
                }
                case PLACE_PURPLE_PIXEL:
                {
                    if (!drive.isBusy())
                    {
                        intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE_AUTO;
                        timerPunerePixel.reset();
                        status = WAIT_FOR_PURPLE_PIXEL;
                    }
                    break;
                }
                case WAIT_FOR_PURPLE_PIXEL:
                {
                    if (timerPunerePixel.seconds()>timePlacePixel)
                    {
                        intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                        if (cazAuto == 1)
                        {
                            drive.followTrajectorySequenceAsync(PLACE_SPIKE_LEFT);
                        }
                        else if (cazAuto ==2)
                        {
                            drive.followTrajectorySequenceAsync(PLACE_SPIKE_MID);
                        }
                        else if (cazAuto == 3)
                        {
                            drive.followTrajectorySequenceAsync(PLACE_SPIKE_RIGHT);
                        }
                        timerLift.reset();
                        status = PLACE_SPIKE_BACKDROP;
                    }
                    break;
                }
                case PLACE_SPIKE_BACKDROP:
                {
                    if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT
                            && timerLift.seconds()>delayLift)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.LOW_AUTO;
                    }
                    if (!drive.isBusy())
                    {
                        scoringController.currentStatus = ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        timerPunerePixel.reset();
                        status = GO_TO_STACK_FIRST;
                    }
                    break;
                }
                case GO_TO_STACK_FIRST:
                {
                    if (timerPunerePixel.seconds() > waitTimeBackDrop)
                    {
                        nrCycles++;
                        if (nrCycles == 1)
                        {
                            if (cazAuto == 1)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_LEFT);
                            }
                            else if (cazAuto ==2)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_MID);
                            }
                            else if (cazAuto ==3)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_RIGHT);
                            }
                        }
                        else
                        {
                            if (nrCycles > howManyCycles)
                            {
                                drive.followTrajectorySequenceAsync(PARK_ROBOT);
                            }
                            else
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_GENERAL);
                            }
                        }
                        timerLift.reset();
                        status = RETRACT_LIFT;
                    }
                    break;
                }
                case RETRACT_LIFT:
                {
                    if (timerLift.seconds()> 0.3 && liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                    }
                    if (!drive.isBusy())
                    {
                        if (nrCycles > 0)
                        {
                            status = END_AUTO;
                            break;
                        }
                        status = GO_COLLECT_PIXELS;
                        timerSlides.reset();
                        drive.followTrajectorySequenceAsync(GO_COLLECT_STACK);
                    }
                    break;
                }
                case GO_COLLECT_PIXELS:
                {
                    if (timerSlides.seconds()>timeOpenSlides &&
                            extenderController.currentStatus != ExtenderController.ExtenderStatus.FAR)
                    {
                        extenderController.currentStatus = ExtenderController.ExtenderStatus.FAR;
                        collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK;
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT_DRIVE;
                    }
                    if (robot.beamFront.getState() == false && robot.beamBack.getState() == false)
                    {
                        pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.OPEN;

                        // Pun timpul pentru extendo
                        transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;

                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                        drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                        status = PLACE_STACK_PIXELS_BB;
                    }
                    break;
                }
                case PLACE_STACK_PIXELS_BB:
                {
                    if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT &&
                            transferController.currentStatus == TransferController.TransferStatus.INIT)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.LOW;
                    }
                    if (!drive.isBusy() && liftMotorController.currentStatus == LiftMotorController.LiftStatus.LOW)
                    {
                        scoringController.currentStatus = ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        timerPunerePixel.reset();
                        status = GO_TO_STACK_FIRST;
                    }
                    break;
                }
            }








            cataratController.update();
            avionController.update();
            extenderController.update();
            rotateClawController.update();
            forbarOuttakeController.update();
            liftMotorController.update();
            intakeController.update();
            collectForbarController.update();
            tubuleteController.update();
            transferController.update();
            pixel2Controller.update();
            parbrizController.update();
            sigurantaOuttakeController.update();
            scoringController.update();
            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}