package org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.COLLECT_FAILSAFE;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.FAILSAFE_NO_PIXELS;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.LEAVE_WITH_2_PIXELS;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.Auto2_10BB.STROBOT.PLACE_STACK_PIXELS_BB;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.ANGLE_PARK_RIGHT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.ANGLE_SPIKE_LEFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.ANGLE_SPIKE_MID;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.ANGLE_SPIKE_RIGHT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PARK_RIGHT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PARK_RIGHT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PLACE_SPIKE_LEFT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PLACE_SPIKE_LEFT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PLACE_SPIKE_MID_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PLACE_SPIKE_MID_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PLACE_SPIKE_RIGHT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PLACE_SPIKE_RIGHT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PRELOAD_ANGLE_LEFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PRELOAD_LEFT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PRELOAD_MID_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PRELOAD_MID_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PRELOAD_RIGHT_X;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.PRELOAD_RIGHT_Y;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.STROBOT.PARK;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.STROBOT.PLACE_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.STROBOT.PLACE_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.AutoBLUEPreloadBB.STROBOT.WAIT_FOR_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.writtenCode.CaseDetectionPipeline;
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


@Config
@Autonomous(group = "Auto")

public class Auto2_10BLUE extends LinearOpMode {

    public static double forwardRight = 6.75;
    public static  double PRELOAD_LEFT_X = 35;
    public static  double PRELOAD_LEFT_Y = 35;
    public static  double PRELOAD_ANGLE_LEFT = 0;
    public static  double PLACE_SPIKE_LEFT_X = 51.5;
    public static  double PLACE_SPIKE_LEFT_Y = 46;
    public static  double ANGLE_SPIKE_LEFT = 0;
    public static  double PARK_LEFT_X = 43;
    public static  double PARK_LEFT_Y = 20;
    public static  double ANGLE_PARK_LEFT = 0;
    public static  double PRELOAD_MID_X = 25;
    public static  double PRELOAD_MID_Y = 25;
    public static  double PRELOAD_ANGLE_MID = 0;
    public static  double PLACE_SPIKE_MID_X = 51.5;
    public static  double PLACE_SPIKE_MID_Y = 40;
    public static  double ANGLE_SPIKE_MID = 0;
    public static  double PARK_MID_X = 43;
    public static  double PARK_MID_Y = 20;
    public static  double ANGLE_PARK_MID = 0;
    public static  double PRELOAD_RIGHT_X = 16;
    public static  double PRELOAD_RIGHT_Y = 33;
    public static  double PRELOAD_ANGLE_RIGHT = 0;
    public static  double PLACE_SPIKE_RIGHT_X = 51.5;
    public static  double PLACE_SPIKE_RIGHT_Y = 32;
    public static double ANGLE_SPIKE_RIGHT = 0;
    public static double PARK_RIGHT_X = 44;
    public static double PARK_RIGHT_Y = 20;
    public static double ANGLE_PARK_RIGHT = 0;
    public static final double GO_TO_STACK_X = 20f;
    public static final double GO_TO_STACK_Y = 11.5f;
    public static final double GO_TO_STACK_ANGLE = 1;
    public static final double COLLECT_STACK_X = -27f;
    public static final double COLLECT_STACK_Y = 11.5;
    public static final double PLACE_BB_LLH1_X = 20;
    public static final double PLACE_BB_LLH1_Y = 11;
    public static final int ANGLE_BB_LLH1 = 0;
    public static final double PLACE_BB_LLH2_X = 53.2;
    public static final double PLACE_BB_LLH2_Y = 33.5f;
    public static final int ANGLE_BB_LLH2 = 0;
    public static int timeout_1pixel = 3;
    public static int timeout_nopixel = 5;

    private void wait(int ms) {
        try{
            Thread.sleep(ms);
        }
        catch (InterruptedException ignored) {
            //nu face nimic
        }
    }


    enum STROBOT
    {
        START,
        PLACE_PURPLE_PIXEL,
        WAIT_FOR_PURPLE_PIXEL,
        PLACE_SPIKE_BACKDROP,
        PARK,
        RETRACT_LIFT,
        GO_COLLECT_PIXELS,
        LEAVE_WITH_2_PIXELS,
        PLACE_STACK_PIXELS_BB,
        GO_TO_STACK_FIRST,
        END_AUTO,
        FAILSAFE_NO_PIXELS,
        COLLECT_FAILSAFE
    }
    public static double timePlacePixel = 0.2;
    public static double delayLift = 0.4;
    public static double waitTimeBackDrop = 0.3;
    public static double timeOpenSlides = 0.4;
    boolean DID_FAILSAFE = false;
    ElapsedTime timerPunerePixel = new ElapsedTime();
    ElapsedTime timerLift = new ElapsedTime();
    ElapsedTime timerSlides = new ElapsedTime();
    ElapsedTime timeoutColectare = new ElapsedTime();
    ElapsedTime AutoTimer = new ElapsedTime();
    CaseDetectionPipeline cameraRecognition;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);

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
        pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.OPEN;

        TransferController transferController = new TransferController(
                intakeController,tubuleteController,sigurantaOuttakeController,robot);
        ScoringController scoringController = new ScoringController(pixel2Controller, sigurantaOuttakeController, parbrizController, rotateClawController);

        cataratController.update();
        avionController.update();
        extenderController.update(ExtenderController.extenderInit);
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
        cameraRecognition = new CaseDetectionPipeline(hardwareMap,telemetry,"blue");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);


        Pose2d startPose = new Pose2d(10, 62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectoryVelocityConstraint VELLLH = getVelocityConstraint(40, 5, 12.05);
        double nrCycles = 0;
        double howManyCycles = 1;
        TrajectorySequence PLACE_PRELOAD_LEFT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                .build();
        TrajectorySequence PLACE_SPIKE_LEFT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_LEFT.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y,Math.toRadians(ANGLE_SPIKE_LEFT)))
                .build();
        TrajectorySequence PARK_ROBOT_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .lineToLinearHeading(new Pose2d(PARK_LEFT_X, PARK_LEFT_Y,Math.toRadians(ANGLE_PARK_LEFT)))
                .strafeRight(5)
                .forward(10)
                .build();


        TrajectorySequence PLACE_PRELOAD_MID = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_MID_X, PRELOAD_MID_Y,Math.toRadians(PRELOAD_ANGLE_MID)))
                .build();
        TrajectorySequence PLACE_SPIKE_MID = drive.trajectorySequenceBuilder(PLACE_PRELOAD_MID.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_MID_X, PLACE_SPIKE_MID_Y,Math.toRadians(ANGLE_SPIKE_MID)))
                .build();

        TrajectorySequence PLACE_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_RIGHT_X, PRELOAD_RIGHT_Y,Math.toRadians(PRELOAD_ANGLE_RIGHT)))
                .back(forwardRight)
                .build();

        TrajectorySequence PLACE_SPIKE_RIGHT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_RIGHT.end())
                .setVelConstraint(VELLLH)
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
                .back(7)
                .lineToLinearHeading(new Pose2d(PARK_RIGHT_X, PARK_RIGHT_Y,Math.toRadians(ANGLE_PARK_RIGHT)))
                .build();

        double cazAuto = 1;
        while (!isStarted()&&!isStopRequested())
        {
            int c =  cameraRecognition.getCase();
            telemetry.addData("detected",c);
            cazAuto=c;
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int extenderCurrentPosition = robot.rightExtension.getCurrentPosition();

            switch (status)
            {
                case START:
                {
                    AutoTimer.reset();
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
                        status = STROBOT.WAIT_FOR_PURPLE_PIXEL;
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
                        status = STROBOT.PLACE_SPIKE_BACKDROP;
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
                        status = STROBOT.GO_TO_STACK_FIRST;
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
                        status = STROBOT.RETRACT_LIFT;
                    }
                    break;
                }
                case RETRACT_LIFT:
                {
                    if (timerLift.seconds()> 0.3 && (liftMotorController.currentStatus != LiftMotorController.LiftStatus.GOING_DOWN && liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT))
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.GOING_DOWN;
                    }
//                    else if(liftMotorController.currentPosition>=-120 && liftMotorController.currentStatus == LiftMotorController.LiftStatus.GOING_DOWN && liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT)
//                    {
//                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
//                    }
                    if (!drive.isBusy())
                    {
                        if (nrCycles > howManyCycles)
                        {
                            status = STROBOT.END_AUTO;
                            break;
                        }
                        status = STROBOT.GO_COLLECT_PIXELS;
                        timerSlides.reset();
                        drive.followTrajectorySequenceAsync(GO_COLLECT_STACK);
                    }
                    break;
                }
                case GO_COLLECT_PIXELS:
                {
                    if (timerSlides.seconds()>timeOpenSlides && extenderController.currentStatus != ExtenderController.ExtenderStatus.FAR)
                    {
                        extenderController.currentStatus = ExtenderController.ExtenderStatus.FAR;
                        collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_5;
                        if(nrCycles==2) collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_LOW;

                        intakeController.currentStatus = IntakeController.IntakeStatus.STACK;
                        timeoutColectare.reset();
                    }
                    if (robot.beamFront.getState() == false && robot.beamBack.getState() == false)
                    {
                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                        status = STROBOT.LEAVE_WITH_2_PIXELS;
                    }
                    else if(robot.beamBack.getState() == false)
                    {
                        if(nrCycles==1){
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_4;
                        }
                        if (timeoutColectare.seconds() > timeout_1pixel) {

                            pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.OPEN;
                            // Pun timpul pentru extendo
                            transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                            transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                            intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                            telemetry.addData("1 PIXEL FAILSAFE ACTIVATED",1);
                            telemetry.update();
                            drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                            status = STROBOT.PLACE_STACK_PIXELS_BB;
                        }
                    }
                    else if(robot.beamBack.getState() == true && robot.beamFront.getState() == true)
                    {
                        if (timeoutColectare.seconds() > timeout_nopixel) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_4;
                        }
                    }
                    break;
                }
                case FAILSAFE_NO_PIXELS:
                {

                    wait(500);
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.FAILSAFE;
                    intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE_AUTO;
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_4;
                    DID_FAILSAFE = true;
                    status = STROBOT.COLLECT_FAILSAFE;
                    break;
                }
                case COLLECT_FAILSAFE:
                {
                    wait(1000);
                    timeoutColectare.reset();
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.FAR;
                    intakeController.currentStatus = IntakeController.IntakeStatus.STACK;
                    if (robot.beamFront.getState() == false && robot.beamBack.getState() == false)
                    {
                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                        status = STROBOT.COLLECT_FAILSAFE;
                    }
                    else if(robot.beamBack.getState() == false)
                    {
                        if (timeoutColectare.seconds() > timeout_1pixel) {

                            pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.OPEN;
                            // Pun timpul pentru extendo
                            transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                            transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                            intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                            drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                            status = STROBOT.PLACE_STACK_PIXELS_BB;
                        }
                    }
                    else if(robot.beamBack.getState() == true && robot.beamFront.getState() == true)
                    {
                        if (timeoutColectare.seconds() > timeout_nopixel) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_LOW;
                            status=STROBOT.FAILSAFE_NO_PIXELS;
                        }
                    }
                    break;

                }
                case LEAVE_WITH_2_PIXELS:
                {
                    wait(300);
                    pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.OPEN;

                    // Pun timpul pentru extendo
                    transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                    intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                    status = STROBOT.PLACE_STACK_PIXELS_BB;
                }
                case PLACE_STACK_PIXELS_BB:
                {
                    if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT &&
                            transferController.currentStatus == TransferController.TransferStatus.INIT)
                    {
                        if(cazAuto==1) {
                            liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE1_C1;
                            if (nrCycles == 2) {
                                liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE2_C1;
                            }
                        }
                        else {
                            liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE1_C23;
                            if (nrCycles == 2) {
                                liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE2_C23;
                            }
                        }
                    }
                    if (!drive.isBusy() && (liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE1_C1 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE2_C1 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE1_C23 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE2_C23))
                    {
                        scoringController.currentStatus = ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        timerPunerePixel.reset();
                        status = STROBOT.GO_TO_STACK_FIRST;
                    }
                    break;
                }
            }
            if(AutoTimer.seconds()>28)
            {
                extenderController.currentStatus= ExtenderController.ExtenderStatus.INIT;
            }
            if(AutoTimer.seconds()>28 && status != STROBOT.PLACE_SPIKE_BACKDROP && status != STROBOT.PLACE_STACK_PIXELS_BB && status != STROBOT.GO_TO_STACK_FIRST)
            {
                liftMotorController.currentStatus = LiftMotorController.LiftStatus.GOING_DOWN;
            }
            if(robot.liftMotor.getCurrentPosition()>=-120 && liftMotorController.currentStatus == LiftMotorController.LiftStatus.GOING_DOWN)
            {
                liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
            }





            cataratController.update();
            avionController.update();
            extenderController.update(extenderCurrentPosition);
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
            telemetry.addData("AutoTimer", AutoTimer.seconds());
            telemetry.addData("Extender status", extenderController.currentStatus);
            telemetry.update();
        }
    }

}