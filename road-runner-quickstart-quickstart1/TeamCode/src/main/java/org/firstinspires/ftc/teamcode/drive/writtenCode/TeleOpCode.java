/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.BoxController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectForbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ParbrizController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.Pixel2Controller;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.SigurantaOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TubuleteController;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpCode", group="Linear OpMode")
public class TeleOpCode extends LinearOpMode {
    // Putine disclaimere :

    // Ce este loop time-ul?
    // Daca va uitati avem un while in runOpMode cu while(opModeIsActive())
    // Cu cat acest while se evalueaza mai repede cu atat robotul se va misca mai bine pentru ca
    // partile din hard vor fi updatate mai repede.

    // Comentariile in verde de deasupra functiilor se numesc javadov
    // Daca nu stiti ce face o functie puteti da hover peste ea si o sa va apara exact ce scrie in
    // comentariile verzi, try it out!

    /**
     * Pune running mode-ul la motoare
     * Recomand sa va uitati pe ce runningMode-uri sunt disponibile.
     * Pentru sasiu de obicei e bine sa folositi RUN_WITHOUT_ENCODER
     * @param leftFront - motorul din stanga fata
     * @param rightFront - motorul din dreapta fata
     * @param leftBack - motorul din stanga spate
     * @param rightBack - motorul din dreapta spate
     * @param runningMode - ce RunMode vor avea motoarele
     */
    public void setMotorRunningMode(DcMotor leftFront , DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode)
    {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }
    /**
     * Pune zeroPowerBehaviour la motoare
     * De obicei se foloseste BREAK
     * @param leftFront - motorul din stanga fata
     * @param rightFront - motorul din dreapta fata
     * @param leftBack - motorul din stanga spate
     * @param rightBack - motorul din dreapta spate
     * @param zeroPowerBehavior - ce RunMode vor avea motoarele
     */
    public void setMotorZeroPowerBehaviour(DcMotor leftFront,DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Aici punem puterea in motoare ca sa se miste robot centric
     * Pentru mai multe detalii la cum functioneaza citit gm0.
     * @param leftFront - Motorul stanga fata
     * @param leftBack - Motorul stanga spate
     * @param rightFront - Motorul dreapta fata
     * @param rightBack - Motorul dreapta spate
     */
    public void robotCentricDrive(DcMotor leftFront , DcMotor leftBack,
                                  DcMotor rightFront ,DcMotor rightBack)
    {
        /// O sa va intrebati cum putem accesa gamepad1 si gamepad2 ?
        /// Probabil sunt variabile globale , n-ar trebui sa va faceti multe griji

        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;


        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RobotMap robot = new RobotMap(hardwareMap);

        IntakeController intakeController = new IntakeController(robot);
        CollectForbarController collectForbarController = new CollectForbarController(robot);
        TubuleteController tubuleteController = new TubuleteController(robot);
        BoxController boxController = new BoxController(robot);
        Pixel2Controller pixel2Controller = new Pixel2Controller(robot);
        ParbrizController parbrizController = new ParbrizController(robot);
        SigurantaOuttakeController sigurantaOuttakeController = new SigurantaOuttakeController(robot);
        LiftMotorController liftMotorController = new LiftMotorController(robot);
        TransferController transferController = new TransferController(
                intakeController,tubuleteController,sigurantaOuttakeController,robot);


        liftMotorController.update();
        intakeController.update();
        collectForbarController.update();
        tubuleteController.update();
        boxController.update();
        transferController.update();
        pixel2Controller.update();
        parbrizController.update();
        sigurantaOuttakeController.update();

        // Declaram motoarele din drive aici
        // Numele motoarelor sunt destul de evidente
        // Ce este intre ghilimele trebuie sa fie acelasi nume ca in config
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");

        // Dam toate cele patru motoare si le punem sa mearga fara encodere
        // deoarece asa se misca mai bine
        // Puteti citi documentatia si comentarii pe reddit dar long story short
        // daca merge cu encodere
        // Programul le va folosi pentru a calcula ce viteza sa le dea si asta nu
        // prea e bine pentru loop time.
        setMotorRunningMode(leftFront,leftBack,rightFront,rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /// Foarte important pentru sasiul de mecanum.
        /// Cititi pe gm0 de ce este necesar
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /// Aici facem ca motoarele cand nu primesc nicio comanda sa-si tina
        /// pozitia pe care o aveau inainte. Asta va permite sa nu fiti impinsi foarte usor
        /// de alti roboti
        setMotorZeroPowerBehaviour(leftFront,leftBack,rightFront,rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);

        /// Este clar ca currentGamepad1 este de la primul driver
        /// currentGamepad2 este de la al doilea driver
        /// De ce avem nevoie de previousGamepad1 si previousGamepad2?
        /// Raspunsul scurt este pentru ca avem nevoie sa vedem cand am apasat o data pe buton
        /// Pentru mai multe explicatii uitati-va pe gm0.

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) return;

            /// Updatam motoarele cu puterile necesare ca sa miscam sasiul
            /// Vei vedea ca folosim aceeasi chestie ca pe gm0
            /// Doar am pus-o in alta functie pentru ca nu prea o sa o modificam des
            robotCentricDrive(leftFront,leftBack,rightFront,rightBack);



            /// Destul de clar ce fac aceste linii
            /// Uitati-va pe gm0 pentru explicatii
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /// Am apasat dpad_left ->
            // 4Bar ul intake se duce in pozitie
            // Colectare incepe.
            // Switch/Toggle - Continous press
            // Dpad_left- > 4Bar-ul se duce , incepe colectare , Dpad_left - > 4barul se intoarce (Switch)
            // Cat timp e apasat se intampla ce s-a scris mai sus (Continous)

            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
            {
                if (intakeController.currentStatus == IntakeController.IntakeStatus.STOP)
                {
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.COLECTARE;
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT_DRIVE;
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                }
                else
                {
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.BLOCARE;
                    intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                    collectForbarController.currentStatus = CollectForbarController.CollectStatus.INIT;
                }
            }

            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)
            {
                intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
            }

            /// Apas pe right_trigger
            /// Se blocheaza + Reverse colectare
            /// Flip
            /// Asteapta
            /// Transfer
            /// Revine la intake

            if (currentGamepad2.right_trigger>0 && previousGamepad2.right_trigger==0)
            {
                if (transferController.currentStatus == TransferController.TransferStatus.INIT &&
                    liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT)
                {
                    // Cand fac transferul vreau sa fie inchis parbrizul
                    parbrizController.currentStatus = ParbrizController.ParbrizStatus.CLOSED;
                    // Sa fie deschis pixel2 ca sa nu impiedice colectarea
                    pixel2Controller.currentStatus = Pixel2Controller.Pixel2Status.OPEN;
                    transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                }
            }
            /// Lift going to low
            if (currentGamepad2.a && !previousGamepad2.a)
            {
                if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT)
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.LOW;
                }
                else
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                }
            }
            /// Lift going to mid
            if (currentGamepad2.x && !previousGamepad2.x)
            {
                if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT)
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.MID;
                }
                else
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                }
            }
            /// Lift going to high
            if (currentGamepad2.y && !previousGamepad2.y)
            {
                if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT)
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.HIGH;
                }
                else
                {
                    liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                }
            }
            liftMotorController.update();
            sigurantaOuttakeController.update();
            pixel2Controller.update();
            parbrizController.update();
            intakeController.update();
            collectForbarController.update();
            boxController.update();
            transferController.update();
        }
    }
}