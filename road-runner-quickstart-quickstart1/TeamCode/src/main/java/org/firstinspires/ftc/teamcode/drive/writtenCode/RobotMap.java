package org.firstinspires.ftc.teamcode.drive.writtenCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.hardware.Sensor;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;



import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * Aici punem tot hardware-ul ca sa fie mult mai usor de accesat
 * De asemenea vom crea o instanta din aceasta clasa pentru a forma
 * controllerele
 */
public class RobotMap {
    /// Initializam toate motoarele , servourile si senzorii aici
    /// Ca sa putem da mult mai usor da debug la probleme de config
    /// Probleme de config sunt cand nu puneti acelasi nume ca cel din aplicatie in cod.
    public DcMotorEx liftMotor = null;
    public DcMotorEx intakeMotor = null;
    public DcMotorEx rightExtension = null;
    public DcMotorEx leftExtension = null;
    public Servo airplaneServo = null;

    public Servo sigurantaOuttake = null;
    public Servo clawRotate = null;
    public Servo parbrizOuttake = null;
    public Servo pixel2Outtake = null;
    public Servo forbarOuttake = null;
    public Servo forbarCutieIntake = null;
    public Servo leftTransferServo = null;
    public Servo rightTransferServo = null;

    public Servo forbarIntake = null;
    public Servo hangingLeft = null;
    public Servo hangingRight=  null;
    public RobotMap(HardwareMap Init)
    {
        /// Motoare

        liftMotor = Init.get(DcMotorEx.class , "liftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor = Init.get(DcMotorEx.class , "intakeMotor");

        /// Motoarele de la extensie
        leftExtension = Init.get(DcMotorEx.class, "leftExtension");
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Important ca sa utilizam encoderele
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightExtension = Init.get(DcMotorEx.class , "rightExtension");
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Important ca sa utilizam encoderele
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// Este clar ca motorul din stanga si cel din dreapta trebuie sa se invarteasca invers
        /// Astfel ca sa putem da aceeasi pozitie in ambele dam reverse din cod la pozitia unuia.
        rightExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        /// Servo-uri

        /// Control Hub
        airplaneServo = Init.get(Servo.class,"airplane");
        sigurantaOuttake = Init.get(Servo.class, "sigurantaOuttake"); /// Tot timpul inchis dupa transfer
        clawRotate = Init.get(Servo.class, "clawRotate");
        parbrizOuttake = Init.get(Servo.class , "parbrizOuttake"); /// Pentru cand vreti sa puneti pixeli orizontal
        pixel2Outtake = Init.get(Servo.class , "pixel2Outtake"); /// Pentru al doilea pixel , de obicei deschis
        forbarOuttake = Init.get(Servo.class, "forbarOuttake");

        /// Expansion Hub
        forbarCutieIntake = Init.get(Servo.class, "forbarCutieIntake");
        leftTransferServo = Init.get(Servo.class, "leftTransferServo");
        rightTransferServo = Init.get(Servo.class,"rightTransferServo");
        forbarIntake = Init.get(Servo.class, "forbarIntake");
        hangingLeft = Init.get(Servo.class, "hangingLeft");
        hangingRight = Init.get(Servo.class, "hangingRight");

    }
}