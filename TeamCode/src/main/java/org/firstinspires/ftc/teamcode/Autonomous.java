package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "Tests")

/**
 * Created by Howard on 9/21/16.
 */
public class Autonomous extends LinearOpMode {
    ModernRoboticsI2cGyro gyro;

    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    public void runOpMode() throws InterruptedException{
        initialize();
        waitForStart();
        driveTrain.rotateClockwiseDegrees(90);
    }
    public void initialize(){

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        gyro.resetZAxisIntegrator();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,gyro,this);
    }
}

