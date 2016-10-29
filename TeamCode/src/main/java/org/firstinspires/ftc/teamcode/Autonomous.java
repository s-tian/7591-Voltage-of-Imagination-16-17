package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Tests")

/**
 * Created by Howard on 9/21/16.
 */
public class Autonomous extends LinearOpMode {
    ModernRoboticsI2cGyro gyro;

    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        driveTrain.getTicks();
        driveTrain.moveRightNInch(0.5,20, 10, true);
        sleep(1000);
        driveTrain.moveLeftNInch(0.5,20, 10, true);
        driveTrain.stopAll();
        driveTrain.getTicks();
    }

    public void initialize(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        gyro.resetZAxisIntegrator();
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,gyro,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

