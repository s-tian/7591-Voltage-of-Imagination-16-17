package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodes.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;


/**
 * Created by bunnycide on 11/21/16.
 */

@TeleOp(name = "Drive Test", group = "Test")

public class DriveTest extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    VOIImu imu;
    BNO055IMU adaImu;
    MecanumDriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        System.out.println(1);
        System.out.println("FL "  + frontLeft.getCurrentPosition());
        System.out.println("FR "  + frontRight.getCurrentPosition());
        System.out.println("BL "  + backLeft.getCurrentPosition());
        System.out.println("BR "  + backRight.getCurrentPosition());
        driveTrain.strafeRight(1);
        sleep(500);
        System.out.println(2);
        System.out.println("FL "  + frontLeft.getCurrentPosition());
        System.out.println("FR "  + frontRight.getCurrentPosition());
        System.out.println("BL "  + backLeft.getCurrentPosition());
        System.out.println("BR "  + backRight.getCurrentPosition());
        sleep(2000);
        System.out.println(3);
        System.out.println("FL "  + frontLeft.getCurrentPosition());
        System.out.println("FR "  + frontRight.getCurrentPosition());
        System.out.println("BL "  + backLeft.getCurrentPosition());
        System.out.println("BR "  + backRight.getCurrentPosition());
        driveTrain.stopAll();


    }
    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        driveTrain = new MecanumDriveTrain(backLeft, backRight, frontLeft, frontRight, this);

        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}