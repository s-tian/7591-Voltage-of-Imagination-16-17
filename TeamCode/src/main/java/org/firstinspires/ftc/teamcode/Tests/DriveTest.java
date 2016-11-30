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
        int start = backRight.getCurrentPosition();
        System.out.println(start);
        driveTrain.moveRightTicksWithEncoders(0.5, 2000, 10,false);
        sleep(1000);
        System.out.println(backRight.getCurrentPosition());

    }
    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        //adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu = new VOIImu(adaImu);
        driveTrain = new MecanumDriveTrain(backLeft, backRight, frontLeft, frontRight, imu, this);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}