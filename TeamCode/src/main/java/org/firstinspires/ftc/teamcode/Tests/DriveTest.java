package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;

/**
 * Created by Howard on 10/23/16.
 */
@TeleOp(name = "Drive Test", group = "Test")

public class DriveTest extends LinearOpMode {
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight, sweeper;
    VOIImu imu;
    BNO055IMU adaImu;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        driveTrain.rotateDegrees((int)(90*0.7), true);

    }
    public void initialize(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = new VOIImu(adaImu);
        driveTrain = new MecanumDriveTrain(backLeft, backRight, frontLeft, frontRight,imu, this);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void testTicks(){
        driveTrain.getTicks();
        int startFL = frontLeft.getCurrentPosition();
        int startBL = backLeft.getCurrentPosition();
        int startFR = frontRight.getCurrentPosition();
        int startBR = backRight.getCurrentPosition();
//        System.out.println("FrontLeft: " + frontLeft.getCurrentPosition());
//        System.out.println("BackLeft: " + backLeft.getCurrentPosition());
//        System.out.println("FrontRight: " + frontLeft.getCurrentPosition());
//        System.out.println("FrontLeft: " + frontLeft.getCurrentPosition());
        driveTrain.powerAllMotors(0.5);
        sleep(1750);
        driveTrain.stopAll();
        System.out.println("Front Left: " + (frontLeft.getCurrentPosition()-startFL));
        System.out.println("Back Left: " + (backLeft.getCurrentPosition()-startBL));
        System.out.println("Front Right: " + (frontRight.getCurrentPosition()-startFR));
        System.out.println("Back Right : " + (backRight.getCurrentPosition()-startBR));
    }
}
