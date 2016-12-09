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
        // MC 7 has frontLeft motor, MC 6 is back motors
        System.out.println("MC 7: " + hardwareMap.voltageSensor.get("Motor Controller 7").getVoltage());
        System.out.println("MC 6: " + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage());
        System.out.println("MC 3: " + hardwareMap.voltageSensor.get("Motor Controller 3").getVoltage());
        System.out.println("MC 2: " + hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage());


        frontLeft.setPower(1);
        frontRight.setPower(1);
        sleep(2000);
        System.out.println("after");
        System.out.println("MC 7: " + hardwareMap.voltageSensor.get("Motor Controller 7").getVoltage());
        System.out.println("MC 6: " + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage());
        System.out.println("MC 3: " + hardwareMap.voltageSensor.get("Motor Controller 3").getVoltage());
        System.out.println("MC 2: " + hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage());

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