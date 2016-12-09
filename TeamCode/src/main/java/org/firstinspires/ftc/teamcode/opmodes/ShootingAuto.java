package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;

/**
 * Created by Howard on 12/8/16.
 */

public class ShootingAuto extends LinearOpMode{
    final int topSensorID = 0x3c;
    final int bottomSensorID = 0x44;
    double shootPower = 0.8;
    ColorSensor colorSensorTop, colorSensorBottom;
    VOIColorSensor voiColorSensorTop, voiColorSensorBottom;
    Servo forkLeft, forkRight, button;
    VOIImu imu;
    VOISweeper sweeper;
    CRServo sweeper1, sweeper2;
    BNO055IMU adaImu;
    public FlywheelTask flywheelTask;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft; //sweeper, conveyor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void runOpMode() {
        initialize();
        flywheelTask.setFlywheelPow(-0.3);
    }
    public void initialize() {
        timer.reset();
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sweeper1 = hardwareMap.crservo.get("sweeper1");
        sweeper2 = hardwareMap.crservo.get("sweeper2");
        sweeper = new VOISweeper(sweeper1, sweeper2);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        colorSensorBottom = hardwareMap.colorSensor.get("colorBottom");
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        colorSensorBottom.setI2cAddress(I2cAddr.create8bit(topSensorID));//maybe create8bit
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(bottomSensorID));
        voiColorSensorTop = new VOIColorSensor(colorSensorTop, this);
        voiColorSensorBottom = new VOIColorSensor(colorSensorBottom, this);
        button = hardwareMap.servo.get("button");
        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        forkLeft.setPosition(0.8);
        forkRight.setPosition(0.12);
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        button.setPosition(0.1);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        flywheelTask.start();

    }

    public void pickUp() {
        // pick up third ball

    }
    public void shoot() {
        flywheelTask.setFlywheelPow(shootPower);
        sleep(2000);
        sweeper.setPower(1);
    }




}
