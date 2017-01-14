package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * Created by Howard on 12/10/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "ShootRotationTest", group = "Tests")
@Disabled

public class ShootRotationTest extends LinearOpMode{
    boolean missed = false,  detectRed1 = false;
    boolean missedLineUp = false;
    boolean shootFirst = false;

    final int topSensorID = 0x3c;
    final int bottomSensorID = 0x44;

    int shootTime = 2500;

    // Distances
    int betweenBeacon = 30; // far beacon distance
    int bbalt = 36; // near beacon distance
    int bbalt2 = 5; // miss first beacon distance

    // Angles
    final double pickUpRotation = 152;
    double shootRotation = 108; // first beacon shoot rotation near
    double shootRotation2 = 43; // near shoot rotation
    double sralt = 39.5; // far shoot rotation
    double sralt3 = 90; // first beacons shoot rotation far
    double parallelAngle; // angle perpendicular to start wall (should be approx equal to wallAngle)
    double angle = -35; // normal white line rotation angle
    double sfAngle = -125; // shoot first alignment angle
    double angleAlt = -55; // shoot first rotation angle
    double wallAngle; // angle parallel to beacon wall

    // Powers
    double shootPower = 0.63; // normal shoot power
    double spalt = 0.6; // short shoot power (from first beacon)
    double spalt2 = .8; // shoot first (from starting position)
    double bpPower = 0.11; // beacon pressing driveTrain power

    // Hardware
    double voltageLevel;
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
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void runOpMode() {
        initialize();
        double mc7 = hardwareMap.voltageSensor.get("Motor Controller 7").getVoltage();
        double mc6 = hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage();
        double mc3 = hardwareMap.voltageSensor.get("Motor Controller 3").getVoltage();
        double mc2 = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        voltageLevel = (mc7 + mc6 + mc3 + mc2) / 4;
        flywheelTask.voltage = voltageLevel;
        telemetry.addData("Ready!", "");
        telemetry.update();
        waitForStart();
        correctionStrafe();
        drivePushButton2();
        moveFromWall2();

    }

    public void initialize(){
        timer.reset();
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sweeper1 = hardwareMap.crservo.get("sweeper1");
        sweeper2 = hardwareMap.crservo.get("sweeper2");
        //sweeper = new VOISweeper(sweeper1, sweeper2);
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
        button.setPosition(0);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTask = new FlywheelTask(this);
        flywheelTask.start();
    }

    public void correctionStrafe() {
        correctionStrafe(0.5);
    }

    public void correctionStrafe(double seconds) {
        driveTrain.moveRightNInch(0.2, 5, seconds, false, true);
    }

    public void moveFromWall2(){
        telemetry.addData("moveFromWall2", "");
        telemetry.update();
        driveTrain.moveLeftNInch(0.6, 3, 10, false, true);
        double offset = VOIImu.subtractAngles(imu.getAngle(), wallAngle);
        shootRotation2 = VOIImu.subtractAngles(shootRotation2, offset);
        driveTrain.rotateDegreesPrecision(shootRotation2);
        if (shootFirst) {
            driveTrain.moveBackwardNInch(0.5, 50, 10, false, true);
        }
        else {
            sleep(500);
            driveTrain.moveBackwardNInch(0.10, 1, 3, false, false);
            driveTrain.moveBackwardNInch(0.2, 12, 3, false, false);
            driveTrain.moveBackwardNInch(0.10, 5, 3, false, true);
            flywheelTask.setFlywheelPow(shootPower);
            sleep(2000);
            sweeper.setPower(1);
            sleep(shootTime);
            coolDown();
            hitCapBall2();
        }
    }

    public void coolDown() {
        sweeper.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    public void hitCapBall2(){
        telemetry.addData("hitCapBall2", "");
        telemetry.update();
        driveTrain.moveBackwardNInch(0.4, 45, 10, false, true);
    }

    public void drivePushButton2() {
        telemetry.addData("drivePushButton2", "");
        telemetry.update();
        int timeo = 5000;
        boolean timeAdded = false;
        //driveTrain.moveForwardNInch(0.45, betweenBeacon, 10, false, false);
        //flywheelTask.setFlywheelPow(shootPower);
        //pause();
        correctionStrafe(1);
        wallAngle = imu.getAngle();
        boolean detectColor = false;
        driveTrain.powerAllMotors(bpPower);
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
            if (timer.time() > 30) {
                // determine which side of beacon
                if (voiColorSensorTop.isRed() && !timeAdded){
                    shootRotation2 = sralt;
                    timeAdded = true;
                    timeo += 1000;
                }
                detectColor = voiColorSensorTop.isBlue();
                timer.reset();
            }
            if (timeout.time() >= timeo) {
                //giveUpSecond();
            }
        }
        correctionStrafe(0.5);
        pushButton();
        //driveTrain.powerAllMotors(0.08);
        //while (opModeIsActive() && voiColorSensorTop.isBlue()) {}
        //driveTrain.stopAll();
    }

    public void pushButton(){
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
        sleep(500);
    }



}
