package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

import static java.lang.Thread.sleep;

/**
 * Created by Stephen on 10/4/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoDrive", group = "Tests")
public class AutoDrive extends LinearOpMode {
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensorBottom;
    ColorSensor colorSensorTop;
    VOIColorSensor voiColorSensorBottom;
    VOIColorSensor voiColorSensorTop;
    Servo gate, button;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        lineUpToWall();
        drivePushButton();
        pushButton();
    }


    public void testColor(){
        while(opModeIsActive()){
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            telemetry.addData("Bottom: ", red + " " + green + " " + blue);
            telemetry.addData("Top: ", colorSensorTop.red() + " " + colorSensorTop.green() + " " + colorSensorTop.blue());
            telemetry.addData("White: ", voiColorSensorBottom.isWhite() || voiColorSensorBottom.isWhite());
            System.out.println("Top: " + colorSensorTop.red() + " " + colorSensorTop.green() + " " + colorSensorTop.blue());
            System.out.println("Bottom: " + red + " " + green + " " + blue);
            System.out.println("White: " + (voiColorSensorTop.isWhite() || (voiColorSensorBottom.isWhite())));
            updateTelemetry(telemetry);
        }
    }
    public void initialize(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        colorSensorBottom = hardwareMap.colorSensor.get("colorBottom");
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        final int topSensorID = 0x3c;
        final int bottomSensorID = 0x44;
        colorSensorBottom.setI2cAddress(I2cAddr.create8bit(topSensorID));//maybe create8bit
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(bottomSensorID));
        voiColorSensorBottom = new VOIColorSensor(colorSensorBottom);
        voiColorSensorTop = new VOIColorSensor(colorSensorTop);
        gate = hardwareMap.servo.get("gate");
        button = hardwareMap.servo.get("button");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        while(gyro.isCalibrating()) {

        }
        gyro.resetZAxisIntegrator(); //address is 0x20
        button.setPosition(0);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,gyro,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void lineUpToWall() throws InterruptedException{
        driveTrain.powerAll(0.3);
        while(!voiColorSensorBottom.isWhite() && opModeIsActive()){
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            telemetry.addData("Color: ", red + " " + green + " " + blue);
            updateTelemetry(telemetry);
        }
        driveTrain.rotateDegrees(-50);
        driveTrain.moveRightNInch(0.4, 20, 3);
        driveTrain.moveLeftNInch(0.4, 0.5,2);
    }
    public void drivePushButton() throws InterruptedException {
        driveTrain.moveBackwardNInch(0.5,0.3);
        driveTrain.powerAll(0.5);
        while(!voiColorSensorTop.isBlue()){
        }
        driveTrain.moveForwardNInch(0.5,3);
        driveTrain.stopAll();
        pushButton();
    }
    public void pushButton() {
        button.setPosition(1.0);
        sleep(500);
        button.setPosition(0);
    }
}
