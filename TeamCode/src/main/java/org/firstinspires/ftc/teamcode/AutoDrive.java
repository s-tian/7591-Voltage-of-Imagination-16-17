package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

import static java.lang.Thread.sleep;

/**
 * Created by Stephen on 10/4/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoDrive", group = "Tests")
public class AutoDrive extends LinearOpMode {
    static int delay = 200;
    static boolean REDTEAM = false;
    static final int topSensorID = 0x3c;
    static final int bottomSensorID = 0x44;
    static int angle = 45;
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensorTop, colorSensorBottom;
    VOIColorSensor voiColorSensorTop, voiColorSensorBottom;
    Servo gate, button;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, sweeper, conveyor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        lineUpToWall();
        drivePushButton();
        drivePushButton2();
        checkFirst();
        moveFromWall();
        coolDown();
    }
    public void pause(){
        driveTrain.stopAll();
        sleep(delay);
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
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
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
        gate = hardwareMap.servo.get("gate");
        button = hardwareMap.servo.get("button");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        gyro.resetZAxisIntegrator(); //address is 0x20
        button.setPosition(0);
        gate.setPosition(0.4);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,gyro,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void lineUpToWall() throws InterruptedException{
        driveTrain.moveForwardNInch(0.7,50);
        pause();
        driveTrain.powerAllMotors(0.3);
        boolean detectColor = false;
        timer.reset();
        while(!detectColor && opModeIsActive()){
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            if (timer.time() > 30){
                detectColor = voiColorSensorBottom.isWhite();
                telemetry.addData("Color: ", red + " " + green + " " + blue);
                updateTelemetry(telemetry);
            }
        }
        pause();
        // align with wall
        driveTrain.rotateDegrees((int)(-angle*0.7));
        pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 20, 3);
        pause();
    }
    public void drivePushButton() throws InterruptedException {
        // move backwards to get behind beacon
        driveTrain.moveBackwardNInch(0.4,10);
        // move forward until beacon detected
        pause();
        driveTrain.moveRightNInch(0.2,1,3);
        pause();
        driveTrain.powerAllMotors(0.2);
        boolean detectColor = false;
        timer.reset();
        if (REDTEAM) {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed();
                    timer.reset();
                }
            }
        } else{
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isBlue();
                    timer.reset();
                }
            }
        }
        pause();
        // move forward to align button pusher with beacon button and push
        driveTrain.moveForwardNInch(0.2,3);
        pause();
        pushButton();
    }
    public void drivePushButton2() throws InterruptedException{
        driveTrain.moveForwardNInch(0.5,25);
        pause();
        boolean detectColor = false;
        driveTrain.powerAllMotors(0.2);
        timer.reset();
        if (REDTEAM) {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed();
                    timer.reset();
                }
            }
        } else{
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isBlue();
                    timer.reset();
                }
            }
        }
        pause();
        driveTrain.moveRightNInch(0.2,1,3);
        pause();
        driveTrain.moveForwardNInch(0.2,3);
        pause();
        pushButton();
    }
    public void pushButton() {
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
        sleep(500);
    }
    public void moveFromWall() throws InterruptedException{
        setFlywheelPower(0.7);
        driveTrain.moveLeftNInch(0.6, 10, 10);
        pause();
        driveTrain.rotateDegreesPrecision(90);
        pause();
        driveTrain.setMotorPower(conveyor, 0.3);
        sleep(5000);
    }
    public void checkFirst()throws InterruptedException{
        driveTrain.moveBackwardNInch(0.5,25);
        pause();
        driveTrain.moveRightNInch(0.2,1,2);
        pause();
        driveTrain.powerAllMotors(-0.2);
        boolean detectColor = false;
        boolean wrongColor = false;
        DETECTINGCOLOR:
        if (REDTEAM) {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed();
                    if (voiColorSensorTop.isBlue()){
                        driveTrain.stopAll();
                        wrongColor = true;
                        break DETECTINGCOLOR;
                    }
                    timer.reset();
                }
            }
        } else{
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isBlue();
                    if (voiColorSensorTop.isRed()){
                        driveTrain.stopAll();
                        wrongColor = true;
                        break DETECTINGCOLOR;
                    }
                    timer.reset();
                }
            }
        }
        if (wrongColor){
            pause();
            pushButton();
        }
        pause();
    }
    public void coolDown(){
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000){}
        setFlywheelPower(0.4);
        driveTrain.setMotorPower(conveyor, 0.15);
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000){}
        setFlywheelPower(0);
        driveTrain.setMotorPower(conveyor, 0);
    }
    public void setFlywheelPower(double power){
        driveTrain.setMotorPower(flywheelLeft, power);
        driveTrain.setMotorPower(flywheelRight, power);
    }

}
