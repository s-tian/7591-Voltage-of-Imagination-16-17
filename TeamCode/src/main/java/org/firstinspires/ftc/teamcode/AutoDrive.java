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
        System.out.println("Hello world");
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
    public void initialize(){
        timer.reset();
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        //pause();
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
        //pause();
        // align with wall
        driveTrain.rotateDegrees((int)(-angle*0.7));
        //pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 20, 10);
        //pause();
    }
    public void drivePushButton() throws InterruptedException {
        // move backwards to get behind beacon
        driveTrain.moveBackwardNInch(0.4,5);
        // move forward until beacon detected

        //pause();
        driveTrain.powerAllMotors(0.2);
        boolean detectColor = false;
        boolean oppositeColor = false;
        int counter = 0;
        int initialTicks = backRight.getCurrentPosition();
        timer.reset();
        if (REDTEAM) {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed();
                    timer.reset();
                    if (voiColorSensorTop.isBlue()) oppositeColor = true;
                    if (oppositeColor && !voiColorSensorTop.isBlue() && !voiColorSensorTop.isRed()){
                        counter++;
                        System.out.println("BAD: " + counter);
                    }
                    if (counter > 80){
                        //pause();
                        return;
                    }
                    if (backRight.getCurrentPosition()-initialTicks > 1200){
                        //pause();
                        return;
                    }
                }
            }
        } else{
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isBlue();
                    timer.reset();
                    if (voiColorSensorTop.isRed()) oppositeColor = true;
                    if (oppositeColor && !voiColorSensorTop.isBlue() && !voiColorSensorTop.isRed()){
                        counter++;
                        System.out.println("BAD: " + counter);
                    }
                    if (counter > 50){
                        return;
                    }
                    if (backRight.getCurrentPosition()-initialTicks > 1200){
                        //pause();
                        return;
                    }
                }
            }
        }
        //pause();
        // move forward to align button pusher with beacon button and push
        driveTrain.moveForwardNInch(0.2,3);
        //pause();
        correctionStrafe();
        //pause();
        pushButton();
    }
    public void drivePushButton2() throws InterruptedException{
        driveTrain.moveForwardNInch(0.5,25);
        pause();
        boolean detectColor = false;
        driveTrain.powerAllMotors(0.2);
        timer.reset();
        int counter = 0;

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
        driveTrain.moveRightNInch(0.2,1,0.5);
        pause();
        driveTrain.moveForwardNInch(0.2,3);
        pause();
        pushButton();
    }
    public void pushButton(){
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
        sleep(500);
    }
    public void moveFromWall() throws InterruptedException{
        setFlywheelPower(1);
        sweeper.setPower(1);
        System.out.println("Sweeper: " + sweeper.getPower());
        driveTrain.moveLeftNInch(0.6, 8, 10);
        //pause();
        driveTrain.rotateDegreesPrecision(90);
        //pause();
        System.out.println("Sweeper: " + sweeper.getPower());
        sweeper.setPower(1);
        System.out.println("Sweeper: " + sweeper.getPower());

        driveTrain.setMotorPower(conveyor, 0.3);

        sleep(8000);
    }
    public void checkFirst()throws InterruptedException{
        driveTrain.moveBackwardNInch(0.5,45);
        //pause();
        correctionStrafe();
        //pause();
        driveTrain.powerAllMotors(-0.2);
        boolean detectColor = false;
        boolean wrongColor = false;
        DETECTING_COLOR:
        if (REDTEAM) {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed();
                    if (voiColorSensorTop.isBlue()){
                        driveTrain.stopAll();
                        wrongColor = true;
                        break DETECTING_COLOR;
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
                        break DETECTING_COLOR;
                    }
                    timer.reset();
                }
            }
        }
        if (wrongColor){
            System.out.println("WRONG COLOR");
            //pause();
            correctionStrafe();
            //pause();
            pushButton();
        }
        else {
            boolean blue = voiColorSensorTop.isBlue();
            boolean red = false;
            System.out.println("BLUE2 " + blue);
            while (opModeIsActive() && blue){
                if (timer.time()>30){
                    blue = voiColorSensorTop.isBlue();
                    timer.reset();
                }
                System.out.println("blue");
            }

            while (opModeIsActive() && !blue && !red){
                if (timer.time()>30){
                    blue = voiColorSensorTop.isBlue();
                    red = voiColorSensorTop.isRed();
                    timer.reset();
                }
                System.out.println("blank");
            }
            if (voiColorSensorTop.isRed())
                goBackAndPress();
        }
        //pause();
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
        driveTrain.setMotorPower(sweeper, 0);
    }
    public void setFlywheelPower(double power){
        driveTrain.setMotorPower(flywheelLeft, power);
        driveTrain.setMotorPower(flywheelRight, power);
    }
    public void goBackAndPress() throws InterruptedException{
        driveTrain.powerAllMotors(0.3);
        while (opModeIsActive() && !voiColorSensorTop.isBlue()){}
        //pause();
        driveTrain.moveForwardNInch(0.3,3);
        //pause();
        correctionStrafe();
        //pause();
        pushButton();
        //pause();
    }
    public void correctionStrafe() throws InterruptedException{
        driveTrain.moveRightNInch(0.2,1,0.5);
    }

}
