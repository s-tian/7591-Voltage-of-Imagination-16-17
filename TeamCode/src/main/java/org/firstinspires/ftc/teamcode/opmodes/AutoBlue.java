package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;

import static java.lang.Thread.sleep;

/**
 * Created by Stephen on 10/4/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoBlue", group = "Tests")

public class AutoBlue extends LinearOpMode {
    static int delay = 200;
    boolean missed = false, pickUp = false, detectRed1 = false;
    static int shootRotation = 95;
    static int shootRotation2 = 40                    ;
    static int sralt = 35;
    static final int capBallRotation = -180;
    static final int pickUpRotation = 152;
    static final boolean REDTEAM = false;
    static final int topSensorID = 0x3c;
    static final int bottomSensorID = 0x44;
    static int betweenBeacon = 28;
    static int angle = 40;
    static double shootPower = 0.8;
    //ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensorTop, colorSensorBottom;
    VOIColorSensor voiColorSensorTop, voiColorSensorBottom;
    Servo gate, button;
    VOIImu imu;
    BNO055IMU adaImu;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, sweeper, conveyor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void runOpMode() {
        System.out.println("Hello world");
        initialize();
        options();
        waitForStart();
        if (pickUp){
            pickUpBall();
            lineUpToWall(40);
        } else {
            lineUpToWall(50);
        }
        drivePushButton();
        //firstBeaconOptimization();
        drivePushButton2();
        //checkFirst();
        if (missed){
            checkFirst();
            moveFromWall();
            coolDown();
            hitCapBall();
        }else {
            moveFromWall2();
            coolDown();
            hitCapBall2();
        }
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
//        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
//        gyro.calibrate();
//        gyro.resetZAxisIntegrator(); //address is 0x20
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        button.setPosition(0);
        gate.setPosition(0.4);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void lineUpToWall(int distance) {
        driveTrain.moveForwardNInch(0.3, 3, 10, false);
        driveTrain.moveForwardNInch(0.7, distance-3, 10, false);
        //pause();
        driveTrain.powerAllMotors(0.3);
        boolean detectColor = false;
        timer.reset();
        int initialTicks = frontRight.getCurrentPosition();
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                detectColor = voiColorSensorBottom.isWhite();

                timer.reset();
                if (frontRight.getCurrentPosition() - initialTicks > 35 * driveTrain.TICKS_PER_INCH_FORWARD ){
                    driveTrain.moveBackwardNInch(0.5, 10, 8, false);

                    break;
                }
            }
        }
        //pause();
        // align with wall
        if (detectColor)
            driveTrain.rotateDegrees((int) (angle * 0.7), true);
        else
            driveTrain.rotateDegrees(90, true);

        //pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 40, 10, true);
        //pause();

    }
    public void drivePushButton() {

        // move backwards to get behind beacon
        driveTrain.moveBackwardNInch(0.4, 4, 10, false);

        // move forward until beacon detected
        //pause();
        driveTrain.powerAllMotors(0.2);
        boolean detectColor = false;
        int counter = 0;
        int initialTicks = backRight.getCurrentPosition();
        timer.reset();
        timer2.reset();
        missed = false;
        while (!detectColor && opModeIsActive()) {
            System.out.println("In loop");
            if (timer.time() > 30) {
                detectColor = voiColorSensorTop.isBlue();
                timer.reset();
                if (voiColorSensorTop.isRed() && voiColorSensorTop.isBlue()) detectRed1 = true;
                if (detectRed1 && !voiColorSensorTop.isBlue() && !voiColorSensorTop.isRed()) {
                    counter++;
                    System.out.println("BAD: " + counter);
                }
//                    if (counter > 50) {
//                        return;
//                    }
                if (backRight.getCurrentPosition()-initialTicks > 20*driveTrain.TICKS_PER_INCH_FORWARD || timer2.time()>5000) {
                    //pause();
                    missed = true;
                    return;
                }
            }
        }
        if (detectRed1)
            betweenBeacon -= 4;

        //pause();
        // move forward to align button pusher with beacon button and push
        //pause();
        correctionStrafe();
        //pause();
        pushButton();
    }
    public void drivePushButton2() {
        if (!missed) {
            driveTrain.moveForwardNInch(0.7,betweenBeacon, 10, false);
        } else {
            driveTrain.moveForwardNInch(0.7, betweenBeacon -15, 10, false);
        }
        //pause();
        correctionStrafe();
        boolean detectColor = false;
        driveTrain.powerAllMotors(0.2);
        timer.reset();
        int counter = 0;

        if (REDTEAM) {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                    timer.reset();
                }
            }
        } else {
            while (!detectColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    // determine which side of beacon
                    if (voiColorSensorTop.isRed()){
                        shootRotation2 = sralt;
                    }
                    detectColor = voiColorSensorTop.isBlue();
                    timer.reset();
                }
            }
        }
        //pause();
        correctionStrafe();
        //pause();
        //pause();
        pushButton();
    }
    public void firstBeaconOptimization(){
        if(voiColorSensorTop.isRed()){
            driveTrain.stopAll();
            pushButton();
            return;
        }
        if (voiColorSensorTop.isBlue()){
            driveTrain.moveForwardNInch(0.3,2,2,false);
            driveTrain.stopAll();
            pushButton();
            return;
        }
        driveTrain.powerAllMotors(-0.2);
        while (opModeIsActive()&&!voiColorSensorTop.isRed()&&!voiColorSensorTop.isBlue()){
        }
        firstBeaconOptimization();

    }
    public void pushButton(){
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
        sleep(500);
    }
    public void moveFromWall(){
        setFlywheelPower(shootPower);
        sweeper.setPower(1);
        //System.out.println("Sweeper: " + sweeper.getPower());
        driveTrain.moveLeftNInch(0.6, 8, 10, false);
        //pause();
        driveTrain.rotateDegreesPrecision(shootRotation);
        driveTrain.moveForwardNInch(0.6, 2, 10, false);
        //pause();
        //System.out.println("Sweeper: " + sweeper.getPower());
        sweeper.setPower(1);
        //System.out.println("Sweeper: " + sweeper.getPower());
        conveyor.setPower(0.3);
        sweeper.setPower(1);
        sleep(2500);

    }
    public void checkFirst() {
        driveTrain.moveBackwardNInch(0.8,42, 10, false);
        //pause();
        correctionStrafe();
        //pause();
        driveTrain.powerAllMotors(-0.3);
        boolean detectColor = false;
        boolean wrongColor = false;
        if (REDTEAM) {
            while (!detectColor && !wrongColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                    if (voiColorSensorTop.isBlue()){
                        driveTrain.stopAll();
                    }
                    timer.reset();
                }
            }
        } else {
            while (!detectColor && !wrongColor && opModeIsActive()) {
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isBlue();
                    if (voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue()){
                        driveTrain.stopAll();
                        wrongColor = true;
                    }
                    timer.reset();
                }
            }
        }
        if (wrongColor && !detectColor){
            System.out.println("WRONG COLOR");
            //pause();
            correctionStrafe();
            pause();
            pushButton();
        } else {
            boolean blue = voiColorSensorTop.isBlue();
            boolean red = false;
            boolean white = false;
            System.out.println("BLUE2 " + blue);
            while (opModeIsActive() && blue && !white){
                if (timer.time()>30){
                    blue = voiColorSensorTop.isBlue();
                    timer.reset();
                    white = voiColorSensorBottom.isWhite();

                }
                System.out.println("blue");
            }

            while (opModeIsActive() && !blue && !red && !white){
                if (timer.time()>30) {
                    blue = voiColorSensorTop.isBlue();
                    red = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                    white = voiColorSensorBottom.isWhite();
                    timer.reset();
                }
                System.out.println("blank");
            }
            if (red) {
                goBackAndPress();
                shootRotation = 90;
            }else if (white){
                driveTrain.moveForwardNInch(0.5, 2, 3, false);
            }

        }
        //pause();
    }
    public void coolDown() {
        timer.reset();
        sleep(1000);
        while (opModeIsActive() && timer.time() < 1000){}
        setFlywheelPower(0.4);
        driveTrain.setMotorPower(conveyor, 0.15);
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000){}
        setFlywheelPower(0);
        driveTrain.setMotorPower(conveyor, 0);
        driveTrain.setMotorPower(sweeper, 0);
    }
    public void setFlywheelPower(double power) {
        flywheelLeft.setPower(0.78*power);
        flywheelRight.setPower(0.78*power);
    }
    public void goBackAndPress() {
        driveTrain.powerAllMotors(0.3);
        while (opModeIsActive() && !voiColorSensorTop.isBlue()){}
        //pause();
        driveTrain.moveForwardNInch(0.3,3,10, false);
        //pause();
        correctionStrafe();
        //pause();
        pushButton();
        //pause();
    }
    public void correctionStrafe() {
        driveTrain.moveRightNInch(0.2,5,0.5, false);
    }
    public void hitCapBall(){
        int initialDirection = imu.getAngle();
        driveTrain.moveBackwardNInch(1, 50, 10, true);
        driveTrain.rotateDegrees((int) (capBallRotation * 0.3), false);
        driveTrain.rotateDegrees((int)((initialDirection-imu.getAngle())*0.25), false);
        driveTrain.moveBackwardNInch(1, 18, 10, true);
    }
    public void pickUpBall(){
        sweeper.setPower(1);
        sleep(1500);
        sweeper.setPower(0);
        driveTrain.moveRightNInch(0.5, 10, 5, false);
        driveTrain.rotateDegreesPrecision(pickUpRotation);
    }
    public void options(){
        telemetry.addData("Pick up ball?", pickUp);
        telemetry.update();
        boolean confirmed = false;
        boolean dPadUpPressed = false;
        boolean dPadDownPressed = false;
        while(!confirmed){
            if (gamepad1.a){
                pickUp = true;
            }
            if (gamepad1.b){
                pickUp = false;
            }
            if (gamepad1.dpad_down&& !dPadDownPressed && shootPower > 0){
                dPadDownPressed = true;
                shootPower -= 0.01;
            }
            if (gamepad1.dpad_up && !dPadUpPressed && shootPower < 1){
                dPadUpPressed = true;
                shootPower += 0.01;
            }
            if (!gamepad1.dpad_down){
                dPadDownPressed = false;
            }
            if (!gamepad1.dpad_up){
                dPadUpPressed = false;
            }
            telemetry.addData("Shoot power", shootPower);
            telemetry.addData("Pick up ball?", pickUp);
            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Shoot power", shootPower);
                telemetry.addData("Pick up ball?", pickUp);
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                confirmed = true;
            }

        }
    }
    public void moveFromWall2 (){
        setFlywheelPower(shootPower);
        driveTrain.moveLeftNInch(1, 8, 10, false);
        //pause();
        driveTrain.rotateDegreesPrecision(shootRotation2);
        //pause();
        driveTrain.moveBackwardNInch(1,18,3,false);
        sweeper.setPower(1);
        conveyor.setPower(0.3);
        sweeper.setPower(1);
        sleep(2500);
    }
    public void hitCapBall2(){
        driveTrain.moveBackwardNInch(1, 36, 10, false);
    }
}
