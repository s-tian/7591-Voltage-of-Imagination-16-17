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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoBlue", group = "Tests")

public class AutoBlue extends LinearOpMode {
    static int delay = 200;
    static int shootRotation = 102;
    static int capBallRotation = -180;
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
    public void runOpMode() {
        System.out.println("Hello world");
        initialize();
        waitForStart();
        lineUpToWall();
        drivePushButton();
        drivePushButton2();
        checkFirst();
        moveFromWall();
        coolDown();
        hitCapBall();
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
    public void lineUpToWall() {
        driveTrain.moveForwardNInch(0.7, 50, 10, false);
        //pause();
        driveTrain.powerAllMotors(0.3);
        boolean detectColor = false;
        timer.reset();
        while (!detectColor && opModeIsActive()) {
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            if (timer.time() > 30) {
                detectColor = voiColorSensorBottom.isWhite();
                telemetry.addData("Color: ", red + " " + green + " " + blue);
                updateTelemetry(telemetry);
                timer.reset();
            }
        }
        //pause();
        // align with wall
        driveTrain.rotateDegrees((int) (-angle * 0.7), true);

        //pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 40, 10, true);
        //pause();

    }
    public void drivePushButton() {
        // move backwards to get behind beacon
        driveTrain.moveBackwardNInch(0.4,3, 10, false);
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
                    detectColor = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                    timer.reset();
                    if (voiColorSensorTop.isBlue()) oppositeColor = true;
                    if (oppositeColor && !voiColorSensorTop.isBlue() && !voiColorSensorTop.isRed()) {
                        counter++;
                        System.out.println("BAD: " + counter);
                    }

                    //if (backRight.getCurrentPosition()-initialTicks > 1200) {
                        //pause();
                     //   return;
                    //}
                }
            }
        } else {
            while (!detectColor && opModeIsActive()) {
                System.out.println("In loop");
                if (timer.time() > 30) {
                    detectColor = voiColorSensorTop.isBlue();
                    timer.reset();
                    if (voiColorSensorTop.isRed() && voiColorSensorTop.isBlue()) oppositeColor = true;
                    if (oppositeColor && !voiColorSensorTop.isBlue() && !voiColorSensorTop.isRed()) {
                        counter++;
                        System.out.println("BAD: " + counter);
                    }
//                    if (counter > 50) {
//                        return;
//                    }
//                    if (backRight.getCurrentPosition()-initialTicks > 1200) {
//                        //pause();
//                        return;
//                    }
                }
            }
        }
        //pause();
        // move forward to align button pusher with beacon button and push
        driveTrain.moveForwardNInch(0.2,3, 10, false);
        //pause();
        correctionStrafe();
        //pause();
        pushButton();
    }
    public void drivePushButton2() {
        driveTrain.moveForwardNInch(0.7,32, 10, false);
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
                    detectColor = voiColorSensorTop.isBlue();
                    timer.reset();
                }
            }
        }
        pause();
        correctionStrafe();
        pause();
        driveTrain.moveForwardNInch(0.2,3, 10, false);
        pause();
        pushButton();
    }
    public void pushButton(){
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
        sleep(500);
    }
    public void moveFromWall(){
        setFlywheelPower(1);
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
        driveTrain.moveBackwardNInch(0.8,40, 10, false);
        //pause();
        correctionStrafe();
        //pause();
        driveTrain.powerAllMotors(-0.2);
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
                shootRotation = 95;
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
        int initialDirection = gyro.getIntegratedZValue();
        driveTrain.moveBackwardNInch(1, 50, 10, true);

        driveTrain.rotateDegrees((int) (capBallRotation * 0.3), false);
        driveTrain.rotateDegrees((int)((initialDirection-gyro.getIntegratedZValue())*0.3), false);
        driveTrain.moveBackwardNInch(1, 15, 10, true);
    }


}
