package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by Stephen on 9/17/2016.
 */
public class MecanumDriveTrain {
    //export PATH="$PATH:/Users/Howard/Library/Android/sdk/platform-tools"
    public static final double TICKS_PER_INCH_FORWARD= 57.89;
    public static final double TICKS_PER_INCH_STRAFE = 61.05;
    public static final double TICKS_PER_MS_FORWARD = 2.489; // at power = 1
    public static final double TICKS_PER_MS_STRAFE = 1.961; // at power = 1
    public static final double factorFL = 1;
    public static final double factorFR = 1;//0.9072
    public static final double factorBL = 0.9069;//0.9069
    public static final double factorBR = 0.9323;
    //public ElapsedTime timer, timer2, timer3;
    public ElapsedTime timer;
    static final double POWER_RATIO = 0.78;
    public DcMotor backLeft, backRight, frontLeft, frontRight;
    ModernRoboticsI2cGyro gyro;
    LinearOpMode opMode;
    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, LinearOpMode opMode) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.opMode = opMode;
        reverseMotors();
    }
    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gyro = gyro;
        this.opMode = opMode;
        reverseMotors();
    }
    private void reverseMotors() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setEncoderMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
    }
    public void setMotorPower(DcMotor motor,double power){
        double factor = 1;
        if (motor == frontLeft){
            factor = factorFL;
        }else if (motor == frontRight){
            factor = factorFR;
        }else if (motor == backLeft){
            factor = factorBL;
        }else if (motor == backRight){
            factor = factorBR;
        }
        motor.setPower(POWER_RATIO*power*factor);
    }
    public void powerAllMotors(double power){
        setMotorPower(backLeft, power);
        setMotorPower(backRight, power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, power);
    }
    public void stopAll() {
        powerAllMotors(0);
    }
    public void powerRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(frontRight, power);
    }
    public void powerLeft(double power){
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, power);
    }
    public void startRotation(double power){
        // power > 0 : clockwise
        // power < 0 : counterclockwise
        powerLeft(power);
        powerRight(-power);
    }
    public void strafeLeft(double power) {
        setMotorPower(backRight,  -power);
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, -power);
        setMotorPower(frontRight, power);
    }
    public void strafeRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(backLeft, -power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, -power);
    }
    public void rotateDegreesPrecision(int degrees){
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;
        double velocity, targetGyro = gyro.getIntegratedZValue() + degrees;
        rotateDegrees(degrees, true);
        while (Math.abs(gyro.getIntegratedZValue() - targetGyro) > 2 && opMode.opModeIsActive()){
            double gyroValue = gyro.getIntegratedZValue();
            if (gyroValue < targetGyro)
                velocity = Math.max((targetGyro - gyroValue)*0.35/degrees, 0.1);
            else
                velocity = Math.min((targetGyro-gyroValue)*0.35/degrees, -0.1);
            startRotation(velocity);
        }
        stopAll();
        System.out.println(gyro.getIntegratedZValue());
    }
    public void rotateDegrees(int degrees, boolean slowdown){
        double gyroValue = gyro.getIntegratedZValue();
        int targetGyro = gyro.getIntegratedZValue() + degrees;
        double velocity;
        timer.reset();
        if (degrees < 0){
            startRotation(-1);
            while (targetGyro < gyroValue && opMode.opModeIsActive()) {
                gyroValue = gyro.getIntegratedZValue();
                if (slowdown) {
                    velocity = Math.min((targetGyro - gyroValue) * 0.35 / degrees, -0.2);
                    startRotation(velocity);
                }
            }
        }
        else{
            startRotation(1);
            while (gyroValue < targetGyro && opMode.opModeIsActive()){
                gyroValue = gyro.getIntegratedZValue();
                if (slowdown){
                    velocity = Math.max((targetGyro - gyroValue)*0.35/degrees, 0.2);
                    startRotation(velocity);
                }
            }
        }
    }
    public void moveForwardNInch(double power, double inches, double timeout, boolean detectStall)  {
        moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall);
    }
    public void moveBackwardNInch(double power, double inches, double timeout, boolean detectStall) {
        moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall);
    }
    public void moveLeftNInch(double power, double inches, double timeout, boolean detectStall) {
        moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, detectStall);
}
    public void moveRightNInch(double power, double inches, double timeout, boolean detectStall) {
        moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, detectStall);
    }
    public void moveLeftTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() - ticks;
        strafeLeft(power);
        timer.reset();
        boolean startDetectingStall = false;
        long currentTime = System.currentTimeMillis();
        while(backRight.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 500) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                }
                if (startDetectingStall && System.currentTimeMillis() - currentTime > 100) {
                    if (stalling(false)) {
                        return;
                    }
                }
            }
        }
        timer.reset();
        stopAll();
    }
    public void moveRightTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        strafeRight(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 300) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(false)) {
                        stopAll();
                        return;
                    }
                }
            }
        }
        stopAll();
    }
    public void moveForwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        powerAllMotors(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 300) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(true)) {
                        stopAll();
                        return;
                    }
                }
            }
        }
        stopAll();
    }
    public void moveBackwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() - ticks;
        powerAllMotors(-power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 300) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(true)) {
                        stopAll();
                        return;
                    }
                }
            }
        }
        stopAll();
    }
    public void getTicks(){
        System.out.println(backRight.getCurrentPosition());
    }
    public void powerForTime(double power, double time) {
        double timeMS = time*1000;
        long currentTime = System.currentTimeMillis();
        powerAllMotors(power);
        while (opMode.opModeIsActive() && (System.currentTimeMillis()-currentTime) < timeMS){
        }
        stopAll();
    }
    public boolean stalling(boolean forward){
        int initialBackRight = backRight.getCurrentPosition();
        int initialFrontRight = frontRight.getCurrentPosition();
        int initialBackLeft = backLeft.getCurrentPosition();
        int initialFrontLeft = frontLeft.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - currentTime) < 100 && opMode.opModeIsActive()){}
        int ticksBackRight = Math.abs(backRight.getCurrentPosition()-initialBackRight);
        int ticksFrontRight = Math.abs(frontRight.getCurrentPosition() - initialFrontRight);
        int ticksBackLeft = Math.abs(backLeft.getCurrentPosition() - initialBackLeft);
        int ticksFrontLeft = Math.abs(frontLeft.getCurrentPosition() - initialFrontLeft);
        int expected = (int) (TICKS_PER_MS_FORWARD*100/2);

        if(!forward)
            expected = (int) (TICKS_PER_MS_STRAFE*100*0.7);

        int stallingMotors = 0;
        if (ticksBackLeft < expected) stallingMotors ++;
        if (ticksBackRight < expected) stallingMotors ++;
        if (ticksFrontRight < expected) stallingMotors ++;
        if (ticksFrontLeft < expected) stallingMotors ++;

        return stallingMotors >= 2;
    }
}
