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
    //public ElapsedTime timer, timer2, timer3;\
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
        motor.setPower(POWER_RATIO*power);
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
    public void rotateDegreesPrecision(int degrees) throws InterruptedException{
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;
        double velocity, targetGyro = gyro.getIntegratedZValue() + degrees;
        rotateDegrees(degrees);
        while (Math.abs(gyro.getIntegratedZValue() - targetGyro) > 2 && opMode.opModeIsActive()){
            double gyroValue = gyro.getIntegratedZValue();
            if (gyroValue < targetGyro)
                velocity = Math.max((targetGyro - gyroValue)*0.35/degrees, 0.2);
            else
                velocity = Math.min((targetGyro-gyroValue)*0.35/degrees, -0.2);
            startRotation(velocity);
        }
        stopAll();
        System.out.println(gyro.getIntegratedZValue());
    }
    public void rotateDegrees(int degrees) throws InterruptedException{
        double gyroValue = gyro.getIntegratedZValue();
        int targetGyro = gyro.getIntegratedZValue() + degrees;
        double velocity;
        timer.reset();
        if (degrees < 0){
            while (targetGyro < gyroValue && opMode.opModeIsActive()) {
                gyroValue = gyro.getIntegratedZValue();
                if (timer.time() > 50) {
                    System.out.println(gyroValue);
                    timer.reset();
                }
                velocity = Math.min((targetGyro - gyroValue) * 0.35 / degrees, -0.2);
                startRotation(velocity);
            }
        }
        else{
            while (gyroValue < targetGyro && opMode.opModeIsActive()){
                gyroValue = gyro.getIntegratedZValue();
                if (timer.time() > 50) {
                    System.out.println(gyroValue);
                    timer.reset();
                }
                velocity = Math.max((targetGyro - gyroValue)*0.35/degrees, 0.2);
                startRotation(velocity);
            }
        }
    }
    public void moveForwardNInch(double power, double inches) throws InterruptedException {
        moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD));
    }
    public void moveForwardNInchDiagonal(double power, double inches, double ratio){
        int target = backRight.getCurrentPosition() + (int)(inches*TICKS_PER_INCH_FORWARD);
        setMotorPower(backLeft, ratio*power);
        setMotorPower(frontRight, ratio*power);
        setMotorPower(backRight, ratio);
        setMotorPower(frontLeft, ratio);
        powerRight(power*ratio);
        while (opMode.opModeIsActive() && backRight.getCurrentPosition() > target) {}
        stopAll();
    }
    public void moveForwardDiagonal(double power, double ratio){
        powerLeft(power);
        powerRight(power*ratio);
    }
    public void moveBackwardNInch(double power, double inches) throws InterruptedException {
        moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD));
    }
    public void moveLeftNInch(double power, double inches, int timeout) throws InterruptedException{
        moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, power == 1);
}
    public void moveRightNInch(double power, double inches, int timeout) throws InterruptedException{
        moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, power == 1);
    }
    public void moveLeftTicksWithEncoders(double power, int ticks, int timeout, boolean detectStall) throws InterruptedException{
        int timeOutMS = timeout*1000;
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
                    if (stalling()) {
                        return;
                    }
                }
            }
        }
        timer.reset();
        stopAll();
    }
    public void moveRightTicksWithEncoders(double power, int ticks, int timeout, boolean detectStall) throws InterruptedException{
        int timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        strafeRight(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 500) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 105) {
                    currentTime = System.currentTimeMillis();
                    if (stalling()) {
                        return;
                    }
                }
            }
        }
        timer.reset();
        stopAll();
    }
    public void moveForwardTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int target = backRight.getCurrentPosition() + ticks;
        powerAllMotors(power);
        while (opMode.opModeIsActive() && backRight.getCurrentPosition() < target) {}
        stopAll();
    }
    public void moveBackwardTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int target = backRight.getCurrentPosition() - ticks;
        powerAllMotors(-power);
        while (opMode.opModeIsActive() && backRight.getCurrentPosition() > target) {}
        stopAll();
    }
    public void getTicks(){
        System.out.println(backRight.getCurrentPosition());
    }
    public void powerForTime(double power, double time) throws InterruptedException{
//        double timeMS = time*1000;
//        long currentTime = System.currentTimeMillis();
//        powerAllMotors(power);
//        while (opMode.opModeIsActive() && (System.currentTimeMillis()-currentTime) < timeMS){
//        }
        powerAllMotors(power);
        opMode.sleep(1000);
        stopAll();
    }
    public boolean stalling(){
        int initialBackRight = backRight.getCurrentPosition();
        int initialFrontRight = frontLeft.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - currentTime) < 100 && opMode.opModeIsActive()){}
        int ticksBackRight = Math.abs(backRight.getCurrentPosition()-initialBackRight);
        int ticksFrontRight = Math.abs(backRight.getCurrentPosition() - initialFrontRight);
        int expected = (int) (TICKS_PER_INCH_STRAFE*100/2);
        return ticksBackRight < expected && ticksFrontRight < expected ;
    }
}
