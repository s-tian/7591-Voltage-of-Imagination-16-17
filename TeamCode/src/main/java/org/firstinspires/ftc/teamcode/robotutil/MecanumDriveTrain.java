package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by Stephen on 9/17/2016.
 */
public class MecanumDriveTrain {
    public static final double frontBackRatio = 1;
    public static double TICKS_PER_INCH_FORWARD= 57.89;
    public static double TICKS_PER_INCH_RIGHT = 61.05;
    final static double POWER_RATIO = 0.78;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    ModernRoboticsI2cGyro gyro;
    LinearOpMode opMode;
    public DcMotor[] motorArray = new DcMotor[4];

    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        reverseMotors();
        motorArray[0] = backLeft;
        motorArray[1] = backRight;
        motorArray[2] = frontLeft;
        motorArray[3] = frontRight;
    }
    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) {
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
    public enum DriveTrainMotor {
        BACK_LEFT, BACK_RIGHT, FRONT_LEFT ,FRONT_RIGHT;
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
    public void powerAll(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void stopAll() {
        powerAll(0);
    }
    public void setMotorPower(DcMotor motor,double power){
        motor.setPower(POWER_RATIO*power);
    }
    public void setMotorPower(MecanumDriveTrain.DriveTrainMotor motor, double power){
        switch(motor){
            case BACK_LEFT:
                backLeft.setPower(power);
                break;
            case BACK_RIGHT:
                backRight.setPower(power);
                break;
            case FRONT_LEFT:
                frontLeft.setPower(power);
                break;
            case FRONT_RIGHT:
                frontRight.setPower(power);
        }
    }
    public void powerAllMotors(double power){
        setMotorPower(backLeft, power);
        setMotorPower(backRight, power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, power);
    }
    public void powerRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(frontRight, power);
    }
    public void powerLeft(double power){
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, power);
    }
    public void startClockWiseRotation(double power){
        powerLeft(power);
        powerRight(-power);
    }
    public void startCounterClockWiseRotation(double power){
        powerLeft(-power);
        powerRight(power);
    }
    public void startRotation(double power){
        powerLeft(power);
        powerRight(-power);
    }
    public void strafeLeft(double power) {
        setMotorPower(backRight,  -power);
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, -power*frontBackRatio);
        setMotorPower(frontRight, power*frontBackRatio);
    }
    public void strafeRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(backLeft, -power);
        setMotorPower(frontLeft, power*frontBackRatio);
        setMotorPower(frontRight, -power*frontBackRatio);
    }
    public void rotateDegrees(int degrees) throws InterruptedException{
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;
        System.out.println(gyro.getIntegratedZValue());
        int targetGyro = gyro.getIntegratedZValue() + degrees;
        double velocity;
        while (Math.abs(gyro.getIntegratedZValue() - targetGyro) > 2 && opMode.opModeIsActive()){
            double gyroValue = gyro.getIntegratedZValue();
            if (gyroValue < targetGyro) {
                velocity = Math.max((targetGyro - gyroValue)*0.55/degrees, 0.2);
            } else {
                velocity = Math.min((targetGyro-gyroValue)*0.55/degrees, -0.2);
            }
            startRotation(velocity);
        }
        stopAll();
        System.out.println(gyro.getIntegratedZValue());
    }
    public void moveForwardNInch(double power, double inches) throws InterruptedException {
        moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD));
        stopAll();
    }
    public void moveBackwardNInch(double power, double inches) throws InterruptedException {
        moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD));
        stopAll();
    }
    public void moveLeftNInch(double power, double inches, int timeout) throws InterruptedException{
        moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_RIGHT), timeout);
}
    public void moveRightNInch(double power, double inches, int timeout) throws InterruptedException{
        moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_RIGHT), timeout);
    }
    public void moveLeftTicksWithEncoders(double power, int ticks, int timeout) throws InterruptedException{
        long initialTime = System.nanoTime();
        int initialBackRight = backRight.getCurrentPosition();
        strafeLeft(power);
        while(backRight.getCurrentPosition() > initialBackRight - ticks && opMode.opModeIsActive() && System.nanoTime() < initialTime + timeout*1000000000L) {
        }
        stopAll();
    }
    public void moveRightTicksWithEncoders(double power, int ticks, int timeout) throws InterruptedException{
        long initialTime = System.nanoTime();
        int initialBackRight = backRight.getCurrentPosition();
        strafeRight(power);
        while(backRight.getCurrentPosition() < initialBackRight + ticks && opMode.opModeIsActive() && System.nanoTime() < initialTime + timeout*1000000000L) {
        }
        stopAll();
    }
    public void moveForwardTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int target = frontLeft.getCurrentPosition() - ticks;
        powerAllMotors(power);
        while (opMode.opModeIsActive()) {
            if (frontLeft.getCurrentPosition() > target) {
                stopAll();
                return;
            }
        }
    }
    public void moveBackwardTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int target = frontLeft.getCurrentPosition() - ticks;
        powerAllMotors(power);
        while (opMode.opModeIsActive()) {
            if (frontLeft.getCurrentPosition() > target) {
                stopAll();
                return;
            }
        }
    }
    public void getTicks(){
        System.out.println(frontLeft.getCurrentPosition());
    }
}
