package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class FlywheelTask extends Thread {

    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    private LinearOpMode opMode;
    public volatile boolean running = true;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public FlywheelState state;
    private final int THEORETICAL_MAX_RPM = 1500;
    private final int FULL_SPEED_RPM = 1000;
    private final int TICKS_PER_REV = 112;
    private final int MAX_ENCODER_TICKS_PER_SEC = (int) (1.0*FULL_SPEED_RPM/60*TICKS_PER_REV);
    private final double MAX_ALLOWED_ERROR = 0.05;      //Currently set at 0.05 error, we'll see how this works out.
    private final double KP = 1.0/MAX_ENCODER_TICKS_PER_SEC;     //Proportional error constant to tune, temp set at 1/Maximum encoder ticks

    private int targetEncoderRate = 0;
    private int lastEncoderReadingLeft = 0;
    private int lastEncoderReadingRight = 0;
    private long lastTime = 0;
    private long currentTime = 0;
    private double leftPower = 0;
    private double rightPower = 0;

    public enum FlywheelState {
        STATE_STOPPED, STATE_ACCELERATING, STATE_ADJUSTING, STATE_RUNNING_NEAR_TARGET
    }

    public String getFlywheelState() {
        return state.toString();
    }

    public FlywheelTask(LinearOpMode opMode, DcMotor flywheelLeft, DcMotor flywheelRight) {
        this.flywheelLeft = flywheelLeft;
        this.flywheelRight = flywheelRight;
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.opMode = opMode;
        this.state = FlywheelState.STATE_STOPPED;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {

            if (opMode.gamepad2.a){
                setFlywheelPow(0.7);
            } else if (opMode.gamepad2.x){
                setFlywheelPow(0);
            } else if (opMode.gamepad2.b){
                setFlywheelPow(1.0);
            } else if(opMode.gamepad2.dpad_left) {
                setFlywheelPow(-0.2);
            }

            currentTime = System.nanoTime();
            long deltaTime = currentTime - lastTime;
            int encoderReadingLeft = getEncoderLeft();
            int encoderReadingRight = getEncoderRight();

            if(state == FlywheelState.STATE_ACCELERATING) {
                if(timer.time() > 500) {//Give the flywheel half a second to power up before adjusting speed
                    state = FlywheelState.STATE_ADJUSTING;
                }
            } else if (state == FlywheelState.STATE_ADJUSTING || state == FlywheelState.STATE_RUNNING_NEAR_TARGET) {
                if(lastEncoderReadingLeft == 0) {
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                    timer.reset();
                } else if (deltaTime > 50000000L) {
                    int approxRateLeft = (int) ((encoderReadingLeft - lastEncoderReadingLeft)*1.0/deltaTime*1000000000L);
                    int approxRateRight = (int) ((encoderReadingRight - lastEncoderReadingRight)*1.0/deltaTime*1000000000L);
                    if(Math.abs(approxRateLeft - targetEncoderRate) < targetEncoderRate*MAX_ALLOWED_ERROR && Math.abs(approxRateRight - targetEncoderRate) < targetEncoderRate*MAX_ALLOWED_ERROR) {
                        state = FlywheelState.STATE_RUNNING_NEAR_TARGET;
                    }

                    leftPower += 1.0*(targetEncoderRate - approxRateLeft)*KP;
                    rightPower += 1.0*(targetEncoderRate - approxRateRight)*KP;
                    leftPower = range(leftPower);
                    rightPower = range(rightPower);

                    flywheelLeft.setPower(leftPower);
                    flywheelRight.setPower(rightPower);

                    lastTime = currentTime;
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                }

            }

            opMode.telemetry.addData("Flywheel", flywheelRight.getPower());
            opMode.telemetry.addData("Flywheel Status", getFlywheelState());

        }
        flywheelRight.setPower(0);
        flywheelLeft.setPower(0);
    }

    private double range(double rawPower) {
        if(rawPower > 1) {
            System.out.println("Raw power going over 1, tune your code better");
            return 1;
        } else if (rawPower < -1) {
            System.out.println("Raw power going under 1, something is seriously wrong");
            return -1;
        } else {
            return rawPower;
        }
    }

    public void setFlywheelPow(double power) {
        if(power == 0) {
            state = state.STATE_STOPPED;
        } else {
            state = state.STATE_ACCELERATING;
        }
        flywheelRight.setPower(power*FULL_SPEED_RPM/THEORETICAL_MAX_RPM);
        flywheelLeft.setPower(power*FULL_SPEED_RPM/THEORETICAL_MAX_RPM);
        timer.reset();
        targetEncoderRate = (int) (MAX_ENCODER_TICKS_PER_SEC * power);
        leftPower = rightPower = power*FULL_SPEED_RPM/THEORETICAL_MAX_RPM;
        //We intentionally set the power so that it is highly likely to be lower than the
        //"correct" value so that it continues to adjust upwards.
    }

    private int getEncoderLeft() {
        return flywheelLeft.getCurrentPosition();
    }

    private int getEncoderRight() {
        return flywheelRight.getCurrentPosition();
    }

}
