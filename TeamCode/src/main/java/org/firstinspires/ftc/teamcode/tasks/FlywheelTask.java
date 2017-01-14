package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class FlywheelTask extends Thread {

    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    private LinearOpMode opMode;
    public volatile boolean running = true;
    public volatile boolean teleOp = false;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public volatile FlywheelState state;
    private final int THEORETICAL_MAX_RPM = 2000;
    private final int FULL_SPEED_RPM = 1600;
    private final int TICKS_PER_REV = 112;
    private final int MAX_ENCODER_TICKS_PER_SEC = (int) (1.0*FULL_SPEED_RPM/60*TICKS_PER_REV);
    private final double MAX_ALLOWED_ERROR = 0.15;      //When the difference between the actual speed and targeted speed is smaller than this percentage, the state will display as RUNNNING_NEAR_TARGET.
    private final double CLOSE_ERROR = 0.05;
    public double currentErrorLeft, currentErrorRight;
    private final double KP = 1.0/15/MAX_ENCODER_TICKS_PER_SEC;     //Proportional error constant to tune

    private int targetEncoderRate = 0;
    private int lastEncoderReadingLeft = 0;
    private int lastEncoderReadingRight = 0;
    private long lastTime = 0;
    private long currentTime = 0;
    private double leftPower = 0;
    private double rightPower = 0;
    public double voltage = 12;
    private double expectedVoltage = 12;

    public enum FlywheelState {
        STATE_STOPPED, STATE_ACCELERATING, STATE_ADJUSTING, STATE_RUNNING_NEAR_TARGET
    }


    public FlywheelTask(LinearOpMode opMode) {
        flywheelRight = opMode.hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = opMode.hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.opMode = opMode;
        this.state = FlywheelState.STATE_STOPPED;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if (teleOp) {
                if (opMode.gamepad2.a) {
                    setFlywheelPow(0.75);
                } else if (opMode.gamepad2.x) {
                    setFlywheelPow(0);
                } else if (opMode.gamepad2.b) {
                    setFlywheelPow(0.8);
                } else if (opMode.gamepad2.y) {
                    setFlywheelPow(-0.4);
                }
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
                if(lastEncoderReadingLeft == 0 && lastEncoderReadingRight == 0) {
                    //We just entered this state from the adjusting state, update encoder and time values.
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                    lastTime = currentTime;
                } else if (deltaTime > 200000000L) {
                    int approxRateLeft = (int) ((encoderReadingLeft - lastEncoderReadingLeft)*1.0/deltaTime*1000000000L);
                    int approxRateRight = (int) ((encoderReadingRight - lastEncoderReadingRight)*1.0/deltaTime*1000000000L);
                    currentErrorLeft = (double)(approxRateLeft - targetEncoderRate)/targetEncoderRate;
                    currentErrorRight = (double)(approxRateRight - targetEncoderRate)/targetEncoderRate;
                    if(Math.abs(currentErrorLeft) < MAX_ALLOWED_ERROR && Math.abs(currentErrorRight) < MAX_ALLOWED_ERROR) {
                        state = FlywheelState.STATE_RUNNING_NEAR_TARGET;
                    } else {
                        state = FlywheelState.STATE_ADJUSTING;
                    }

                    leftPower += 1.0*(targetEncoderRate - approxRateLeft)*KP;
                    rightPower += 1.0*(targetEncoderRate - approxRateRight)*KP;
                    leftPower = range(leftPower);
                    rightPower = range(rightPower);

                    System.out.println("Target rate: " + targetEncoderRate + " Left rate: " + approxRateLeft + " Right rate: " + approxRateRight);
                    System.out.println("LPower " + leftPower + " RPower " + rightPower);

                    updatePowers();

                    lastTime = currentTime;
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                }

            }

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
        double ratio = Math.min(expectedVoltage/voltage, 1);
        timer.reset();
        targetEncoderRate = (int) (MAX_ENCODER_TICKS_PER_SEC * power);
        leftPower = rightPower = power*FULL_SPEED_RPM/THEORETICAL_MAX_RPM * expectedVoltage / voltage;
        updatePowers();
        //We intentionally set the power so that it is highly likely to be lower than the
        //"correct" value so that it continues to adjust upwards.
        lastEncoderReadingRight = lastEncoderReadingLeft = 0;
    }

    private void updatePowers() {
        flywheelRight.setPower(rightPower);
        flywheelLeft.setPower(leftPower);
    }

    private int getEncoderLeft() {
        return flywheelLeft.getCurrentPosition();
    }

    private int getEncoderRight() {
        return flywheelRight.getCurrentPosition();
    }

    public FlywheelState getFlywheelState() { return state; }

    public String getFlywheelStateString() {
        return state.toString();
    }

}
