package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;

import static org.firstinspires.ftc.teamcode.R.layout.servo;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends Thread {

    private DcMotor capBottom, capTop;
    private Servo forkLeft, forkRight;
    private LinearOpMode opMode;
    double startLeft = 0.55, startRight = 0.12, downLeft = 0.0, downRight = 0.62; // fork lift positions
    boolean aPushed = false;
    public volatile boolean running = true;
    boolean forkliftOut = false;

    double targetPower = 0;
    int bottomPosition = 0;
    int topPosition = 0;

    boolean pressing = false;
    public static final double flStart = 0.55, frStart = 0.13; // fork left initialize positions
    // decrease flStart and increase frStart to make forklift more out
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public CapBallTask(LinearOpMode opMode) {
        forkLeft = opMode.hardwareMap.servo.get("forkLeft");
        forkRight = opMode.hardwareMap.servo.get("forkRight");
        capBottom = opMode.hardwareMap.dcMotor.get("capBottom");
        capTop = opMode.hardwareMap.dcMotor.get("capTop");
        this.opMode = opMode;
        capBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        capTop.setDirection((DcMotorSimple.Direction.REVERSE));
        bottomPosition = capBottom.getCurrentPosition();
        topPosition = capTop.getCurrentPosition();
        setForkPosition();
    }

    @Override
    public void run() {
        timer2.reset();
        while(opMode.opModeIsActive() && running) {
            if (timer2.time() > 200) {
                if (opMode.gamepad1.right_bumper || opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger > 0.15) {
                    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setLiftPower(1);
                    pressing = true;
                } else if (opMode.gamepad1.left_bumper || opMode.gamepad2.left_trigger - opMode.gamepad2.right_trigger > 0.15) {
                    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setLiftPower(-0.1);
                    pressing = true;
                } else if (pressing) {
                    holdPosition();
                }
            }
            if (opMode.gamepad1.dpad_down && !forkliftOut) {
                forkliftOut = true;
                timer.reset();
                setLiftPower(1);
                while(opMode.opModeIsActive() && timer.time() < 600);
                timer.reset();
                setLiftPower(0);
                while(opMode.opModeIsActive() && timer.time() < 500);
                timer.reset();
                setLiftPower(-1);
                while(opMode.opModeIsActive() && timer.time() < 600);
                setLiftPower(0);
                forkliftOut = false;
                pressing = true;
            }
            timer2.reset();

            if(opMode.gamepad2.right_bumper) {
                forkLeft.setPosition(downLeft);
                forkRight.setPosition(downRight);
                forkliftOut = false;
            } else {
                forkLeft.setPosition(startLeft);
                forkRight.setPosition(startRight);
            }

        }
        setLiftPower(0);
    }

    public void setLiftPower(double power){
        capBottom.setPower(power);
        capTop.setPower(power);
    }

    public double getLiftPower() {
        return capBottom.getPower();
    }

    public void setForkPosition() {
        forkLeft.setPosition(startLeft);
        forkRight.setPosition(startRight);
    }
    public void setMode(DcMotor.RunMode mode) {
        capBottom.setMode(mode);
        capTop.setMode(mode);
    }
    public void holdPosition() {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pressing = false;
        topPosition = capTop.getCurrentPosition();
        bottomPosition = capBottom.getCurrentPosition();
        capTop.setTargetPosition(topPosition);
        capBottom.setTargetPosition(bottomPosition);
        capTop.setPower(1);
        capBottom.setPower(1);
    }



//    private void dropForklift() {
//        //Probably want to use encoders for this
//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        timer.reset();
//        while(timer.time() < 1400 && opMode.opModeIsActive()) {
//            if(opMode.gamepad1.right_bumper||opMode.gamepad2.right_trigger-opMode.gamepad2.left_trigger>0.15 || opMode.gamepad1.left_bumper||opMode.gamepad2.left_trigger-opMode.gamepad2.right_trigger>0.15) {
//                return;
//            }
//            if(timer.time() < 500) {
//                setLiftPower(1);
//            } else {
//                setLiftPower(-1);
//            }
//        }
//        setLiftPower(0);
//        forkliftOut = true;
//    }

}