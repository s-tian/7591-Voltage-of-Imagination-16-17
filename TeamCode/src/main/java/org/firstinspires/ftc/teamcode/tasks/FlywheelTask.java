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


    public FlywheelTask(LinearOpMode opMode, DcMotor flywheelLeft, DcMotor flywheelRight) {
        this.flywheelLeft = flywheelLeft;
        this.flywheelRight = flywheelRight;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opMode.opModeIsActive() && running) {

            if (opMode.gamepad2.a){
                setPow(flywheelRight, 0.7);
                flywheelLeft.setPower(flywheelRight.getPower());
            }
            else if (opMode.gamepad2.x){
                setPow(flywheelRight, 0);
                flywheelLeft.setPower(flywheelRight.getPower());
            }
            else if (opMode.gamepad2.b){
                setPow(flywheelRight, 1.0);
                flywheelLeft.setPower(flywheelRight.getPower());
            } else if(opMode.gamepad2.dpad_left) {
                setPow(flywheelRight, -0.2);
                flywheelLeft.setPower(flywheelRight.getPower());
            }

            opMode.telemetry.addData("Flywheel", flywheelRight.getPower());

        }
        flywheelRight.setPower(0);
        flywheelLeft.setPower(0);
    }
    private void setPow(DcMotor motor, double power){
        motor.setPower(power*0.78);
    }
}
