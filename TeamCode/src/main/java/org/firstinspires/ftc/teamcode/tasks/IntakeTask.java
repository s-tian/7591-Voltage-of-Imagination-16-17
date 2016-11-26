package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class IntakeTask extends Thread {

    private DcMotor sweeper;
    private DcMotor conveyor;
    private ThreadedTeleOp opMode;
    public volatile boolean running = true;


    public IntakeTask(ThreadedTeleOp opMode, DcMotor sweeper, DcMotor conveyor) {
        this.sweeper = sweeper;
        this.conveyor = conveyor;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if(opMode.gamepad2.dpad_right){
                conveyor.setPower(0);
            }
            else if(opMode.gamepad2.dpad_up){
                conveyor.setPower(0.3);
            }
            else if(opMode.gamepad2.dpad_down){
                conveyor.setPower(-0.3);
            }

            if(opMode.gamepad1.y) {
                sweeper.setPower(1);
            } else if(opMode.gamepad1.a) {
                sweeper.setPower(-1);
            } else if(opMode.gamepad1.b){
                sweeper.setPower(0);
            }
        }
        conveyor.setPower(0);
        sweeper.setPower(0);
    }
}
