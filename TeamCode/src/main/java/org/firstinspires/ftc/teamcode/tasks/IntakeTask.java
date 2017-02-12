package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;

/**
 * Created by Howard on 10/15/16.
 */
public class IntakeTask extends TaskThread {

    public volatile double power = 0;
    public volatile int sweepTime = 0;
    private VOISweeper sweeper;
    public volatile boolean oscillate = false;
    CRServo sweeper1, sweeper2, sweeper3;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public IntakeTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void initialize() {
        sweeper1 = opMode.hardwareMap.crservo.get("sweeper1");
        sweeper2 = opMode.hardwareMap.crservo.get("sweeper2");
        sweeper3 = opMode.hardwareMap.crservo.get("sweeper3");
        this.sweeper = new VOISweeper(sweeper1, sweeper2, sweeper3);
        sweeper.setPower(0);
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {

            // TeleOp commands
            if (teleOp) {
                if (opMode.gamepad2.dpad_up) {
                    sweeper.setPower(1);
                } else if (opMode.gamepad2.dpad_down) {
                    sweeper.setPower(-1);
                } else if (opMode.gamepad1.right_trigger > 0) {
                    sweeper.setPower(1);
                } else if (opMode.gamepad1.left_trigger > 0) {
                    sweeper.setPower(-1);
                } else {
                    sweeper.setPower(0);
                }
            }
            // Autonomous commands
            if (power != 0) {
                sweeper.setPower(power);
                power = 0;
                int temp = sweepTime;
                sweepTime = 0;
                sleep(temp);
                sweeper.setPower(0);
            }
            if (oscillate) {
                power = -1;
                sweeper.setPower(-1);
                timer.reset();
                while (oscillate) {
                    if (timer.time() > 50) {
                        power = -power;
                        sweeper.setPower(power);
                        timer.reset();
                    }
                }
            }


        }
        sweeper.setPower(0);
    }

    public void setPower(double power) {
        sweeper.setPower(power);
    }

}
