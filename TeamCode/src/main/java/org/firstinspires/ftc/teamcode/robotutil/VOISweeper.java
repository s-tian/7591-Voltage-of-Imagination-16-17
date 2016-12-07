package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Howard on 12/5/16.
 */

public class VOISweeper {
    CRServo sweeper1, sweeper2;
    public VOISweeper(CRServo sweeper1, CRServo sweeper2) {
        this.sweeper1 = sweeper1;
        this.sweeper2 = sweeper2;
        setPower(0);
    }
    public void setPower(double power) {
        sweeper1.setPower(power);
        sweeper2.setPower(power);
    }

}
