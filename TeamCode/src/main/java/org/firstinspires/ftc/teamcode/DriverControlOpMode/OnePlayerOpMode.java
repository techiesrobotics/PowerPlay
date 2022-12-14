package org.firstinspires.ftc.teamcode.DriverControlOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="1 Player: DriverControlOpMode", group="Linear Opmode")
public class OnePlayerOpMode extends TechiesOpMode {
    public double getTurn() {
        double turn = gamepad1.right_stick_x;
        return turn;
    }

    public double getDrivefb() {
        double drivefb = -gamepad1.left_stick_y;
        return drivefb;
    }


    public double getDrivelr() {
        double drivelr = gamepad1.left_stick_x;
        return drivelr;
    }

    public void moveSlideFree() {
        if (gamepad1.left_bumper)  {
            robotCore.slides.rightSlide.setPower(-.75);
            robotCore.slides.leftSlide.setPower(.75);
        }

        else {
            robotCore.slides.rightSlide.setPower(0.001);
            robotCore.slides.leftSlide.setPower(0.001);
        }

        //up
        if (gamepad1.right_bumper)  {
            robotCore.slides.rightSlide.setPower(1);
            robotCore.slides.leftSlide.setPower(-1);
        }

        else {
            robotCore.slides.rightSlide.setPower(0.001);
            robotCore.slides.leftSlide.setPower(0.001);
        }
        //if (gamepad1.b)  {
          //  encoderSlide(1,10,10,5);
        //}

    }




}
