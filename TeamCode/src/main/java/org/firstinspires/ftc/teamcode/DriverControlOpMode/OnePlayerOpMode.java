package org.firstinspires.ftc.teamcode.DriverControlOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDtest;
import org.firstinspires.ftc.teamcode.TechiesSlideHardware;

@TeleOp(name="1 Player: DriverControlOpMode", group="Linear Opmode")
public class OnePlayerOpMode extends TechiesOpMode {
    PIDtest pidSlides = new PIDtest();
    TechiesSlideHardware theActualSlides = new TechiesSlideHardware(hardwareMap);
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
        if (gamepad1.left_bumper) {
            robotCore.slides.rightSlide.setPower(-.75);
            robotCore.slides.leftSlide.setPower(.75);
        } else {
            robotCore.slides.rightSlide.setPower(-0.001);
            robotCore.slides.leftSlide.setPower(0.001);
        }

        //up
        if (gamepad1.right_bumper) {
            robotCore.slides.rightSlide.setPower(1);
            robotCore.slides.leftSlide.setPower(-1);
        } else {
            robotCore.slides.rightSlide.setPower(-0.001);
            robotCore.slides.leftSlide.setPower(0.001);
        }
        if (gamepad1.b)  {
            pidSlides.slideMovementPID(theActualSlides, 500);
        }

    }


    /*public void moveSlideWithButton() {
        // low
        if (gamepad1.x) {
            encoderSlide(1, -19, 19, 3);
        }
        // middle
        else if (gamepad1.y) {
            encoderSlide(1, -29, 29, 3);

        }
        // high
        else if (gamepad1.b) {
            encoderSlide(1, -39, 39, 3);
        }
    }

     */

   /* public double driveSpeed() {
        double Multiplier = 1;
        if (gamepad1.left_bumper) {
            Multiplier = 0.5;

        }
        return  Multiplier;
    }*/
}
