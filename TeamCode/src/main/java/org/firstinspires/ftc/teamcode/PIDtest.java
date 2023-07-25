package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TechiesSlideHardware;

public class PIDtest  {
    Telemetry telemetry = null;
    private ElapsedTime runtime = new ElapsedTime();
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;

    public void slideMovementPID (TechiesSlideHardware slides, int targetPosition) {
        slides.rightSlide.setTargetPosition(targetPosition);
        slides.leftSlide.setTargetPosition(-targetPosition);
        slides.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(slides.rightSlide.isBusy() && repetitions < 800) {
            slides.rightSlide.setPower(0.5);
            slides.leftSlide.setPower(0.5);

        }
        else{
            slides.rightSlide.setPower(0);
            slides.leftSlide.setPower(0);
            repetitions = 0;
        }
        currentVelocity = slides.rightSlide.getVelocity();
        currentPos = slides.leftSlide.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        /*telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos- slides.rightSlide.getTargetPosition());
        telemetry.addData("power", slides.rightSlide.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();*/
        repetitions++;
    }
}
