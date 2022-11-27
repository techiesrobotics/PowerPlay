/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.DriverControlOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechiesHardware;
import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public abstract class TechiesOpMode extends LinearOpMode {

    // Declare OpMode members.
    TechiesHardware robot   = new TechiesHardware();
    private ElapsedTime runtime = new ElapsedTime();
    TechiesHardwareWithoutDriveTrain robotCore;
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0.0;

    abstract public double getTurn() ;
    abstract public double getDrivefb();
    abstract public double getDrivelr();
    abstract public void moveSlide();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double backleftPower;
            double backrightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double turn = getTurn();
            double drivefb  = getDrivefb();  //-gamepad1.left_stick_y;
            double drivelr = getDrivelr(); //gamepad1.left_stick_x;

            leftPower    = Range.clip(drivefb + turn + drivelr, -1.0, 1.0) ;
            rightPower   = Range.clip(drivefb - turn - drivelr, -1.0, 1.0) ;
            backleftPower   = Range.clip(drivefb + turn - drivelr, -1.0, 1.0) ;
            backrightPower   = Range.clip(drivefb - turn + drivelr, -1.0, 1.0) ;


            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.leftBack.setPower(backleftPower);
            robot.rightBack.setPower(backrightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();



            //slides
            //up
            moveSlide();

            if (gamepad1.a) {
                if (robotCore.claw.getPosition() > 0.5) {
                    robotCore.claw.setPosition(0);
                    sleep(200);
                }
                else if (robotCore.claw.getPosition() <= 0.5) {
                    robotCore.claw.setPosition(1);
                    sleep(200);
                }
            }

        }
    }




    /*protected void SlideMovementPID (int targetPosition) {
        telemetry.addData("SlideMovementPID", "start SlideMovementPID");
        robotCore.slides.setTargetPosition(targetPosition,-targetPosition);
        robotCore.slides.rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robotCore.slides.leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (robotCore.slides.rightSlide.isBusy() && repetitions < 800) {
            robotCore.slides.setRiserPower(0.8,0.8);
        }
        else {
            robotCore.slides.setRiserPower(0,0);
            repetitions = 0;
        }
        currentVelocity = robotCore.slides.rightSlide.getVelocity();
        currentPos = robotCore.slides.leftSlide.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos- robotCore.slides.rightSlide.getTargetPosition());
        telemetry.addData("power", robotCore.slides.rightSlide.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }

     */
}