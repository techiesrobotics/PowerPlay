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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class TechiesHardwareWithoutDriveTrain
{
    /* Public OpMode members. hi*/
    //public DcMotor  leftDrive   = null;
    //public DcMotor  rightDrive  = null;
    //public DcMotor  leftBack    = null;
    //public DcMotor  rightBack   = null;
    public TechiesSlideHardware slides = null;
    public Servo   claw = null;

    public DcMotor  leftSlide    = null;
    public DcMotor  rightSlide   = null;


    /* local OpMode members. */
    HardwareMap hwMap     =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */

    public TechiesHardwareWithoutDriveTrain(HardwareMap aHWMap){

        /* Initialize standard Hardware interfaces */

        // Save reference to Hardware map
        hwMap = aHWMap;
            // Define and Initialize Motors
        slides = new TechiesSlideHardware(hwMap);

        //leftDrive  = hwMap.get(DcMotor.class, "frontleft");
        //rightDrive = hwMap.get(DcMotor.class, "frontright");
        //leftBack  = hwMap.get(DcMotor.class, "backleft");
        //rightBack    = hwMap.get(DcMotor.class, "backright");
        claw = hwMap.get(Servo.class, "claw");

        claw.setPosition(.5);



        //leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //leftBack.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.FORWARD);
        //leftSlide.setDirection(DcMotor.Direction.REVERSE);
        //rightSlide.setDirection(DcMotor.Direction.REVERSE);


            // Set all motors to zero power

        //leftDrive.setPower(0.0);
        //rightDrive.setPower(0.0);
        //leftBack.setPower(0.0);
        //rightBack.setPower(0.0);
        //leftSlide.setPower(0.0);
        //rightSlide.setPower(0.0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    public void init(HardwareMap hardwareMap) {

    }
}

