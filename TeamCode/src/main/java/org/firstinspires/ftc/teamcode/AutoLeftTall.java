/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@Disabled
@Autonomous(name = "AutoLeftTallGoal", group = "ConceptBlue")
public class AutoLeftTall extends LinearOpMode {

    protected static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    double SlidePowerInit = .6;
    int TARGET_LEVEL_DEFAULT = 3;
    int targetZone = TARGET_LEVEL_DEFAULT;
    SampleMecanumDrive odoDriveTrain;
    TechiesHardwareWithoutDriveTrain robot ;
    /*double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;*/

    //SlideMovementPIDController pidController;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    protected static final String VUFORIA_KEY =
            "AWNT9KL/////AAABmVVE0/4aqEuZrZe7Bl0MhilGF88MQ6YYAPiflg2G1Om6F0TrH1ULZwVWI3X/RX5B+FrBmEImMBNB/p5nSnv+67+l1vVeOANwjHuFBU8Jb3kmDZH5cs8a/FNwZqAkwbKk+iCfOeFXnRMQ8kT4g4ndkcYZfbq2LkLp5paPvLMP2uk2ufO+qmqLPmRjWANSj7ltPhpdx3OR62AcQs9WV4M9celQ5SA0coKTUQuPVOclVq5sYZa98GWZ1oBex3mnvsEoP1TWubHw7dxDIZLc3aXkdyEAIXDnHFO1gHBY2/aGRlCZtrW3ep+Hgqaw8iYkLw1r+qMBQvEgmxv7TgmjbsTlo6mCLAnI/cHYjCrDAZH5UIPV";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);
        // set up camera
        initVuforia();
        initTfod();
        //activateCamera();

        // Wait for the game to begin
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
        robot.claw.setPosition(1);
        targetZone = determineLevel();
        telemetry.addData("Target Zone", targetZone);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        waitForStart();
        telemetry.addData("Target Zone", targetZone);
        telemetry.update();

        doMissions(targetZone);
        telemetry.addData("do missions", "finish mission");
        telemetry.update();
    }

    protected int determineLevel() {
        while (!opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                        if (recognition.getLabel().equals("1 Bolt")){
                            targetZone = 1;
                            telemetry.addData(String.format(" target zone: "), targetZone);
                        }
                        else if (recognition.getLabel().equals("2 Bulb")){
                            targetZone = 2;
                            telemetry.addData(String.format(" target zone: "), targetZone);
                        }
                        else if (recognition.getLabel().equals("3 Panel")){
                            targetZone = 3;
                            telemetry.addData(String.format(" target zone: "), targetZone);
                        }
                    }
                    telemetry.update();
                }
            }
        }
        return targetZone;
    }

    protected void activateCamera() {
        if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1, 17.0 / 5.0);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName =hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }




    protected void doMissions(int targetZone) {
        goToJunctionFromStart();
        dropCone();
        pickupCone();
        goToJunction();
        dropCone();
        park();
    }

    protected void goToJunctionFromStart(){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToJunctionFromStart = odoDriveTrain.trajectoryBuilder(startPose)
                .forward(51)
                .build();
        robot.slides.rightSlide.setPower(.55);
        robot.slides.leftSlide.setPower(-.55);
        odoDriveTrain.followTrajectory(goToJunctionFromStart);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
        odoDriveTrain.turn(Math.toRadians(-47));
        forward(9);
    }
/*
    protected void goToCarousel() {telemetry.addData("goto carousel from parent", "parent");};
    protected void spinCarousel() {telemetry.addData("spinCarousel from parent", "parent");};
    protected void park() {{telemetry.addData("park from parent", "parent");};};

   */
    protected void dropCone()   {
        robot.claw.setPosition(0);
        robot.slides.rightSlide.setPower(-.6);
        robot.slides.leftSlide.setPower(.6);
        sleep(850);
        robot.slides.rightSlide.setPower(0);
        robot.slides.leftSlide.setPower(0);
        telemetry.addData("dropPreloadFreight", "dropPreloadFreight");
        telemetry.update();
    }
    protected void pickupCone(){
        back(9);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(137));
        forward(25);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
        robot.slides.rightSlide.setPower(-.8);
        robot.slides.leftSlide.setPower(.8);
        sleep(550);
        robot.slides.rightSlide.setPower(0);
        robot.slides.leftSlide.setPower(0);
        robot.claw.setPosition(1);
        sleep(650);
        robot.slides.rightSlide.setPower(.6);
        robot.slides.leftSlide.setPower(-.6);
        sleep(500);


    }
    protected void goToJunction()   {
        back(25);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(-137));
        forward(9);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
    }

    protected void park() {

        if (targetZone == 1) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(137));
            forward(25);
            Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
            odoDriveTrain.setPoseEstimate(startPose2);
        }
        else if (targetZone == 2) {
            back(9);

            odoDriveTrain.turn(Math.toRadians(47));
            back(7);
        }
        else if (targetZone == 3) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(137));
            back(24);

        }

    }
    protected void back(int inches){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory back = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .back(inches)
                .build();
        odoDriveTrain.followTrajectory(back);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);

    }
    protected void forward (int inches){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory forward = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(inches)
                .build();
        odoDriveTrain.followTrajectory(forward);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
    }
    protected void lineToSpline(int x, int y, int degrees){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory linetospline = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(degrees)))
                .build();
        odoDriveTrain.followTrajectory(linetospline);
    }


}