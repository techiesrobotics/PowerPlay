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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

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
@Autonomous(name = "AutoParent", group = "ConceptBlue")
abstract public class AutoParent extends LinearOpMode {

    double SlidePowerInit = .6;
    int TARGET_LEVEL_DEFAULT = 1;
    int targetZone = TARGET_LEVEL_DEFAULT;

    SampleMecanumDrive odoDriveTrain;
    TechiesHardwareWithoutDriveTrain robot ;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag Id for sleeve
    int Left = 5; // Tag ID 18 from the 36h11 family
    int Middle = 6;
    int Right = 7;
    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() {
        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    if (tagOfInterest.id == Left){
                        telemetry.addLine("Target Zone: Left");
                        targetZone = 1;
                    }
                    else if (tagOfInterest.id == Middle){
                        telemetry.addLine("Target Zone: Middle");
                        targetZone = 2;
                    }
                    else if (tagOfInterest.id == Right){
                        telemetry.addLine("Target Zone: Right");
                        targetZone = 3;
                    }
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        // Wait for the game to begin

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
        return targetZone;
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
                .forward(35)
                .build();

        robot.slides.rightSlide.setPower(.5);
        robot.slides.leftSlide.setPower(-.5);
        odoDriveTrain.followTrajectory(goToJunctionFromStart);
        //Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        Pose2d startPose2 = goToJunctionFromStart.end();
        odoDriveTrain.setPoseEstimate(startPose2);
        back(10);

       // odoDriveTrain.turn(Math.toRadians(-43));
        odoDriveTrain.turn(Math.toRadians(adjustTurn(-43)));
        forward(10.5);
    }
    /*
        protected void goToCarousel() {telemetry.addData("goto carousel from parent", "parent");};
        protected void spinCarousel() {telemetry.addData("spinCarousel from parent", "parent");};
        protected void park() {{telemetry.addData("park from parent", "parent");};};

       */
    protected void dropCone()   {
        robot.claw.setPosition(0);
        robot.slides.rightSlide.setPower(-.5);
        robot.slides.leftSlide.setPower(.5);
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
        odoDriveTrain.turn(Math.toRadians(adjustTurn(45)));
        forward(24);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
        odoDriveTrain.turn(Math.toRadians(adjustTurn(88)));
        forward(25);
        Pose2d startPose3 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose3);
        robot.slides.rightSlide.setPower(-.6);
        robot.slides.leftSlide.setPower(.6);
        sleep(550);
        robot.slides.rightSlide.setPower(0);
        robot.slides.leftSlide.setPower(0);
        robot.claw.setPosition(1);
        sleep(650);
        robot.slides.rightSlide.setPower(.4);
        robot.slides.leftSlide.setPower(-.4);
        sleep(500);
        /*Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(137));
        forward(25);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
        robot.slides.rightSlide.setPower(-.6);
        robot.slides.leftSlide.setPower(.6);
        sleep(550);
        robot.slides.rightSlide.setPower(0);
        robot.slides.leftSlide.setPower(0);
        robot.claw.setPosition(1);
        sleep(650);
        robot.slides.rightSlide.setPower(.4);
        robot.slides.leftSlide.setPower(-.4);
        sleep(500);
        */

    }
    protected void goToJunction()   {
        back(25);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(adjustTurn(141)));
        forward(9);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
    }

    protected void park() {

        if (targetZone == 1) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-137)));
            forward(25);

        }
        else if (targetZone == 2) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-137)));
        }
        else if (targetZone == 3) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-137)));
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
    protected void forward (double inches){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory forward = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(inches)
                .build();
        odoDriveTrain.followTrajectory(forward);
        // Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        Pose2d startPose2 = forward.end();
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

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public abstract double adjustTurn(double angle);
}