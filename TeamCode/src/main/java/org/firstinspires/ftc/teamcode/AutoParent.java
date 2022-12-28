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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Disabled
@Autonomous(name = "AutoParent", group = "ConceptBlue")
abstract public class AutoParent extends LinearOpMode {

    double SlidePowerInit = .6;
    static final int TARGET_LEVEL_DEFAULT = 3;
    static final int TARGET_LEVEL_LEFT = 1;
    static final int TARGET_LEVEL_MIDDLE = 2;
    static final int TARGET_LEVEL_RIGHT = 3;
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
    static final int OPENED_CLAW = 1;
    static final int CLOSED_CLAW = 0;
    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag Id for sleeve
    static final int LEFT = 5; // Tag ID 18 from the 36h11 family
    static final int MIDDLE = 6;
    static final int RIGHT = 7;


    public abstract double adjustTurn(double angle);
    protected abstract void park();

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
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            targetZone = determineTargetZone(currentDetections, telemetry);
            telemetry.addData(">", "Press Play to start op mode");
             telemetry.update();
        }
        robot.claw.setPosition(OPENED_CLAW);
        waitForStart();
        doMissions(targetZone);
        telemetry.addData("do missions", "finish mission");
        telemetry.update();
    }

    protected int determineTargetZone( ArrayList<AprilTagDetection> currentDetections, Telemetry telemetry){
        int targetZone = TARGET_LEVEL_DEFAULT;
        AprilTagDetection tagOfInterest = null;
        for(AprilTagDetection tag : currentDetections)
        {
            tagOfInterest = tag;
            if(tagOfInterest.id == LEFT)
            {
                telemetry.addLine("Target Zone: Left");
                targetZone = TARGET_LEVEL_LEFT;// TODO KL: use a constant instead of just a number
                break;
            }else if (tagOfInterest.id == MIDDLE)
            {
                telemetry.addLine("Target Zone: Middle");
                targetZone = TARGET_LEVEL_MIDDLE;// TODO KL: use a constant instead of just a number
                break;
            } else if (tagOfInterest.id == RIGHT){
                telemetry.addLine("Target Zone: Right");
                targetZone = TARGET_LEVEL_RIGHT; // TODO KL: use a constant instead of just a number
                break;
            } else{
                telemetry.addLine("Don't see tag of interest :(");
            }
            telemetry.update();
        }
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
        //for medium goal
        /*Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToJunctionFromStart = odoDriveTrain.trajectoryBuilder(startPose)
                .forward(35)
                .build();
        robot.slides.rightSlide.setPower(.5);
        robot.slides.leftSlide.setPower(-.5);
        odoDriveTrain.followTrajectory(goToJunctionFromStart);
        Pose2d startPose2 = goToJunctionFromStart.end();
        odoDriveTrain.setPoseEstimate(startPose2);
        back(10);
        odoDriveTrain.turn(Math.toRadians(adjustTurn(-43)));
        forward(10.5);*/

        robot.slides.rightSlide.setPower(.65);
        robot.slides.leftSlide.setPower(-.65);
        forward(52);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
        odoDriveTrain.turn(Math.toRadians(-47));
        forward(12);
    }
    protected void dropCone()   {
        robot.claw.setPosition(CLOSED_CLAW);
        robot.slides.rightSlide.setPower(-.5);
        robot.slides.leftSlide.setPower(.5);
        sleep(850);
        robot.slides.rightSlide.setPower(0);
        robot.slides.leftSlide.setPower(0);
        telemetry.addData("dropPreloadFreight", "dropPreloadFreight");
        telemetry.update();
    }
    protected void pickupCone(){
        //for medium goal
        /*
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
        robot.claw.setPosition(OPENED_CLAW);  // TODO KL: is this correct? Open or close? 1 or 0
        sleep(650);
        robot.slides.rightSlide.setPower(.4);
        robot.slides.leftSlide.setPower(-.4);
        sleep(500);
*/
        back(12);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(140));
        forward(27);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
        robot.slides.rightSlide.setPower(-.95);
        robot.slides.leftSlide.setPower(.95);
        sleep(550);
        robot.slides.rightSlide.setPower(0);
        robot.slides.leftSlide.setPower(0);
        robot.claw.setPosition(OPENED_CLAW);
        sleep(650);
        robot.slides.rightSlide.setPower(.65);
        robot.slides.leftSlide.setPower(-.65);
        sleep(500);

    }
    protected void goToJunction()   {
        /* for medium goal
        back(25);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(adjustTurn(141)));
        forward(9);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
*/
        back(27);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        odoDriveTrain.turn(Math.toRadians(-140));
        forward(12);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);
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


}