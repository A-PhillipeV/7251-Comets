/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AUTORIGHT extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5;    // 5032 yellow jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    final int lowHeight = 6500;
    final int middleHeight = 8400;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);

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

     // Tag ID 1,2,3 from the 36h11 family
    int tag1 = 1;
    int tag2 = 2;
    int tag3 = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        //Robot and Dash
        robot.init();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //Camera
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

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == tag1 || tag.id == tag2 || tag.id == tag3)
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

        /* EDIT CODE BELOW: TRAJECTORY SEQUENCE */
        // ENCODER TRAJECTORIES (preferred)

        if(tagOfInterest == null || tagOfInterest.id == tag3) {
            scoreMediumJunction();
        }
        else if (tagOfInterest.id == tag2) {


        }
        else { //tag1
        }



        /* SETPOWER TRAJECTORIES */
        /*
        if(tagOfInterest == null || tagOfInterest.id == tag1){

            //Move Forward then Right
            double x = -1;
            double y = 1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            double throtte_control = 1;

            robot.motor1.setPower(frontLeftPower*throtte_control);
            robot.motor2.setPower(backLeftPower*throtte_control);
            robot.motor3.setPower(frontRightPower*throtte_control);
            robot.motor4.setPower(backRightPower*throtte_control);

            sleep(6000);

            x = 1;
            y = 0;
            // Move Forward
            denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            frontLeftPower = (y + x) / denominator;
            backLeftPower = (y - x) / denominator;
            frontRightPower = (y - x) / denominator;
            backRightPower = (y + x) / denominator;

            throtte_control = 1;

            robot.motor1.setPower(frontLeftPower*throtte_control);
            robot.motor2.setPower(backLeftPower*throtte_control);
            robot.motor3.setPower(frontRightPower*throtte_control);
            robot.motor4.setPower(backRightPower*throtte_control);
            sleep(2000);



            //Move Forward
        }else if(tagOfInterest.id == tag2){
            //trajectory X & Y MIGHT BE SWAPPED ;) :P B0101101
            double x = 1; //WORKS
            double y = 0; //WORKS

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            double throtte_control = 1;

            robot.motor1.setPower(frontLeftPower*throtte_control);
            robot.motor2.setPower(backLeftPower*throtte_control);
            robot.motor3.setPower(frontRightPower*throtte_control);
            robot.motor4.setPower(backRightPower*throtte_control);

            sleep(2000);

        }else if(tagOfInterest.id == tag3){
            //trajectory
            //Move Forward then Left
            double x = 1; // works
            double y = -1; // works

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            double throtte_control = 1;

            robot.motor1.setPower(frontLeftPower*throtte_control);
            robot.motor2.setPower(backLeftPower*throtte_control);
            robot.motor3.setPower(frontRightPower*throtte_control);
            robot.motor4.setPower(backRightPower*throtte_control);

            sleep(1700);

            //CODE 3

        }

*/
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

    void scoreMediumJunction() {
        strafeDrive(.2,8);


    }


    /*
    Positive = RIGHT
    Negative = LEFT
    */
    public void strafeDrive(double speed,
                            int target) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newMotor1Target = robot.motor1.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
            newMotor2Target = robot.motor2.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
            newMotor3Target = robot.motor3.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
            newMotor4Target = robot.motor4.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
            robot.motor1.setTargetPosition(newMotor1Target);
            robot.motor2.setTargetPosition(newMotor2Target);
            robot.motor3.setTargetPosition(newMotor3Target);
            robot.motor4.setTargetPosition(newMotor4Target);




            // reset the timeout time and start motion.  (these values are for strafing right)
            runtime.reset();
            robot.motor1.setPower(0);
            robot.motor2.setPower(speed*-1);
            robot.motor3.setPower(speed);
            robot.motor4.setPower(0);

            while (opModeIsActive() &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy()) && (robot.motor3.isBusy() && robot.motor4.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newMotor1Target,  newMotor3Target);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        robot.motor1.getCurrentPosition(), robot.motor3.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);

        }
    }

    /*
        Positive: FORWARD
        Negative: BACKWARDS
     */
    public void linearDrive(double speed,
                            double leftInches, double rightInches,
                            double timeoutS) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;
        leftInches *= -1;
        rightInches *= -1;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newMotor1Target = robot.motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newMotor2Target = robot.motor2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newMotor3Target = robot.motor3.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newMotor4Target = robot.motor4.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motor1.setTargetPosition(-newMotor1Target);
            robot.motor2.setTargetPosition(newMotor2Target);
            robot.motor3.setTargetPosition(newMotor3Target);
            robot.motor4.setTargetPosition(-newMotor4Target);


            robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motor1.setPower(Math.abs(speed));
            robot.motor2.setPower(Math.abs(speed));
            robot.motor3.setPower(Math.abs(speed));
            robot.motor4.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy()) && (robot.motor3.isBusy() && robot.motor4.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newMotor1Target,  newMotor3Target);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        robot.motor1.getCurrentPosition(), robot.motor3.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /*
      Moves the arm up and down to a specific height
      NOTE: If going back to orignal position, the new "setTargetPosition" value should be negative
      of the previous encoder ticks

      i.e
      armMove(1, 500, 5); //Moves to 500 ticks
      armMove(1, -500, 5); //Moves back to 0 ticks
     */
    public void armMove(double speed,
                        int target,
                        double timeoutS) {
        int newMotor1Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newMotor1Target = robot.arm.getCurrentPosition() + target;

            robot.arm.setTargetPosition(newMotor1Target);

            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.arm.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.arm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", newMotor1Target);
                telemetry.addData("Currently at",
                        robot.arm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.arm.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void openHand() {
        robot.hand.setPower(1);
        sleep(1750);
        robot.hand.setPower(0);
    }

    public void closeHand() {
        robot.hand.setPower(-1);
        sleep(1750);
        robot.hand.setPower(0);
    }

}
