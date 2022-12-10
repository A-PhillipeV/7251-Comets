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

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

@Autonomous(name="FourMechEncoder", group="Testing")
public class FourMechEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 2786.2 ;    // 5032 yellow jacket
    static final double     DRIVE_GEAR_REDUCTION    = 99.5 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Starting at",  "%7d :%7d %7d :%7d",
                          robot.motor1.getCurrentPosition(),
                          robot.motor2.getCurrentPosition(),
                          robot.motor3.getCurrentPosition(),
                          robot.motor4.getCurrentPosition());
        telemetry.update();

        waitForStart();

        linearDrive(DRIVE_SPEED, 48, 48, 5.0);
        strafeDrive(DRIVE_SPEED,  48,  48, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    /*
        Positive = RIGHT
        Negative = LEFT
     */
    public void strafeDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newMotor1Target = robot.motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newMotor2Target = robot.motor2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newMotor3Target = robot.motor3.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newMotor4Target = robot.motor4.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motor1.setTargetPosition(newMotor1Target);
            robot.motor2.setTargetPosition(newMotor2Target);
            robot.motor3.setTargetPosition(newMotor3Target);
            robot.motor4.setTargetPosition(newMotor4Target);


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
}
