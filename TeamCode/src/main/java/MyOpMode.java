/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode-MINEE", group="Linear OpMode")

public class MyOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //landon asked me to code the following 3 motors (i don't know what they do)
    private DcMotor rrm = null;

    private DcMotor lrm = null;

    private CRServo servo1 = null;
    private CRServo servo2 = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        rrm = hardwareMap.get(DcMotor.class, "rrm");
        lrm = hardwareMap.get(DcMotor.class, "lrm");
        servo1 = hardwareMap.get(CRServo.class, "s1");
        servo2 = hardwareMap.get(CRServo.class, "s2");

        //init arm motor encoder

        //set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rrm.setDirection(DcMotor.Direction.FORWARD);
        lrm.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int[] armPositions = new int[]{0, -79, -178, -212, -300};
        int step = 0;
        //input states
        boolean lb2Pressed = false;
        boolean rb2Pressed = false;

        //for my pid code
        double prevError = 0.0;
        double prevDerivativeError = 0.0;
        int prevPosition = armPositions[step];
        double integralError = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double threshold=0.01;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double rrmPower = -gamepad2.right_stick_y;
            double lrmPower = -gamepad2.left_stick_y;

            //read arm encoder (higher sensitivity -> more change per joystick position)
            //position = Math.max(Math.min(position + armPower * armSensitivity, 0),-705);
            if(gamepad2.left_bumper && !lb2Pressed){step++;}
            if(gamepad2.right_bumper && !rb2Pressed){step--;}
            step = Math.min(Math.max(step, 0), armPositions.length-1);
            int position = armPositions[step];
            lb2Pressed = gamepad2.left_bumper;
            rb2Pressed = gamepad2.right_bumper;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            lrm.setPower(lrmPower);
            rrm.setPower(rrmPower);
            //my pid code (it's also quite mid)
            /*arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // changing this sensitivity below to see if it reduces sensitivity
            double kP = 0.007;
            double kI = 0.0;
            double kD = 0.002;
            double α = .8;
            double maxIntegralError=100;

            int currentPos = arm.getCurrentPosition();
            if(position!=prevPosition) integralError=0;//reset integral error for every new target position
            double error = position - currentPos;
            integralError += error;
            integralError = Math.min(Math.max(integralError, -maxIntegralError), maxIntegralError);//prevent integral error from getting too high
            double derivativeError = error - prevError;
            double filteredDerivativeError = α*derivativeError+(1-α)*prevDerivativeError;//IIR filter to smooth out spikes in derivative error
            double armOutput = Math.min(Math.max(kP * error + kI * integralError + kD * filteredDerivativeError, -1.0), 1.0);
            prevError = error;
            prevDerivativeError = filteredDerivativeError;
            prevPosition = position;
            arm.setPower(armOutput);*/

            //built-in PID control code for arm (it's quite mid)
//            arm.setTargetPosition(position);
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setPower(1.0);
//
//            int currentPos = arm.getCurrentPosition();



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Ayaan Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("RRM Power", "%4.2f", rrmPower);
            telemetry.addData("LRM Power", "%4.2f", lrmPower);
           // telemetry.addData("Current Position", currentPos);
            telemetry.addData("Target Position", position);
            telemetry.update();
        }
    }}
