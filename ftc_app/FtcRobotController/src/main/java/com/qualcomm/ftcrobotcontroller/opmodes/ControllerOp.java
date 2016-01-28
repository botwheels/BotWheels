/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.MatrixDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * ControllerOp Mode for Driver Controlled Period
 */
public class ControllerOp extends OpMode {

	// TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.0;
	final static double ARM_MAX_RANGE  = 1.0;
	final static double CLAW_MIN_RANGE  = 0.60;
	final static double CLAW_MAX_RANGE  = 1.00;
    final static double JOINT_MIN_RANGE = 0.30;
    final static double JOINT_MAX_RANGE = 1.00;

	// position of the arm servo.
	double armPosition;
	// amount to change the arm servo position.
	double armDelta = 0.01;

	// position of the claw motor
	double clawPosition;
    // amount to change the claw servo position by
    double clawDelta = 0.01;

    // position of the joint servo
    double jointPosition;
    // amount to change the joint servo position by
    double jointDelta = 0.01;

    //Position arm, joint, claw starts and ends in
    double armStartingPosition = 0.3;
    double jointStartingPosition = 0.5;
    double clawStartingPosition = 1.0;

    float left;
    float right;

    int clock = 0;

	DcMotor motorRight1;
    DcMotor motorRight2;
	DcMotor motorLeft1;
    DcMotor motorLeft2;
    Servo arm;
    Servo joint;
    Servo claw;
    Servo claw2;
    MatrixDcMotorController mc;
    ServoController sc;

	//Constructor
	public ControllerOp() {

	}

	//This code runs first when Op Mode is enabled
	@Override
	public void init() {
		//Get all the motors
        motorRight1 = hardwareMap.dcMotor.get("motor_1");
        motorRight2 = hardwareMap.dcMotor.get("motor_3");
		motorLeft1 = hardwareMap.dcMotor.get("motor_2");
        motorLeft2 = hardwareMap.dcMotor.get("motor_4");

		//Reverse direction of left motor
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

		//Get all servos
		arm = hardwareMap.servo.get("servo_1");
        joint = hardwareMap.servo.get("servo_2");
        claw = hardwareMap.servo.get("servo_3");
        claw2 = hardwareMap.servo.get("servo_4");

		//Assign the starting position of the wrist and claw
		armPosition = armStartingPosition;
		clawPosition = clawStartingPosition;
        jointPosition = jointStartingPosition;

        arm.setPosition(armPosition);
        claw.setPosition(clawPosition);
        joint.setPosition(jointPosition);

        mc = (MatrixDcMotorController)hardwareMap.dcMotorController.get("MatrixController");
        motorLeft1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        sc = hardwareMap.servoController.get("MatrixController");
        sc.pwmEnable();
	}

    //Loop every couple of ms
	@Override
	public void loop() {
        if(clock % 10 == 0) {
            ///////////////////////////////
            ///////////////////////////////
            //GAMEPAD1, DRIVING THE ROBOT//
            ///////////////////////////////
            ///////////////////////////////

            //Rotate around axis
            if(gamepad1.right_stick_x>0||gamepad1.right_stick_x<0) {
                //Get steering direction
                float direction = gamepad1.right_stick_x;
                left = direction;
                right = -direction;
                //Steer to the right direction
                motorRight1.setPower(right);
                motorRight2.setPower(right);
                motorLeft1.setPower(left);
                motorLeft2.setPower(left);
            }else {
                ////////////////
                //Acceleration//
                ////////////////
                //Get right trigger - Forwards
                float acceleration = gamepad1.right_trigger;
                //Get left trigger - Backwards
                float decelelation = gamepad1.left_trigger;

                //Clip acceleration and deceleration so it doesn't exceed 1 (full motor power)
                acceleration = Range.clip(acceleration, 0, 1);
                decelelation = Range.clip(decelelation, 0, 1);

                //Scale input so it scales exponentially
                acceleration = (float) scaleInput(acceleration);
                decelelation = (float) scaleInput(decelelation);
                DbgLog.msg("deceleration: " + decelelation);

                //Calculate negative acceleration if deceleration is pressed
                if (decelelation > 0) {
                    acceleration = 0 - decelelation;
                    DbgLog.msg("acceleration is now " + acceleration);
                }

                /////////////
                //Direction//
                /////////////
                //Get left stick
                float direction = gamepad1.left_stick_x;
                direction = Range.clip(direction, -1, 1);

                //Calculate speed of left and right motor
                left = acceleration;
                right = acceleration;
                if (direction < 0) {
                    if (acceleration > 0) left = acceleration + direction;
                    if (acceleration < 0) left = acceleration - direction;
                    DbgLog.msg("Left is " + acceleration + "+" + direction + "=" + (acceleration + direction));
                } else if (direction > 0) {
                    if (acceleration > 0) right = acceleration - direction;
                    if (acceleration < 0) right = acceleration + direction;
                    DbgLog.msg("Right is " + acceleration + "-" + direction + "=" + (acceleration - direction));
                }

                if (acceleration > 0) {
                    left = Range.clip(left, 0, 1);
                    right = Range.clip(right, 0, 1);
                } else if (acceleration < 0) {
                    left = Range.clip(left, -1, 0);
                    right = Range.clip(right, -1, 0);
                }

                /////////////
                //Set speed//
                /////////////
                //After here, left and right motor power are calculated, so set values to motor
                motorLeft1.setPower(left);
                motorLeft2.setPower(left);
                motorRight1.setPower(right);
                motorRight2.setPower(right);
            }

            //////////////////////////////
            //////////////////////////////
            //GAMEPAD2, ARM, CLAW, JOINT//
            //////////////////////////////
            //////////////////////////////

            ////////////////
            //Arm Rotation//
            ////////////////
            // If the left stick is pressed, arm rotates
            if (gamepad2.left_stick_x < 0) {
                armPosition += armDelta;
            } else if (gamepad2.left_stick_x > 0) {
                armPosition -= armDelta;
            }
            //Make sure arm doesn't exceed min/max position
            armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
            //Set calculated position of arm
            arm.setPosition(armPosition);

            //////////////////
            //Joint position//
            //////////////////
            //If Y is pressed, arm moves up
            if (gamepad2.y) {
                jointPosition += jointDelta;
            }

            //If A is pressed, arm moves down
            if (gamepad2.a) {
                jointPosition -= jointDelta;
            }

            //Make sure joint doesn't exceed min/max range
            jointPosition = Range.clip(jointPosition, JOINT_MIN_RANGE, JOINT_MAX_RANGE);

            // Set position of joint
            joint.setPosition(jointPosition);

            ////////////////
            //Claw opening//
            ////////////////
            //If right stick is pressed, claw opens/closes
            if (gamepad2.right_stick_y > 0) {
                clawPosition += clawDelta;
            } else if (gamepad2.right_stick_y < 0) {
                clawPosition -= clawDelta;
            }

            //Make sure claw doesn't exceed min/max range
            clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

            //Set position of arm
            claw.setPosition(clawPosition);
            claw2.setPosition(1 - clawPosition);

            /////////////
            //Telemetry//
            /////////////
            //telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Arm", "Arm: " + String.valueOf(armPosition));
            telemetry.addData("Joint","Joint: "+String.valueOf(jointPosition));
            telemetry.addData("Claw", "Claw: " + String.valueOf(clawPosition));
            //telemetry.addData("hook", "hook: "+String.valueOf());
            telemetry.addData("Motor left", "Left: " + left);
            telemetry.addData("Motor right", "Right: " + right);
            //telemetry.addData("motor turn", "turn"+String.valueOf(motorTurn.getPower()));
        }
        clock += 1;
	}

	//Op Mode disabled
	@Override
	public void stop() {
        //Set arm to starting position when shut down
        arm.setPosition(armStartingPosition);
        claw.setPosition(clawStartingPosition);
        joint.setPosition(jointStartingPosition);

	}
	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60};

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}

		double dScale;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}

}
