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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * ControllerOp Mode for Driver Controlled Period
 */
public class ControllerOp extends OpMode {

	// TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.20;
	final static double ARM_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.7;

	// position of the arm servo.
	double armPosition;

	// amount to change the arm servo position.
	double armDelta = 0.1;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.1;

    //Current speed modus, the higher the faster
    int speedMode = 1;

	DcMotor motorRight;
	DcMotor motorLeft;
	Servo claw;
	Servo arm;

	//Constructor
	public ControllerOp() {

	}

	//This code runs first when Op Mode is enabled
	@Override
	public void init() {
		//Get all the motors
		motorRight = hardwareMap.dcMotor.get("motor_2");
		motorLeft = hardwareMap.dcMotor.get("motor_1");
		//Reverse direction of left motor
		motorLeft.setDirection(DcMotor.Direction.REVERSE);

		//Get all servos
		arm = hardwareMap.servo.get("servo_1");
		claw = hardwareMap.servo.get("servo_6");

		//Assign the starting position of the wrist and claw
		armPosition = 0.2;
		clawPosition = 0.2;
	}

    //Loop every couple of ms
	@Override
	public void loop() {
        //Change speed modus
        if(gamepad1.right_bumper) {
            speedMode = speedMode +  1;
        }else if(gamepad1.left_bumper){
            speedMode = speedMode - 1;
        }
        //Range speed modus from one to three (three being max speed, one being minimum)
        speedMode = (int) Range.clip(speedMode,1,3);

        //Forwards
		float acceleration = gamepad1.right_trigger;
        //Backwards
        float decelelation = gamepad1.left_trigger;

        if(decelelation>0){
            acceleration = 0-decelelation;
        }

        acceleration = calculateAcceleration(acceleration);

        //Direction
        //left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        float direction = gamepad1.left_stick_x;
        direction = Range.clip(direction,-1,1);

        if(speedMode == 1){
            direction = direction/4;
        }else if(speedMode == 2){
            direction = direction/2;
        }
        float left = 0;
        float right = 0;
        if(direction < 0){
            left = acceleration+direction;
        }else if(direction > 0){
            right = acceleration-direction;
        }

        //After here, left and right motor power are calculated, so set values to motor
        motorLeft.setPower(left);
        motorRight.setPower(right);



        telemetry.addData("Text", "*** Robot Data***");

	}

    public float calculateAcceleration(float acceleration){
        //Slowest mode
        if(speedMode == 1){
            //Max motor speed = 0.25
            acceleration = acceleration/4;

        }
        //Middle mode
        else if(speedMode == 2){
            //Max motor speed = 0.5
            acceleration = acceleration/2;
        }
        //Fastest mode
        else if(speedMode == 3){
            //Max motor speed = 1

        }
        return acceleration;
    }

	//Op Mode disabled
	@Override
	public void stop() {

	}
	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}

}
