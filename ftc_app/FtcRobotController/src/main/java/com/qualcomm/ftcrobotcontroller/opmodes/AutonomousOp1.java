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

import java.util.Calendar;

/**
 * ControllerOp Mode for Driver Controlled Period
 */
public class AutonomousOp1 extends OpMode {

	// TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.20;
	final static double ARM_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.70;
    final static double JOINT_MIN_RANGE = 0.20;
    final static double JOINT_MAX_RANGE = 0.90;

	// position of the arm servo.
	double armPosition;
	// amount to change the arm servo position.
	double armDelta = 0.1;

	// position of the claw motor
	double clawPosition;
    // amount to change the claw servo position by
    double clawDelta = 0.1;

    // position of the joint servo
    double jointPosition;
    // amount to change the joint servo position by
    double jointDelta = 0.1;

    //Position arm, joint, claw starts and ends in
    double armStartingPosition = 0.2;
    double jointStartingPosition = 0.2;
    double clawStartingPosition = 0.2;

    //Hook folded
    double hookStartingPosition = 0.2;
    //Hook unfolded
    double hookEndPosition = 0.6;

	DcMotor motorRight1;
    DcMotor motorRight2;
	DcMotor motorLeft1;
    DcMotor motorLeft2;
	Servo claw;
	Servo arm;
    Servo joint;
    Servo hook;
    DcMotor motorTurn;

    int currentMode;
    int startingHour;
    int startingMinute;
    int startingSeconds;

	//Constructor
	public AutonomousOp1() {

	}

	//This code runs first when Op Mode is enabled
	@Override
	public void init() {
		//Get all the motors
        motorRight1 = hardwareMap.dcMotor.get("motor_2");
        motorRight2 = hardwareMap.dcMotor.get("motor_1");
		motorLeft1 = hardwareMap.dcMotor.get("motor_3");
        motorLeft1 = hardwareMap.dcMotor.get("motor_4");
        motorTurn = hardwareMap.dcMotor.get("motor_5");

		//Reverse direction of left motor
		motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

		//Get all servos
		arm = hardwareMap.servo.get("servo_1");
        joint = hardwareMap.servo.get("servo_2");
        claw = hardwareMap.servo.get("servo_3");

		//Assign the starting position of the wrist and claw
		armPosition = armStartingPosition;
		clawPosition = clawStartingPosition;
        jointPosition = jointStartingPosition;

        currentMode = 1;

        Calendar calendar = Calendar.getInstance();
        startingHour = calendar.get(Calendar.HOUR_OF_DAY);
        startingMinute = calendar.get(Calendar.MINUTE);
        startingSeconds = calendar.get(Calendar.SECOND);

    }

    //Loop every couple of ms
	@Override
	public void loop() {
        if (currentMode==1){

        }else if(currentMode==2){

        }else if(currentMode==3){

        }else if (currentMode==4){

        }else if(currentMode==5){

        }else if(currentMode==6){

        }else if (currentMode==7){

        }else if(currentMode==8){

        }else if(currentMode==9){

        }else if (currentMode==10){

        }

        Calendar calendar = Calendar.getInstance();
        int currentHour = calendar.get(Calendar.HOUR_OF_DAY);
        int currentMinute = calendar.get(Calendar.MINUTE);
        int currentSecond = calendar.get(Calendar.SECOND);

        if(currentSecond-startingSeconds>29){
            //end program
        }
        else if(currentSecond-startingSeconds<0){
            currentSecond=currentSecond+60;
            if(currentSecond-startingSeconds>29){
                //end program
            }
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm","arm: "+String.valueOf(armPosition));
        telemetry.addData("joint","joint: "+String.valueOf(jointPosition));
        telemetry.addData("claw", "claw: "+String.valueOf(clawPosition));
        telemetry.addData("hook", "hook: "+String.valueOf(hook.getPosition()));
        telemetry.addData("motor left","left: "+String.valueOf(motorLeft1.getPower()));
        telemetry.addData("motor right","right: "+String.valueOf(motorRight1.getPower()));
        telemetry.addData("motor turn", "turn"+String.valueOf(motorTurn.getPower()));
	}

	//Op Mode disabled
	@Override
	public void stop() {
        //Set arm to starting position when shut down
        arm.setPosition(armStartingPosition);
        claw.setPosition(clawStartingPosition);
        joint.setPosition(jointStartingPosition);
        hook.setPosition(hookStartingPosition);
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

		double dScale;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}

}
