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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.Calendar;

/**
 * ControllerOp Mode for Driver Controlled Period
 */
public class AutonomousOpTest extends OpMode {
    Servo servoTest;
    ServoController sc;
    double servoPos = 0.9; //Min .4
    DcMotor dm1;
	DcMotor dm2;
	DcMotor dm3;
	DcMotor dm4;

	//Constructor
	public AutonomousOpTest() {
	}

	//This code runs first when Op Mode is enabled
	@Override
	public void init() {
		servoTest = hardwareMap.servo.get("servo1");
        sc = hardwareMap.servoController.get("matrix");
        dm1 = hardwareMap.dcMotor.get("motor1");
		dm2 = hardwareMap.dcMotor.get("motor2");
		dm3 = hardwareMap.dcMotor.get("motor3");
		dm4 = hardwareMap.dcMotor.get("motor4");
		dm1.setDirection(DcMotor.Direction.REVERSE);
		dm2.setDirection(DcMotor.Direction.REVERSE);

        dm1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		dm2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		dm3.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		dm4.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sc.pwmEnable();
        servoTest.setPosition(servoPos);
    }

    //Loop every couple of ms
	@Override
	public void loop() {
        servoTest.setPosition(servoPos);
        dm1.setPower(1);
		dm2.setPower(1);
		dm3.setPower(1);
		dm4.setPower(1);
        //telemetry.addData("servopos", String.valueOf(servoTest.getPosition()));

        //telemetry.addData("ultrasonic",String.valueOf(currentUltrasonicLevel));
        		//testServo.setPosition(1.0);
        //telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm","arm: "+String.valueOf(armPosition));
        //telemetry.addData("joint","joint: "+String.valueOf(jointPosition));
        //telemetry.addData("claw", "claw: "+String.valueOf(clawPosition));
        //telemetry.addData("hook", "hook: "+String.valueOf(hook.getPosition()));
        //telemetry.addData("motor left","left: "+String.valueOf(motorLeft1.getPower()));
        //telemetry.addData("motor right","right: "+String.valueOf(motorRight1.getPower()));
        //telemetry.addData("motor turn", "turn"+String.valueOf(motorTurn.getPower()));
	}

	//Op Mode disabled
	@Override
	public void stop() {
        //Set arm to starting position when shut down
        servoTest.setPosition(0.6);
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
