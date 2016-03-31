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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.Calendar;

/**
 * ControllerOp Mode for Driver Controlled Period
 */
public class AutonomousOpBlueTeamMountain1 extends OpMode {

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
    ColorSensor colorSensor1;
    UltrasonicSensor ultrasonicSensor1;

    int currentMode;
    int startingMinute;
    int driven;
    double startingSeconds;
    double modeStartingTime;
    int previousBlueColor;
    int previousRedColor;
    String firstColor;
    double ultrasoniclevel;

	//Constructor
	public AutonomousOpBlueTeamMountain1() {

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

        //Get all sensors
        colorSensor1 = hardwareMap.colorSensor.get("colorsensor");
        ultrasonicSensor1 = hardwareMap.ultrasonicSensor.get("ultrasonicsensor");


		//Assign the starting position of the wrist and claw
		armPosition = armStartingPosition;
		clawPosition = clawStartingPosition;
        jointPosition = jointStartingPosition;

        currentMode = 1;

        Calendar calendar = Calendar.getInstance();
        startingMinute = calendar.get(Calendar.MINUTE);
        startingSeconds = getTimeSeconds();

    }

    //Loop every couple of ms
	@Override
	public void loop() {
        //Drive forward out of parking spot on full speed for 2 seconds
        if (currentMode==1){
            if (compareTime(startingSeconds)<1){
                motorLeft1.setPower(1);
                motorLeft2.setPower(1);
                motorRight1.setPower(1);
                motorRight2.setPower(1);
            }else{
                //Go to the next mode after 2 seconds
                currentMode=2;
                modeStartingTime = getTimeSeconds();
            }
        }
        //Turn left 90 degrees
        else if(currentMode==2){
            if (compareTime(modeStartingTime)<1){
                motorLeft1.setPower(-1);
                motorLeft2.setPower(-1);
                motorRight1.setPower(1);
                motorRight2.setPower(1);
            }else{
                //After 1 second drive forward again
                //currentMode=3;
                modeStartingTime = getTimeSeconds();
            }
        }
        //Go forward
        else if(currentMode==3){
            //Go forward on full speed as long as there's nothing in front of the robot
            if (getUltrasonic()>20) {
                motorLeft1.setPower(1);
                motorLeft2.setPower(1);
                motorRight1.setPower(1);
                motorRight2.setPower(1);
                //Count how many times it has looped through this code to get a rough idea of how close the robot is to the wall
                driven++;
            }else{
                //Stop driving forward if there's something in front of the robot
                motorLeft1.setPower(0);
                motorLeft2.setPower(0);
                motorRight1.setPower(0);
                motorRight2.setPower(0);
                //If the driven variable is larger as 100 assume that the robot is now at the wall and go to the next mode
                //Otherwise assume that there's something else in front of the robot (another robot) and don't go to the next mode
                if(driven>100){
                    currentMode=4;
                    modeStartingTime = getTimeSeconds();
                }
            }
        }
        //Turn right 90 degrees when in front of the edge of the field
        else if (currentMode==4){
            if (compareTime(modeStartingTime)<1){
                motorLeft1.setPower(1);
                motorLeft2.setPower(1);
                motorRight1.setPower(-1);
                motorRight2.setPower(-1);
                motorTurn.setPower(-1);
            }else {
                //After turning for 1 second go to the next mode
                currentMode = 5;
                modeStartingTime = getTimeSeconds();
            }
        }
        //Drive to the beacon
        else if(currentMode==5){
            //Test if the robot is standing in the middle of the beacon where the previous color was red and current color is blue, or the other way around
            //and move forward until this is the case, so the robot will stop in the middle of the beacon
            if(!((previousBlueColor>1 && colorSensor1.blue()<1 && previousRedColor<1 && colorSensor1.red()>1)||(previousBlueColor<1 && colorSensor1.blue()>1 && previousRedColor>1 && colorSensor1.red()<1))){
                //Move forward if this is not the case
                motorLeft1.setPower(0.5);
                motorLeft2.setPower(0.5);
                motorRight1.setPower(0.5);
                motorRight2.setPower(0.5);
            }else{
                //If this is the case go to the next mode
                currentMode = 6;
                //Save the first color for later in the program to define if the robot has to move forward or backwards to press the button
                if(colorSensor1.blue()>1){
                    firstColor = "red";
                }else{
                    firstColor = "blue";
                }
            }
            //Get color of sensors for next run, so they can be compared to the current value of that run
            previousBlueColor = colorSensor1.blue();
            previousRedColor = colorSensor1.red();
        }
        //Drop the equipped climbers in the shelter
        else if(currentMode==6){
            //Drop climbers in shelter by setting the servos of the arm to the correct value
            arm.setPosition(1);
            joint.setPosition(1);
            claw.setPosition(1);
            //Go to the next mode once the climbers are dropped
            currentMode = 7;
            modeStartingTime = getTimeSeconds();
        }
        //Drive the robot next to the button
        else if (currentMode==7){
            if(compareTime(modeStartingTime)<1){
                //Drive for/backwards a little bit to stand next to the button of the blue side of the beacon
                if(firstColor.equals("red")) {
                    motorRight1.setPower(0.5);
                    motorRight2.setPower(0.5);
                    motorLeft1.setPower(0.5);
                    motorLeft2.setPower(0.5);
                }else{
                    motorRight1.setPower(-0.5);
                    motorRight2.setPower(-0.5);
                    motorLeft1.setPower(-0.5);
                    motorLeft2.setPower(-0.5);
                }
            }else{
                //After moving next to the button go to the next mode
                currentMode = 8;
            }
        }
        //Press the button on the beacon
        else if(currentMode==8){
            //Press the button
            arm.setPosition(1);
            joint.setPosition(1);
            claw.setPosition(1);
            //Then go to the next mode
            currentMode = 9;
            modeStartingTime = getTimeSeconds();
        }
        //Drive to  mountain 1
        else if(currentMode==9) {
            //Robot turns 225 degrees to the left
            if (compareTime(modeStartingTime) < 4) {
                motorLeft1.setPower(.3);
                motorLeft2.setPower(.3);
                motorRight1.setPower(1);
                motorRight2.setPower(1);
            }
            //Drives forward
            else if (compareTime(modeStartingTime) < 5) {
                motorLeft1.setPower(1);
                motorLeft2.setPower(1);
                motorRight1.setPower(1);
                motorRight2.setPower(1);
            } else {
                currentMode = 10;
                modeStartingTime = getTimeSeconds();
            }
        }
        else if (currentMode==10){
            //Drives forward, onto the mountain
            if (compareTime(modeStartingTime) < 5){
                motorLeft1.setPower(1);
                motorLeft2.setPower(1);
                motorRight1.setPower(1);
                motorRight2.setPower(1);
            }else{
                currentMode=11;
            }
        }

        Calendar calendar = Calendar.getInstance();
        double currentSecond = getTimeSeconds();

        if(compareTime(startingSeconds)>29){
            //end program
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

    private double getUltrasonic(){
        double currentUltrasonicLevel = ultrasonicSensor1.getUltrasonicLevel();
        if(currentUltrasonicLevel == 0 && ultrasoniclevel != 0){
            double temp = currentUltrasonicLevel;
            currentUltrasonicLevel = ultrasoniclevel;
            ultrasoniclevel = temp;
        }
        ultrasoniclevel = currentUltrasonicLevel;
        return currentUltrasonicLevel;
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

    double getTimeSeconds(){
        Calendar calendar = Calendar.getInstance();
        double second = (double) calendar.get(Calendar.SECOND);
        double millisecond = ((double)calendar.get(Calendar.MILLISECOND))/1000;
        return second+millisecond;
    }

    double compareTime(double firstTime){
        Calendar calendar = Calendar.getInstance();
        double second = (double) calendar.get(Calendar.SECOND);
        double millisecond = ((double)calendar.get(Calendar.MILLISECOND))/1000;
        double currentTime = second+millisecond;
        double delta = currentTime-firstTime;
        if(delta<0){
            delta = delta+60;
        }
        return delta;
    }

}
