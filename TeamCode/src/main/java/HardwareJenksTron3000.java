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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */

public class HardwareJenksTron3000
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  frontDrive  = null;
    public DcMotor  backDrive   = null;
    public Servo    foundation1  = null;
    public Servo    foundation2 = null;

    public static final double FOUNDATION1_START =  0.95;
    public static final double FOUNDATION1_GRAB  =  0.20;

    public static final double FOUNDATION2_START =  0.20;
    public static final double FOUNDATION2_GRAB  =  0.95;

    public static int driveOrient = 0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    // sensors
    //ModernRoboticsI2cRangeSensor rangeSensor;

    /* Constructor */
    public HardwareJenksTron3000(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //sensor initialization
        //rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");

        // Define and Initialize Motors
        leftDrive   = hwMap.get(DcMotor.class, "left_drive");
        rightDrive  = hwMap.get(DcMotor.class, "right_drive");
        frontDrive  = hwMap.get(DcMotor.class, "front_drive");
        backDrive   = hwMap.get(DcMotor.class, "back_drive");

        //Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontDrive.setDirection(DcMotor.Direction.FORWARD);
        backDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        frontDrive.setPower(0);
        backDrive.setPower(0);

        // Set all drive motors to run using encoders.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.
        foundation1  = hwMap.get(Servo.class, "foundation1");
        foundation1.setPosition(FOUNDATION1_START);
        foundation2  = hwMap.get(Servo.class, "foundation2");
        foundation2.setPosition(FOUNDATION2_START);
    }


    //drive forward and backward based on the D-Pad direction selected (chooses the front)
    public void driveForwardBackward (double power){

        //the front is the front
        if(driveOrient == 0){
            leftDrive.setPower(-power/1.5);
            rightDrive.setPower(power/1.5);
            frontDrive.setPower(0.0);
            backDrive.setPower(0.0);
        }

        //the right is the front
        if(driveOrient == 1){
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
            frontDrive.setPower(power/1.5);
            backDrive.setPower(-power/1.5);
        }

        //the back is the front
        if(driveOrient == 2){
            leftDrive.setPower(power/1.5);
            rightDrive.setPower(-power/1.5);
            frontDrive.setPower(0.0);
            backDrive.setPower(0.0);
        }

        //the left is the front
        if(driveOrient == 3){
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
            frontDrive.setPower(-power/1.5);
            backDrive.setPower(power/1.5);
        }
    }


    //drive left and right based on the D-pad direction selected (chooses front of robot)
    public void driveRightLeft (double power) {

        //the front is the front
        if(driveOrient == 0){
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
            frontDrive.setPower(power/1.5);
            backDrive.setPower(-power/1.5);
        }

        //the right is the front
        if(driveOrient == 1){
            leftDrive.setPower(-power/1.5);
            rightDrive.setPower(power/1.5);
            frontDrive.setPower(0.0);
            backDrive.setPower(0.0);
        }

        //the back is the front
        if(driveOrient == 2){
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
            frontDrive.setPower(-power/1.5);
            backDrive.setPower(power/1.5);
        }

        //the left is the front
        if(driveOrient == 3){
            leftDrive.setPower(power/1.5);
            rightDrive.setPower(-power/1.5);
            frontDrive.setPower(0.0);
            backDrive.setPower(0.0);
        }
    }

    //rotate clockwise and counter clockwise
    public void rotate (double power){
        leftDrive.setPower(power/4.0);
        rightDrive.setPower(power/4.0);
        frontDrive.setPower(power/4.0);
        backDrive.setPower(power/4.0);
    }

    //stop all motors
    public void driveNotAtAll (){
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        frontDrive.setPower(0.0);
        backDrive.setPower(0.0);
    }

}

