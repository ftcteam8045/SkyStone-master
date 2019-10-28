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

package org.firstinspires.ftc.teamcode.oldcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 *
 */
public class Hardware8045testbot
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;
    public DcMotor  liftmotor   = null;
    public DcMotor  liftDown   = null;


    //    public Servo    rightClaw   = null;

    public com.qualcomm.hardware.rev.RevBlinkinLedDriver LEDDriver;
    //public com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern LEDpattern;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;




    public final double turn_THRESHOLD = 2.0;
    public static final double drive_THRESHOLD = 1.0;
    public final double turn_MIN_SPEED = 0.15;
    public final double turn_COEF = 1.0;
    public static final double drive_COEF = 1.0; //Maximum additional speed to add to a motor during a gyro drive


//    Tetrix motors 1440 counts per rev,  neverrest 40  1120cpr  NR O20 537.6  yellowjacket 223rpm: 753.2cpr  60rpm: 2,786cpr  30rpm: 5,264cpr
//    public final double WHEEL_DIAMETER = 4.0;
//    public final double GEAR_RATIO = 24/24;
//    public final double TICKS_REV = 537.6;
//    public final double COUNTS_PER_INCH = (TICKS_REV * GEAR_RATIO) / (WHEEL_DIAMETER * 3.1415);
//    public final double COUNTS_PER_INCH = 45;   // Yellowjacket 223 24/32 gear ratio
//    public final double COUNTS_PER_INCH = 60;   // Yellowjacket 223 32/32 gear ratio
//    public final double COUNTS_PER_INCH = 32;   // Neverest 20 orbitals 24/32 gear ratio
//    public final double COUNTS_PER_INCH = 42;   // Neverest 20 orbitals 32/32 gear ratio
    public final double COUNTS_PER_INCH = 114;  // Tetrix Direct
    public final double COUNTS_PER_CM = COUNTS_PER_INCH / 2.54 ;

//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;




    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware8045testbot(){




    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;



        // LED lights
        LEDDriver = hwMap.get(com.qualcomm.hardware.rev.RevBlinkinLedDriver.class, "ledlights");
        //LEDpattern =  com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;


        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftRear  = hwMap.get(DcMotor.class, "left_rear");
        rightRear = hwMap.get(DcMotor.class, "right_rear");
//        liftmotor = hwMap.get(DcMotor.class, "lift_motor");
//// this is for neverrest.  yellow jacket?
//         leftFront.setDirection(DcMotor.Direction.FORWARD);
//         leftRear.setDirection(DcMotor.Direction.FORWARD);
//         rightFront.setDirection(DcMotor.Direction.REVERSE);
//         rightRear.setDirection(DcMotor.Direction.REVERSE);

//   this is for Tetrix
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
//        liftmotor.setDirection (DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
//        liftmotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Motors

        // Define and initialize ALL installed servos.
//        leftClaw  = hwMap.get(Servo.class, "left_hand");
//        rightClaw = hwMap.get(Servo.class, "right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);

        /**
         * IMU SETUP
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. port 0 on rev hub
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



 }


}

