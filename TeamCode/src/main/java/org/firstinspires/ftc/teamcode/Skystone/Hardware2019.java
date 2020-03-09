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

package org.firstinspires.ftc.teamcode.Skystone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
public class Hardware2019
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;
    public DcMotor  leftIntake    = null;
    public DcMotor  rightIntake   = null;
    public DcMotor  leftLift    = null;
    public DcMotor  rightLift   = null;
    public Servo clamp1 = null;
    public Servo clamp2 = null;
    public Servo clawTop = null;
    public Servo clawMid = null;
    public Servo clawBot = null;
    public Servo armGrabber = null;

    //public com.qualcomm.hardware.rev.RevBlinkinLedDriver LEDDriver;

    //    public Servo    rightClaw   = null;

    //public com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern LEDpattern;

    // The IMU sensor object
    public BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;




    public final double turn_THRESHOLD = 2.0;
    public static final double drive_THRESHOLD = 1.0;
    public final double turn_MIN_SPEED = 0.15;
    public final double turn_COEF = 1.0;
    public static final double drive_COEF = 1.0; //Maximum additional speed to add to a motor during a gyro drive

////    public final double WHEEL_DIAMETER = 4.0;
////    public final double GEAR_RATIO = 24/24;
////    public final double TICKS_REV = 537.6;
////    public final double COUNTS_PER_INCH = (TICKS_REV * GEAR_RATIO) / (WHEEL_DIAMETER * 3.1415);
    public final double COUNTS_PER_INCH = 72;   // Yellowjacket 223 24/32 gear ratio   100 inches drove only 62
//    public final double COUNTS_PER_INCH = 60;   // Yellowjacket 223 32/32 gear ratio
//    public final double COUNTS_PER_INCH = 32;   // Neverest 20 orbitals 24/32 gear ratio
//    public final double COUNTS_PER_INCH = 42;   // Neverest 20 orbitals 32/32 gear ratio
//    public final double COUNTS_PER_INCH = 114;  // Tetrix Direct
//    public final double COUNTS_PER_CM = COUNTS_PER_INCH / 2.54 ;

//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;




    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

   // DistanceSensor sensor;

    public ColorSensor sensorColor;
    public DistanceSensor frontSensor;
    public DistanceSensor backSensor;
    public DistanceSensor sideSensor;
    public AnalogInput batterySensor;
    public AnalogInput sharp2;



    /* Constructor */
    public Hardware2019(){




    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;



        // LED lights
        //LEDDriver = hwMap.get(com.qualcomm.hardware.rev.RevBlinkinLedDriver.class, "ledlights");
        //LEDpattern =  com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

        clamp1 = hwMap.get(Servo.class, "clamp_1");
        clamp2 = hwMap.get(Servo.class, "clamp_2");
        clawTop = hwMap.get(Servo.class, "claw_top");
        clawMid = hwMap.get(Servo.class, "claw_mid");
        clawBot = hwMap.get(Servo.class, "claw_bot");
        armGrabber = hwMap.get(Servo.class, "arm_grab");


        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftRear  = hwMap.get(DcMotor.class, "left_rear");
        rightRear = hwMap.get(DcMotor.class, "right_rear");
        leftIntake  = hwMap.get(DcMotor.class, "left_intake");
        rightIntake = hwMap.get(DcMotor.class, "right_intake");
        leftLift  = hwMap.get(DcMotor.class, "left_lift");
        rightLift = hwMap.get(DcMotor.class, "right_lift");
        // this should be for neverest  & using it for Matrix/yellowjackets as well.
         leftFront.setDirection(DcMotor.Direction.FORWARD);
         leftRear.setDirection(DcMotor.Direction.FORWARD);
         rightFront.setDirection(DcMotor.Direction.REVERSE);
         rightRear.setDirection(DcMotor.Direction.REVERSE);
         rightIntake.setDirection(DcMotor.Direction.REVERSE);
         rightLift.setDirection(DcMotor.Direction.REVERSE);
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

////this is for Tetrix
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        rightRear.setDirection(DcMotor.Direction.FORWARD);


//        liftmotor.setDirection (DcMotor.Direction.FORWARD);  //   old configuration

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);

        // Define and Initialize Motors
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.



//        sweepServo = hwMap.get(CRServo.class, "sweeper"  );
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



        //sensor = hwMap.get(DistanceSensor.class, "sensor");
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color");
        frontSensor = hwMap.get(DistanceSensor.class, "dis_sensor1");
        backSensor = hwMap.get(DistanceSensor.class, "dis_sensor2");
        sideSensor = hwMap.get(DistanceSensor.class, "side_sensor");
        batterySensor = hwMap.analogInput.get("dis_battery");
        sharp2 = hwMap.analogInput.get("sharp_2");




    }


}

