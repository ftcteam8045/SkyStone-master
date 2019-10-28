package org.firstinspires.ftc.teamcode.Skystone;


import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.oldcode.AutoTransitioner;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_COEF;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_THRESHOLD;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.turn_MIN_SPEED;
//Lara + Liesel positioning code


@Autonomous(name = "testAuto", group = "Cosmo")
//@Disabled
public class testAuto extends LinearOpMode {

    /* Declare OpMode members. */
//    Hardware8045testbot Cosmo = new Hardware8045testbot();   // Use a Pushbot's hardware
    testHardware Cosmo = new testHardware();   // Use a Pushbot's hardware

    /**
     * Menu Parameter Initialization
     **/
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AWfr4/T/////AAAAGRMg80Ehu059mDMJI2h/y+4aBmz86AidOcs89UScq+n+QQyGFT4cZP+rzg1M9B/CW5bgDoVf16x6x3WlD5wYKZddt0UWQS65VIFPjZlM9ADBWvWJss9L1dj4X2LZydWltdeaBhkXTXFnKBkKLDcdTyC2ozJlcAUP0VnLMeI1n+f5jGx25+NdFTs0GPJYVrPQRjODb6hYdoHsffiOCsOKgDnzFsalKuff1u4Z8oihSY9pvv3me2gJjzrQKqp2gCRIZAXDdYzln28Z/8vNSU+aXr6eoRrNXPpYdAwyYI+fX2V9H04806eSUKsNYcPBSbVlhe2KoUsSD7qbOsBMagcEIdMZxo010kVCHHhnhV3IFIs8";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public int waitTime1 = 0;
    public int driveDis1 = 16;
    public int driveDis2 = 22;
    public int driveDis3 = 10; //forward+backward
    public int driveDis4 = 30;

    public boolean loadingPosition = false;
    public boolean buildingPosition = false;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;


    @Override
    public void runOpMode() {

        final double FORWARD_SPEED = 0.3;
        final double TURN_SPEED = 0.3;
        final int cycletime = 500;
        int goldPosition = 2;   // 0 is on left, 1 in center, 2 on right


        /*
         * Initialize the drive system variables.the Robot
         * The init() method of the hardware class does all the work here
         */
        Cosmo.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        /**************************************************************
         // Actual Init loop
         *************************************************************/
        while (!opModeIsActive() && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
        }


        /**************************************************************
         // End Init loop
         *************************************************************/


        // Wait for the game to start (driver presses PLAY) replaced by init loop
        //       waitForStart();


        /**************************************************************
         // Actual RUN instructions
         *************************************************************/
        tfod.deactivate();


//        if (loadingPosition){            /** loading zone drive  **/
//
//        /** scans for skystone first **/
//            mecanumDrive(0.5, driveDis3, 0, 0); //drive forward
//            sleep(200);
//            mecanumTurn(0.5, -90); // turn left
//            sleep(200);
//            mecanumDrive(2.5, driveDis4, 0, 0); // drive forward
//            sleep(200);
//            /** place down block? **/
//            mecanumDrive(0.5, -driveDis3, 0, 0); //drive backward
//
//
//        }
//
//
//        if (buildingPosition){             /** building zone drive  **/
//
//            mecanumDrive(0.5, driveDis1,0, 0);       // drive forward
//            sleep(200);
//            mecanumDrive(.5, -driveDis1,0,0);        // drive backwards
//            sleep(200);
//            mecanumTurn(0.5, 90); //turn right
//            sleep(200);
//            mecanumDrive(0.5, driveDis1,0,0); //drive forward
//
//
//        }

//        Cosmo.leftFront.setPower(0);
//        Cosmo.rightFront.setPower(0);
//        Cosmo.leftRear.setPower(0);
//        Cosmo.rightRear.setPower(0);

    }
    /**************************************************************
     // End Actual Program Run
     *************************************************************/

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    //  Drive routine using the IMU and Mecanum wheels
    //  Robot Orientation is to the field
    //  Drive direction is from the robot
    //

    //  Drive routine using the IMU and Mecanum wheels
    //  Robot Orientation is to the field
    //  Drive direction is from the robot

//    public void mecanumDrive(double speed, double distance, double robot_orientation, double drive_direction) {
//        double max;
//        double multiplier;
//        int right_start;
//        int left_start;
//        int moveCounts;
//        //int drive_direction = -90;
//        moveCounts = (int) (distance * Cosmo.COUNTS_PER_INCH);
//        right_start = Cosmo.rightRear.getCurrentPosition();
//        left_start = Cosmo.leftRear.getCurrentPosition();
//        double lfpower;
//        double lrpower;
//        double rfpower;
//        double rrpower;
//
//        double lfbase;
//        double lrbase;
//        double rfbase;
//        double rrbase;
//        lfbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
//        lrbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
//        rfbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
//        rrbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
//        while (((abs(Cosmo.rightRear.getCurrentPosition() - right_start) + abs(Cosmo.leftRear.getCurrentPosition() - left_start)) / 2 < abs(moveCounts)) && opModeIsActive()  /* ENCODERS*/) {//Should we average all four motors?
//            //Determine correction
//            double correction = robot_orientation - getheading();
//            if (correction <= -180) {
//                correction += 360;
//            } else if (correction >= 180) {                      // correction should be +/- 180 (to the left negative, right positive)
//                correction -= 360;
//            }
//            lrpower = lrbase; //MIGHT BE MORE EFFECIENT TO COMBINE THESE WITHT HE ADJUSTMENT PART AND SET ADJUSTMENT TO ZERO IF NOT NEEDED
//            lfpower = lfbase;
//            rrpower = rrbase;
//            rfpower = rfbase;
//            if (abs(correction) > drive_THRESHOLD) {//If you are off
//                //Apply power to one side of the robot to turn the robot back to the right heading
//                double right_adjustment = Range.clip((drive_COEF * correction / 45), -1, 1);
//                lrpower -= right_adjustment;
//                lfpower -= right_adjustment;
//                rrpower = rrbase + right_adjustment;
//                rfpower = rfbase + right_adjustment;
//
//            }//Otherwise you Are at the right orientation
//
//            //Determine largest power being applied in either direction
//            max = abs(lfpower);
//            if (abs(lrpower) > max) max = abs(lrpower);
//            if (abs(rfpower) > max) max = abs(rfpower);
//            if (abs(rrpower) > max) max = abs(rrpower);
//
//            multiplier = speed / max; //multiplier to adjust speeds of each wheel so you can have a max power of 1 on atleast 1 wheel
//
//            lfpower *= multiplier;
//            lrpower *= multiplier;
//            rfpower *= multiplier;
//            rrpower *= multiplier;
//
//            Cosmo.leftFront.setPower(lfpower);
//            Cosmo.leftRear.setPower(lrpower);
//            Cosmo.rightFront.setPower(rfpower);
//            Cosmo.rightRear.setPower(rrpower);
//
////            RobotLog.ii("[GromitIR] ", Double.toString(18.7754*Math.pow(sharpIRSensor.getVoltage(),-1.51)), Integer.toString(left_front.getCurrentPosition()));
//
//        }
//        //gromit.driveTrain.stopMotors();
//        Cosmo.leftFront.setPower(0.0);
//        Cosmo.rightFront.setPower(0.0);
//        Cosmo.rightRear.setPower(0.0);
//        Cosmo.leftRear.setPower(0.0);
//    }


    // Turn using the IMU and meccanum drive
//    public void mecanumTurn(double speed, double target_heading) {
//        if (speed > 1) speed = 1.0;
//        //else if(speed <= 0) speed = 0.1;
//
//        double correction = target_heading - getheading();
//        if (correction <= -180) {
//            correction += 360;   // correction should be +/- 180 (to the left negative, right positive)
//        } else if (correction >= 180) {
//            correction -= 360;
//        }
//
//        while (abs(correction) >= Cosmo.turn_THRESHOLD && opModeIsActive()) { //opmode active?{
//            correction = target_heading - getheading();
//            if (abs(correction) <= Cosmo.turn_THRESHOLD) break;
//
//            if (correction <= -180)
//                correction += 360;   // correction should be +/- 180 (to the left negative, right positive)
//            if (correction >= 180) correction -= 360;
//            /*^^^^^^^^^^^MAYBE WE ONLY NEED TO DO THIS ONCE?????*/
//
//            double adjustment = Range.clip((Math.signum(correction) * Cosmo.turn_MIN_SPEED + Cosmo.turn_COEF * correction / 100), -1, 1);  // adjustment is motor power: sign of correction *0.07 (base power)  + a proportional bit
//
//            Cosmo.leftFront.setPower(-adjustment * speed);
//            Cosmo.leftRear.setPower(-adjustment * speed);
//            Cosmo.rightFront.setPower((adjustment * speed));
//            Cosmo.rightRear.setPower((adjustment * speed));
//        }
////        gromit.driveTrain.stopMotors();
//        Cosmo.leftFront.setPower(0.0);
//        Cosmo.rightFront.setPower(0.0);
//        Cosmo.rightRear.setPower(0.0);
//        Cosmo.leftRear.setPower(0.0);
//    }


//    public float getheading() {
//        // Acquiring the angles is relatively expensive; we don't want
//        // to do that in each of the three items that need that info, as that's
//        // three times the necessary expense.
//        angles = Cosmo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle; //For a -180 to 180 range
//        //return (angles.firstAngle + 180 + 180)%360; // for a zero to 360 range
//    }

    /*************************************************************************************************\
     |--------------------------------- Eli Edit Method ----------------------------------------------|
     \************************************************************************************************/
//
//    public void editParameters() {
//
//        String arrow01 = " ";
//        String arrow0 = " ";
//        String arrow1 = " ";
//        String arrow2 = " ";
//        String arrow3 = " ";
//        String arrow4 = " ";
//        String arrow5 = " ";
//        String arrow6 = " ";
//        String arrow7 = " ";
//        String arrow8 = " ";
//        String arrow9 = " ";
//        String arrow10 = " ";
//        String arrow11 = " ";
//        String arrow12 = " ";
//        String arrow13 = " ";
//        boolean dpadPressedUp = false;
//        boolean dpadPressedDown = false;
//        boolean dpadPressedLeft = false;
//        boolean dpadPressedRight = false;
//        String[] position = {"base", "crater"};
//        int positionIndex = 0;
//        if (craterPosition) positionIndex = 1;
//
//        String[] color = {"Blue", "Red"};
//        int colorIndex = 0;
//        if (teamIsRed) colorIndex = 1;
//
//        String[] botName = {"Real Bot", "TestBot"};
//        int botIndex = 0;
//        if (testBot) botIndex = 1;
//
//        int currentEdit = 1;
//
//        while (!gamepad1.right_stick_button && !opModeIsActive() && !isStopRequested()) {   // while haven't presse exit button, not in play mode, and not in stop
//            telemetry.addLine("===> Press Right Joystick to exit EDIT mode <===");
//            // Send telemetry message to signify robot waiting;
//
//
//            telemetry.addLine("Use Dpad to Navigate & change");
//            telemetry.addLine().addData("", currentEdit).addData("current edit number", ' ');
//            telemetry.addLine().addData(arrow2, positionIndex).addData(position[positionIndex], arrow2);
//            telemetry.addLine().addData(arrow3, botIndex).addData(botName[botIndex], arrow3);
//            telemetry.addLine().addData(arrow4, driveDis1).addData("First drive foreward", arrow4);
//            telemetry.addLine().addData(arrow5, driveDis2).addData("Distance 2", arrow5);
//            telemetry.addLine().addData(arrow6, driveDis3).addData("Distance 3", arrow6);
//
////            telemetry.addLine().addData(arrow4, "Color       ");
//            telemetry.update();
//
//            if (gamepad1.dpad_down) {
//                dpadPressedDown = true;
//            } else if (gamepad1.dpad_down == false && dpadPressedDown) {
//                dpadPressedDown = false;
//                currentEdit += 1;
//                if (currentEdit > 13) {
//                    currentEdit = -1;
//                }
//            }
//
//            if (gamepad1.dpad_up) {
//                dpadPressedUp = true;
//            } else if (gamepad1.dpad_up == false && dpadPressedUp) {
//                dpadPressedUp = false;
//                currentEdit -= 1;
//                if (currentEdit < -1) {
//                    currentEdit = 13;
//                }
//            }
//
//            if (currentEdit == -1) {
//                arrow01 = "<>";
//            } else {
//                arrow01 = "    ";
//            }
//            if (currentEdit == 0) {
//                arrow0 = "<>";
//            } else {
//                arrow0 = "    ";
//            }
//            if (currentEdit == 1) {
//                arrow1 = "<>";
//            } else {
//                arrow1 = "    ";
//            }
//
//
//
//            if (gamepad1.dpad_left) {
//                dpadPressedLeft = true;
//            } else if (gamepad1.dpad_left == false && dpadPressedLeft) {
//                dpadPressedLeft = false;
//
//
//                if (currentEdit == 0) {
//                    if (positionIndex == 1) {
//                        positionIndex = 0;
//                        craterPosition = false;
//                    } else {
//                        positionIndex = 1;
//                        craterPosition = true;
//                    }
//                }
//                if (currentEdit == 1) {
//                    if (botIndex == 1) {
//                        botIndex = 0;
//                        otherPosition = false;
//                    } else {
//                        botIndex = 1;
//                        otherPosition = true;
//                    }
//                }
//                if (currentEdit == 4) {
//                    driveDis1 -= 1;
//                }
//                if (currentEdit == 5) {
//                    driveDis2 -= 1;
//                }
//                if (currentEdit == 6) {
//                    driveDis3 -= 1;
//                }
//
//            }
//
//            if (gamepad1.dpad_right) {
//                dpadPressedRight = true;
//            } else if (gamepad1.dpad_right == false && dpadPressedRight) {
//                dpadPressedRight = false;
//
//
//                if (currentEdit == 0) {
//                    if (positionIndex == 1) {
//                        positionIndex = 0;
//                        craterPosition = false;
//                    } else {
//                        positionIndex = 1;
//                        craterPosition = true;
//                    }
//                }
//                if (currentEdit == 1) {
//                    if (botIndex == 1) {
//                        botIndex = 0;
//                        otherPosition = false;
//                    } else {
//                        botIndex = 1;
//                        otherPosition = true;
//                    }
//                }
//                if (currentEdit == 4) {
//                    driveDis1 += 1;
//                }
//                if (currentEdit == 5) {
//                    driveDis2 += 1;
//                }
//                if (currentEdit == 6) {
//                    driveDis3 += 1;
//                }
//
//            }
////                if (gamepad1.y) {
////                    break;
////                }
//        }
//        telemetry.update();
//    }

}






