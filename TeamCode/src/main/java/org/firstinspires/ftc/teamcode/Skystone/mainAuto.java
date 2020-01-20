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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.oldcode.AutoTransitioner;

import java.util.List;
import java.util.Arrays;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_COEF;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_THRESHOLD;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.turn_MIN_SPEED;
//Lara + Liesel positioning code


@Autonomous(name = "Auto", group = "Cosmo")
//@Disabled
public class mainAuto extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2019 Cosmo = new Hardware2019();   // Use a Pushbot's hardware

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
    public boolean skystonePosition = true;
    public boolean teamIsRed = true;
    public boolean left = false;
    public boolean center = false;
    public boolean right = true;
    public double grayHueValue = 90.0;  // color sensor values
    public double redHueValue = 5;
    public double s = 0.72727;
    public double extraDistance = 0;
    public double blueHueValue = 189;
    public double grayRedBorder = (grayHueValue + redHueValue) / 2;
    public double grayBlueBorder = (grayHueValue + blueHueValue) / 2;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    public float values[] = hsvValues;
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

        AutoTransitioner.transitionOnStop(this, "MainTele");   // get ready for teleop at the end of auto


        /**************************************************************
         // Actual Init  loop
         *************************************************************/
        while (!opModeIsActive() && !isStopRequested()) {
            if (tfod != null) {


                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    telemetry.addLine("Main Auto");


                    telemetry.addLine(" Press Left Joystick for Edit");
                    telemetry.addLine("------------------------------------------");
                    telemetry.addData("side sensor", Cosmo.sideSensor.getDistance(DistanceUnit.MM));
                    if (left) telemetry.addLine("LEFT");
                    if (center) telemetry.addLine("CENTER");
                    if (right) telemetry.addLine("RIGHT");

                    if (teamIsRed) {
                        telemetry.addData("", "RED");
                    } else {
                        telemetry.addData("", "BLUE");
                    }
                    if (skystonePosition) {
                        telemetry.addData("", "Skystone Position");
                    } else {
                        telemetry.addData("", "Foundation Position");
                    }


                    boolean hasSeenStone = false;  //set to false every init loop to check for updated stone recognitions each for-each loop

                    // step through the list of recognitions and display boundary info.
                    int i = 0;


                    if(!teamIsRed) {  // BLUE SIDE OBJECT DETECTION
                        hasSeenStone = false;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("", updatedRecognitions.size());
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            telemetry.addData("", recognition.getRight());

                            if (recognition.getLabel().equals("Skystone") && recognition.getTop() > 480) {


                                    if (recognition.getLeft() < 300) {
                                        hasSeenStone = true;  //check for skystone
                                        left = true;
                                        center = false;
                                        right = false;
                                    } else if (recognition.getLeft() > 300) {
                                        hasSeenStone = true; //check for skystone
                                        left = false;
                                        center = true;
                                        right = false;

                                    }

                            }

                            if (!hasSeenStone) {
                                left = false;  //no skystone visible
                                center = false;
                                right = true;

                            }



                        }
                    }

                    if(teamIsRed) { // RED SIDE OBJECT DETECTION
                        hasSeenStone = false;

                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("", updatedRecognitions.size());
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            telemetry.addData("", recognition.getRight());

                            if (recognition.getLabel().equals("Skystone") && recognition.getTop() > 480) {

                                //RED SIDE AUTO DETECTION NUMBERS
                                if (recognition.getRight() < 350) {
                                    hasSeenStone = true;
                                    left = false; //check for skystone
                                    center = false;
                                    right = true;
                                } else if (recognition.getLeft() > 400) {
                                    hasSeenStone = true;
                                    left = false; //check for skystone
                                    center = true;
                                    right = false;

                                }

                            }


                            if((recognition.getLabel().equals("Stone")) && recognition.getLeft() < 30 && recognition.getRight() > 600){
                                left = false;  //no skystone visible
                                center = false;
                                right = true;
                            } else if (!hasSeenStone) {
                                left = true; //no skystone visible
                                center = false;
                                right = false;

                            }





                        }
                    }


                    telemetry.update();
                }

            }


            if (gamepad1.back || gamepad1.left_stick_button) {             // edit parameters  & write the new file

                editParameters();

            }

        }


        /**************************************************************
         // End Init loop
         *************************************************************/
        /**************************************************************
         // Actual RUN instructions
         *************************************************************/


        tfod.deactivate();


        if (teamIsRed) { /** RED SIDE CODE **/

            if (skystonePosition) {            /** skystone zone drive  **/

                /** scans for skystone first **/
                Cosmo.clamp1.setPosition(0.99);
                Cosmo.clamp2.setPosition(0.08);
                mecanumDrive(0.6, -15*s, 0, 0); //drive forward
                grabSkystone();
                deliverBlock(30 + extraDistance, 1);
                mecanumDrive(0.9, -18.8, 90, 0);
                mecanumDrivetoTape(0.3,  -10, 90, 0);
                grabSkystoneAgain();
                if(!left) {
                    deliverBlock(42.6 + extraDistance, 1);
                } else  {
                    mecanumDrive(0.9, (34 + extraDistance) * s , 90, 0);
                }
                mecanumTurn(1.0, 180); // turn right again
                if(!left) {
                    mecanumDrive(0.8, -9.5 * s, 180, -90); //strafe over
                }Cosmo.clamp1.setPosition(0.65);
                Cosmo.clamp2.setPosition(0.35);
                if(!left) {
                    mecanumDrive(0.6, 2 * s, 180, 0); //drive forward
                } else {
                    mecanumDrive(0.6, 4.5 * s, 180, 0); //drive forward
                }
                mecanumDrive(0.25, 2.5*s, 180, 0); //drive forward
                Cosmo.clamp1.setPosition(0.42);
                Cosmo.clamp2.setPosition(0.65);
                sleep(400);
                mecanumDrive(0.8, -14.5*s, 155, -30); //drive back from foundation
                mecanumDrive(0.8, -14*s, 90, 0); //drive back from foundation
                if(left){
                    Cosmo.leftIntake.setPower(-0.4);
                    Cosmo.rightIntake.setPower(-0.4);
                }
                Cosmo.clamp1.setPosition(0.99);
                Cosmo.clamp2.setPosition(0.08);
                mecanumDrive(0.6, 12*s, 90, 0); //place foundation
                if(left){
                    Cosmo.leftIntake.setPower(0.0);
                    Cosmo.rightIntake.setPower(0.0);
                }
                Cosmo.clamp1.setPosition(0.99);
                Cosmo.clamp2.setPosition(0.08);
                mecanumDrive(0.8, 11*s, 90, 90); //strafe over
                mecanumDrive(0.7, (-30+ extraDistance) *s , 91, 0);  //drive until tape is detected


            } else {             /** foundation zone drive  **/


                mecanumDrivetoTape(0.6, 12 * s, 0, 0);


            }
        } else {     /** BLUE SIDE CODE **/

            if (skystonePosition) {            /** skystone zone drive  **/

                /** scans for skystone first **/
                Cosmo.clamp1.setPosition(0.99);
                Cosmo.clamp2.setPosition(0.08);
                mecanumDrive(0.6, -15*s, 0, 0); //drive forward
                grabSkystone();
                deliverBlock(-23 + extraDistance, 1);
                mecanumDrive(0.9, 12.6, 90, 0);
                mecanumDrivetoTape(0.3,  10, 90, 0);
                grabSkystoneAgain();
                if(!right) {
                    deliverBlock(-33.85 + extraDistance, 1);
                } else  {
                    mecanumDrive(0.9, (34  + extraDistance) * s, -90, 0);
                }
                mecanumTurn(1.0, -180); // turn right again
                mecanumDrive(0.8, 5*s, -180, -90); //strafe over
                Cosmo.clamp1.setPosition(0.65);
                Cosmo.clamp2.setPosition(0.35);
                mecanumDrive(0.6, 2*s, -180, 0); //drive forward
                mecanumDrive(0.25, 2.5*s, -180, 0); //drive forward
                Cosmo.clamp1.setPosition(0.42);
                Cosmo.clamp2.setPosition(0.65);
                sleep(400);
                mecanumDrive(0.8, -14.5*s, -155, 30); //drive back from foundation
                mecanumDrive(0.9, -14*s, -90, 0); //drive back from foundation
                if(right){
                    Cosmo.leftIntake.setPower(-0.4);
                    Cosmo.rightIntake.setPower(-0.4);
                }
                Cosmo.clamp1.setPosition(0.99);
                Cosmo.clamp2.setPosition(0.08);
                mecanumDrive(0.6, 12*s, -90, 0); //place foundation
                if(right){
                    Cosmo.leftIntake.setPower(0.0);
                    Cosmo.rightIntake.setPower(0.0);
                }
                Cosmo.clamp1.setPosition(0.99);
                Cosmo.clamp2.setPosition(0.08);
                mecanumDrive(0.8, 9*s, -90, -90); //strafe over
                mecanumDrive(1.0, (-29 + extraDistance) *s, -91, 0);  //drive until tape is detected


            } else {             /** foundation zone drive  **/


                mecanumDrivetoTape(0.6, 12 * s, 0, 0);


            }


        }
        Cosmo.leftFront.setPower(0);
        Cosmo.rightFront.setPower(0);
        Cosmo.leftRear.setPower(0);
        Cosmo.rightRear.setPower(0);

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

        // skystone trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.25;
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

    public void grabSkystone() {


        if (teamIsRed) {  //TEAM IS RED
            if (left) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, -1.70 * s, 90, 0);
                mecanumDrivetoObject(0.4, -9 * s, 90, -90, 124);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8.6
                        * s, 90, -90);
                mecanumDrive(0.9, 18 * s, 89, 0);
                mecanumDrivetoTape(0.3, 10 * s, 90, 0);

            }

            if (center) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, -12.5 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, 32.1 * s, 89, 0);
                mecanumDrivetoTape(0.3, 12 * s, 90, 0);
            }

            if (right) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, -7 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, 27.5 * s, 89, 0);
                mecanumDrivetoTape(0.3, 12 * s, 90, 0);

            }

        } else { //TEAM IS BLUE
            if (left) {  // GRAB FIRST SKYSTONE
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, 10.3 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, -36.5 * s, 89, 0);
                mecanumDrivetoTape(0.3, -14 * s, 90, 0);
            }

            if (center) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, 16.1 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, -39.5 * s, 90, 0);
                mecanumDrivetoTape(0.3, -14 * s, 90, 0);

            }

            if (right) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, 5.1 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 124);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, -37.5 * s, 89, 0);
                mecanumDrivetoTape(0.3, -10 * s, 90, 0);

            }
        }

    }


    public void grabSkystoneAgain() {


        if (teamIsRed) {  //TEAM IS RED
            if (left) {
                mecanumDrive(0.8, -25 * s, 90, 0);
                mecanumDrive(0.7, 18*s, 90, 90);
                sleep(200);
                Cosmo.leftIntake.setPower(0.8);
                Cosmo.rightIntake.setPower(0.8);
                mecanumDrive(0.35, -4.6*s, 90, 0);
                Cosmo.leftIntake.setPower(0.0);
                Cosmo.rightIntake.setPower(0.0);
                mecanumDrive(0.45, 4.9*s, 90, 0);
                mecanumDrive(0.7, -16.7*s, 90, 90);
                mecanumDrive(0.9, 22 * s, 90, 0);
                mecanumDrivetoTape(0.3, 12 * s, 90, 0);

            }

            if (center) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, -15.8 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, 16.9 * s, 89, 0);
                mecanumDrivetoTape(0.3, 15 * s, 90, 0);
            }

            if (right) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, -10.5 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, 10 * s, 90, 0);
                mecanumDrivetoTape(0.3, 10 * s, 90, 0);

            }

        } else { //TEAM IS BLUE
            if (left) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, 21.3 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, -21.8 * s, 90, 0);
                mecanumDrivetoTape(0.3, -10 * s, 90, 0);


            }

            if (center) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, 28.2 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 8 * s, 90, -90);
                mecanumDrive(0.9, -24 * s, 90, 0);
                mecanumDrivetoTape(0.3, -12 * s, 90, 0);
            }

            if (right) {
                mecanumDrive(0.8, 28.9 * s, 90, 0);
                mecanumTurn(1.0, -88); // turn right
                mecanumDrive(0.8, -3.15 * s, -90, 0);
                mecanumDrive(0.7, -15.2*s, -90, 90);
                Cosmo.leftIntake.setPower(0.8);
                Cosmo.rightIntake.setPower(0.8);
                mecanumDrive(0.35, -3.5*s, -90, 0);
                Cosmo.leftIntake.setPower(0.0);
                Cosmo.rightIntake.setPower(0.0);
                mecanumDrive(0.45, 4.3*s, -90, 0);
                mecanumDrive(0.7, 16*s, -90, 90);
                mecanumDrive(0.9, 22.5 * s, -90, 0);
                mecanumDrivetoTape(0.3, 12 * s, -90, 0);
            }
        }

    }


    public void readyToGrab(){
        Cosmo.clawBot.setPosition(0.36);
        Cosmo.clawMid.setPosition(0.0);
        Cosmo.clawTop.setPosition(0.98);  //ready to clamp


    }


    public void grabBlockWithClaw(){
        Cosmo.clawBot.setPosition(0.255);
        mecanumDrive(0.45, 0.3 * s, 90, -90);
        mecanumDrive(0.45, -0.3 * s, 90, -90);
        Cosmo.clawTop.setPosition(0.58);
        sleep(300);
        Cosmo.clawMid.setPosition(0.205); //clamp block
        sleep(200);
        Cosmo.clawBot.setPosition(0.78);
        Cosmo.clawMid.setPosition(0.205);
        Cosmo.clawTop.setPosition(0.58);
        sleep(600);

    }

    public void deliverBlock(double distanceFromTape, int layer){

        if(teamIsRed){
            mecanumDrive(0.9, distanceFromTape * s, 91, 0);

        }else {
            mecanumDrive(0.9, distanceFromTape * s, 89, 0);
        }
        mecanumDrivetoObject(0.45, -8 * s, 90, -90, 124);
        if(layer == 1) {
            Cosmo.clawBot.setPosition(0.4);
            sleep(200);
            Cosmo.clawTop.setPosition(0.98);  // drop block while lifting up, on 1st level
            sleep(250);
            Cosmo.clawBot.setPosition(0.78);
        } else if(layer == 2) {
            Cosmo.clawBot.setPosition(0.74);
            Cosmo.clawMid.setPosition(0.15);  // drop block on top of other block
            Cosmo.clawTop.setPosition(0.98);
            sleep(500);
            Cosmo.clawBot.setPosition(0.78);
            Cosmo.clawMid.setPosition(0.205);
            Cosmo.clawTop.setPosition(0.59);

        }
        mecanumDrive(0.5, 4.5 * s, 90, -90);
        Cosmo.clawBot.setPosition(0.79);
        Cosmo.clawMid.setPosition(0.85);
        sleep(100);
        Cosmo.clawTop.setPosition(0.25); // tuck in claw




    }

    public void mecanumDrive(double speed, double distance, double robot_orientation, double drive_direction) {
        double max;
        double multiplier;
        int right_start;
        int left_start;
        int moveCounts;
        //int drive_direction = -90;
        moveCounts = (int) (distance * Cosmo.COUNTS_PER_INCH);
        right_start = Cosmo.rightRear.getCurrentPosition();
        left_start = Cosmo.leftRear.getCurrentPosition();
        double lfpower;
        double lrpower;
        double rfpower;
        double rrpower;

        double lfbase;
        double lrbase;
        double rfbase;
        double rrbase;
        lfbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        lrbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rfbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rrbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        while (((abs(Cosmo.rightRear.getCurrentPosition() - right_start) + abs(Cosmo.leftRear.getCurrentPosition() - left_start)) / 2 < abs(moveCounts)) && opModeIsActive()  /* ENCODERS*/) {//Should we average all four motors?
            //Determine correction
            double correction = robot_orientation - getheading();
            if (correction <= -180) {
                correction += 360;
            } else if (correction >= 180) {                      // correction should be +/- 180 (to the left negative, right positive)
                correction -= 360;
            }
            lrpower = lrbase; //MIGHT BE MORE EFFECIENT TO COMBINE THESE WITHT HE ADJUSTMENT PART AND SET ADJUSTMENT TO ZERO IF NOT NEEDED
            lfpower = lfbase;
            rrpower = rrbase;
            rfpower = rfbase;
            if (abs(correction) > drive_THRESHOLD) {//If you are off
                //Apply power to one side of the robot to turn the robot back to the right heading
                double right_adjustment = Range.clip((drive_COEF * correction / 45), -1, 1);
                lrpower -= right_adjustment;
                lfpower -= right_adjustment;
                rrpower = rrbase + right_adjustment;
                rfpower = rfbase + right_adjustment;

            }//Otherwise you Are at the right orientation

            //Determine largest power being applied in either direction
            max = abs(lfpower);
            if (abs(lrpower) > max) max = abs(lrpower);
            if (abs(rfpower) > max) max = abs(rfpower);
            if (abs(rrpower) > max) max = abs(rrpower);

            multiplier = speed / max; //multiplier to adjust speeds of each wheel so you can have a max power of 1 on atleast 1 wheel

            lfpower *= multiplier;
            lrpower *= multiplier;
            rfpower *= multiplier;
            rrpower *= multiplier;

            Cosmo.leftFront.setPower(lfpower);
            Cosmo.leftRear.setPower(lrpower);
            Cosmo.rightFront.setPower(rfpower);
            Cosmo.rightRear.setPower(rrpower);

//            RobotLog.ii("[GromitIR] ", Double.toString(18.7754*Math.pow(sharpIRSensor.getVoltage(),-1.51)), Integer.toString(left_front.getCurrentPosition()));

        }
        //gromit.driveTrain.stopMotors();
        Cosmo.leftFront.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
    }


    //Turn using the IMU and meccanum drive
    public void mecanumTurn(double speed, double target_heading) {
        if (speed > 1) speed = 1.0;
        //else if(speed <= 0) speed = 0.1;

        double correction = target_heading - getheading();
        if (correction <= -180) {
            correction += 360;   // correction should be +/- 180 (to the left negative, right positive)
        } else if (correction >= 180) {
            correction -= 360;
        }

        while (abs(correction) >= Cosmo.turn_THRESHOLD && opModeIsActive()) { //opmode active?{
            correction = target_heading - getheading();
            if (abs(correction) <= Cosmo.turn_THRESHOLD) break;

            if (correction <= -180)
                correction += 360;   // correction should be +/- 180 (to the left negative, right positive)
            if (correction >= 180) correction -= 360;

            double adjustment = Range.clip((Math.signum(correction) * Cosmo.turn_MIN_SPEED + Cosmo.turn_COEF * correction / 100), -1, 1);  // adjustment is motor power: sign of correction *0.07 (base power)  + a proportional bit

            Cosmo.leftFront.setPower(-adjustment * speed);
            Cosmo.leftRear.setPower(-adjustment * speed);
            Cosmo.rightFront.setPower((adjustment * speed));
            Cosmo.rightRear.setPower((adjustment * speed));
        }
//        gromit.driveTrain.stopMotors();
        Cosmo.leftFront.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
    }


    public void mecanumDrivetoTape(double speed, double distance, double robot_orientation, double drive_direction) {
        double max;
        double multiplier;
        int right_start;
        int left_start;
        int moveCounts;
        //int drive_direction = -90;
        moveCounts = (int) (distance * Cosmo.COUNTS_PER_INCH);
        right_start = Cosmo.rightRear.getCurrentPosition();
        left_start = Cosmo.leftRear.getCurrentPosition();
        double lfpower;
        double lrpower;
        double rfpower;
        double rrpower;

        double lfbase;
        double lrbase;
        double rfbase;
        double rrbase;
        lfbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        lrbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rfbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rrbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        /** this is the main test to see location of color tape
         *  (hue is less than so much || hue is > so much)
         * **/
        Color.RGBToHSV((int) (Cosmo.sensorColor.red() * 255), (int) (Cosmo.sensorColor.green() * 255), (int) (Cosmo.sensorColor.blue() * 255), hsvValues);

        while (((abs(Cosmo.rightRear.getCurrentPosition() - right_start) + abs(Cosmo.leftRear.getCurrentPosition() - left_start)) / 2 < abs(moveCounts))
                && opModeIsActive() &&    // opmode has to be active
                (hsvValues[0] > grayRedBorder && hsvValues[0] < grayBlueBorder)) {         //  stop if the hue goes outside of the gray range
            //Determine correction
            double correction = robot_orientation - getheading();
            if (correction <= -180) {
                correction += 360;
            } else if (correction >= 180) {                      // correction should be +/- 180 (to the left negative, right positive)
                correction -= 360;
            }
            lrpower = lrbase; //MIGHT BE MORE EFFECIENT TO COMBINE THESE WITHT HE ADJUSTMENT PART AND SET ADJUSTMENT TO ZERO IF NOT NEEDED
            lfpower = lfbase;
            rrpower = rrbase;
            rfpower = rfbase;
            if (abs(correction) > drive_THRESHOLD) {//If you are off
                //Apply power to one side of the robot to turn the robot back to the right heading
                double right_adjustment = Range.clip((drive_COEF * correction / 45), -1, 1);
                lrpower -= right_adjustment;
                lfpower -= right_adjustment;
                rrpower = rrbase + right_adjustment;
                rfpower = rfbase + right_adjustment;

            }//Otherwise you Are at the right orientation

            //Determine largest power being applied in either direction
            max = abs(lfpower);
            if (abs(lrpower) > max) max = abs(lrpower);
            if (abs(rfpower) > max) max = abs(rfpower);
            if (abs(rrpower) > max) max = abs(rrpower);

            multiplier = speed / max; //multiplier to adjust speeds of each wheel so you can have a max power of 1 on atleast 1 wheel

            lfpower *= multiplier;
            lrpower *= multiplier;
            rfpower *= multiplier;
            rrpower *= multiplier;

            Cosmo.leftFront.setPower(lfpower);
            Cosmo.leftRear.setPower(lrpower);
            Cosmo.rightFront.setPower(rfpower);
            Cosmo.rightRear.setPower(rrpower);

//            RobotLog.ii("[GromitIR] ", Double.toString(18.7754*Math.pow(sharpIRSensor.getVoltage(),-1.51)), Integer.toString(left_front.getCurrentPosition()));
            // update the Hue
            Color.RGBToHSV((int) (Cosmo.sensorColor.red() * 255), (int) (Cosmo.sensorColor.green() * 255), (int) (Cosmo.sensorColor.blue() * 255), hsvValues);

        }
        //gromit.driveTrain.stopMotors();
        Cosmo.leftFront.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
    }

    public void mecanumDrivetoObject(double speed, double distance, double robot_orientation, double drive_direction, double distanceToObject) {
        double max;
        double multiplier;
        int right_start;
        int left_start;
        int moveCounts;
        //int drive_direction = -90;
        moveCounts = (int) (distance * Cosmo.COUNTS_PER_INCH);
        right_start = Cosmo.rightRear.getCurrentPosition();
        left_start = Cosmo.leftRear.getCurrentPosition();
        double lfpower;
        double lrpower;
        double rfpower;
        double rrpower;

        double lfbase;
        double lrbase;
        double rfbase;
        double rrbase;
        lfbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        lrbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rfbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rrbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        /** this is the main test to see if you've gone far enough,  add the color tape in here!
         *  so you need a || (hue is less than so much || hue is > so much)
         *
         *  could also say  'while it's not withing a little bit of the gray reading'
         *
         * **/

        while (((abs(Cosmo.rightRear.getCurrentPosition() - right_start) + abs(Cosmo.leftRear.getCurrentPosition() - left_start)) / 2 < abs(moveCounts))
                && opModeIsActive() &&    // opmode has to be active
                (Cosmo.sideSensor.getDistance(DistanceUnit.MM)> distanceToObject)) {         //  stop if close enough to block
            //Determine correction
            double correction = robot_orientation - getheading();
            if (correction <= -180) {
                correction += 360;
            } else if (correction >= 180) {                      // correction should be +/- 180 (to the left negative, right positive)
                correction -= 360;
            }
            lrpower = lrbase; //MIGHT BE MORE EFFECIENT TO COMBINE THESE WITHT HE ADJUSTMENT PART AND SET ADJUSTMENT TO ZERO IF NOT NEEDED
            lfpower = lfbase;
            rrpower = rrbase;
            rfpower = rfbase;
            if (abs(correction) > drive_THRESHOLD) {//If you are off
                //Apply power to one side of the robot to turn the robot back to the right heading
                double right_adjustment = Range.clip((drive_COEF * correction / 45), -1, 1);
                lrpower -= right_adjustment;
                lfpower -= right_adjustment;
                rrpower = rrbase + right_adjustment;
                rfpower = rfbase + right_adjustment;

            }//Otherwise you Are at the right orientation

            //Determine largest power being applied in either direction
            max = abs(lfpower);
            if (abs(lrpower) > max) max = abs(lrpower);
            if (abs(rfpower) > max) max = abs(rfpower);
            if (abs(rrpower) > max) max = abs(rrpower);

            multiplier = speed / max; //multiplier to adjust speeds of each wheel so you can have a max power of 1 on atleast 1 wheel

            lfpower *= multiplier;
            lrpower *= multiplier;
            rfpower *= multiplier;
            rrpower *= multiplier;

            Cosmo.leftFront.setPower(lfpower);
            Cosmo.leftRear.setPower(lrpower);
            Cosmo.rightFront.setPower(rfpower);
            Cosmo.rightRear.setPower(rrpower);


            telemetry.addData("side sensor", String.format("%.01f mm", Cosmo.sideSensor.getDistance(DistanceUnit.MM)));
            telemetry.update();


        }
        //gromit.driveTrain.stopMotors();
        Cosmo.leftFront.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
    }



    public float getheading() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles = Cosmo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; //For a -180 to 180 range
        //return (angles.firstAngle + 180 + 180)%360; // for a zero to 360 range
    }

    /*************************************************************************************************\
     |--------------------------------- Eli Edit Method ----------------------------------------------|
     \************************************************************************************************/

    public void editParameters() {

        String arrow0 = " ";
        String arrow1 = " ";
        String arrow2 = " ";

        boolean dpadPressedUp = false;
        boolean dpadPressedDown = false;
        boolean dpadPressedLeft = false;
        boolean dpadPressedRight = false;
        String[] position = {"foundation position", "skystone position"};
        int positionIndex = 0;
        if (skystonePosition) positionIndex = 1;
//
        String[] color = {"blue", "red"};
        int colorIndex = 1;
        if (teamIsRed == false) colorIndex = 0;

//        if (teamIsRed) colorIndex = 1;
//

//        if (testBot) botIndex = 1;

        int currentEdit = 1;

        while (!gamepad1.right_stick_button && !opModeIsActive() && !isStopRequested()) {   // while haven't presse exit button, not in play mode, and not in stop
            telemetry.addLine("===> Press Right Joystick to exit EDIT mode <===");
            // Send telemetry message to signify robot waiting;
            telemetry.addLine("");

            //telemetry.addLine().addData("", currentEdit).addData("current edit number", ' ');
            telemetry.addLine().addData(arrow0, waitTime1).addData("wait time", arrow0);
            telemetry.addLine().addData(arrow1, colorIndex).addData(color[colorIndex], arrow1);
            telemetry.addLine().addData(arrow2, positionIndex).addData(position[positionIndex], arrow2);


//            telemetry.addLine().addData(arrow4, "Color       ");
            telemetry.update();

            if (gamepad1.dpad_down) {
                dpadPressedDown = true;
            } else if (gamepad1.dpad_down == false && dpadPressedDown) {
                dpadPressedDown = false;
                currentEdit += 1;
                if (currentEdit > 2) {
                    currentEdit = 0;
                }
            }

            if (gamepad1.dpad_up) {
                dpadPressedUp = true;
            } else if (gamepad1.dpad_up == false && dpadPressedUp) {
                dpadPressedUp = false;
                currentEdit -= 1;
                if (currentEdit < 0) {
                    currentEdit = 2;
                }
            }

            if (currentEdit == 0) {
                arrow0 = "<>";
            } else {
                arrow0 = "    ";
            }
            if (currentEdit == 1) {
                arrow1 = "<>";
            } else {
                arrow1 = "    ";
            }
            if (currentEdit == 2) {
                arrow2 = "<>";
            } else {
                arrow2 = "    ";
            }


            if (gamepad1.dpad_left) {
                dpadPressedLeft = true;
            } else if (gamepad1.dpad_left == false && dpadPressedLeft) {
                dpadPressedLeft = false;
                if (currentEdit == 0) {
                    waitTime1 -= 1000;
                }
                if (currentEdit == 1) {
                    if (colorIndex == 1) {
                        colorIndex = 0;
                        teamIsRed = false;
                        //Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                    } else {
                        colorIndex = 1;
                        teamIsRed = true;
                        //Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                    }
                }
                if (currentEdit == 2) {
                    if (positionIndex == 1) {
                        positionIndex = 0;
                        skystonePosition = false;

                    } else {
                        positionIndex = 1;
                        skystonePosition = true;

                    }
                }


            }

            if (gamepad1.dpad_right) {
                dpadPressedRight = true;
            } else if (gamepad1.dpad_right == false && dpadPressedRight) {
                dpadPressedRight = false;

                if (currentEdit == 0) {
                    waitTime1 += 1000;
                }
                if (currentEdit == 1) {
                    if (colorIndex == 1) {
                        colorIndex = 0;
                        teamIsRed = false;
                        //Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                    } else {
                        colorIndex = 1;
                        teamIsRed = true;
                        // Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                    }
                }
                if (currentEdit == 2) {
                    if (positionIndex == 1) {
                        positionIndex = 0;
                        skystonePosition = false;

                    } else {
                        positionIndex = 1;
                        skystonePosition = true;

                    }
                }


            }
//                if (gamepad1.y) {
//                    break;
//                }
        }
        telemetry.update();
    }

}