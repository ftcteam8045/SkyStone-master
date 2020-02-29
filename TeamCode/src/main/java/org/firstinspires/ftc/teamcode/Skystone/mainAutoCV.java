package org.firstinspires.ftc.teamcode.Skystone;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.oldcode.AutoTransitioner;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_COEF;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_THRESHOLD;

/**
 * Created by Tristan Barrett  on 9/13/2019.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "mainAutoCV", group="Sky autonomous")
//@Disabled//comment out this line before using
public class mainAutoCV extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2019 Cosmo = new Hardware2019();   // Use a Pushbot's hardware

    /**
     * Menu Parameter Initialization
     **/
    public int waitTime1 = 0;
    public boolean skystonePosition = true;
    public boolean teamIsRed = true;
    public boolean left = false;
    public boolean center = false;
    public boolean right = true;
    public boolean correction1 = false;
    public boolean correction2 = false;
    public boolean over = false;
    public boolean under = false;
    public double grayHueValue = 90.0;  // color sensor values
    public double redHueValue = 5;
    public double s = 0.72727;
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



    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    static float offsetX = 1.65f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    static float offsetY = 0.2f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode(){

        /*
         * Initialize the drive system variables.the Robot
         * The init() method of the hardware class does all the work here
         */
        Cosmo.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        runtime.reset();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        AutoTransitioner.transitionOnStop(this, "MainTele");   // get ready for teleop at the end of auto

        /**************************************************************
         // Actual Init  loop
         *************************************************************/
        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addLine("Main Auto");


            telemetry.addLine(" Press Left Joystick for Edit");
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            if(valLeft == 0){
                left = true;
                center = false;
                right = false;
            }
            if(valMid == 0){
                left = false;
                center = true;
                right = false;
            }
            if(valRight == 0){
                left = false;
                center = false;
                right = true;
            }
            if (left) telemetry.addLine("LEFT");
            if (center) telemetry.addLine("CENTER");
            if (right) telemetry.addLine("RIGHT");

            if (teamIsRed) { /** RED SIDE OBJECT DETECTION **/
                telemetry.addData("", "RED");
                offsetX = 1.5f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
                offsetY = 0.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
            } else {  /** BLUE SIDE OBJECT DETECTION **/
                telemetry.addData("", "BLUE");
                offsetX = 1.6f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
                offsetY = 0.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
            }
            if (skystonePosition) {
                telemetry.addData("", "Skystone Position");
            } else {
                telemetry.addData("", "Foundation Position");
            }




            telemetry.update();

            if (gamepad1.back || gamepad1.left_stick_button) {             // edit parameters  & write the new file

                editParameters();

            }

        }

        if (teamIsRed) { /** RED SIDE CODE **/

            if (skystonePosition) {            /** skystone zone drive  **/

                /** scans for skystone first **/
                Cosmo.clamp1.setPosition(0.76);
                Cosmo.clamp2.setPosition(0.32);
                mecanumDrive(0.6, -15*s, 0, 0); //drive forward
                grabSkystone();
                deliverBlock(30, 1);
                mecanumDrive(0.9, -18, 90, 0);
                mecanumDrivetoTape(0.3,  -10, 90, 0);
                grabSkystoneAgain();
                if(!left) {
                    deliverBlock(42.6, 1);
                } else  {
                    mecanumDrive(0.9, 35 * s, 90, 0);
                }
                mecanumTurn(1.0, 180); // turn right again
                Cosmo.clamp1.setPosition(0.296);
                Cosmo.clamp2.setPosition(0.605);
                if(!left) {
                    mecanumDrive(0.8, -14 * s, 180, -90); //strafe over
                    mecanumDrive(0.6, 2 * s, 180, 0); //drive forward
                } else {
                    mecanumDrive(0.6, 4.5 * s, 180, 0); //drive forward
                }
                mecanumDrive(0.25, 7.4*s, 180, 0); //drive forward
                Cosmo.clamp1.setPosition(0.18);
                Cosmo.clamp2.setPosition(0.89);
                sleep(400);
                mecanumDrive(0.8, -16.8*s, 145, -12); //drive back from foundation
                mecanumDrive(0.9, -14*s, 90, 0); //drive back from foundation
                if(left){
                    Cosmo.leftIntake.setPower(-0.4);
                    Cosmo.rightIntake.setPower(-0.4);
                }
                mecanumDrive(0.6, 17*s, 90, 0); //place foundation
                if(left){
                    Cosmo.leftIntake.setPower(0.0);
                    Cosmo.rightIntake.setPower(0.0);
                }
                Cosmo.clamp1.setPosition(0.76);
                Cosmo.clamp2.setPosition(0.32);
                mecanumDrive(0.8, 16.5*s, 90, 90); //strafe over
                mecanumDrive(1.0, -28*s, 91, 0);  //drive until tape is detected


            } else {             /** foundation zone drive  **/


                mecanumDrivetoTape(0.6, 12 * s, 0, 0);


            }
        } else {     /** BLUE SIDE CODE **/

            if (skystonePosition) {            /** skystone zone drive  **/

                /** scans for skystone first **/
                Cosmo.clamp1.setPosition(0.76);
                Cosmo.clamp2.setPosition(0.32);
                mecanumDrive(0.6, -15*s, 0, 0); //drive forward
                grabSkystone();
                deliverBlock(-23, 1);
                mecanumDrive(0.9, 12.6, 90, 0);
                mecanumDrivetoTape(0.3,  10, 90, 0);
                grabSkystoneAgain();
                if(!right) {
                    deliverBlock(-33.85, 1);
                } else  {
                    mecanumDrive(0.9, 37 * s, -90, 0);
                }
                mecanumTurn(1.0, -180); // turn right again
                mecanumDrive(0.8, 5*s, -180, -90); //strafe over
                Cosmo.clamp1.setPosition(0.296);
                Cosmo.clamp2.setPosition(0.605);
                mecanumDrive(0.6, 3*s, -180, 0); //drive forward
                mecanumDrive(0.25, 2.5*s, -180, 0); //drive forward
                Cosmo.clamp1.setPosition(0.18);
                Cosmo.clamp2.setPosition(0.89);
                sleep(400);
                mecanumDrive(0.8, -10*s, -150, 0); //drive back from foundation
                mecanumDrive(0.7, -10*s, -130, 0); //drive back from foundation
                mecanumDrive(0.9, -14*s, -90, 0); //drive back from foundation
                if(right){
                    Cosmo.leftIntake.setPower(-0.4);
                    Cosmo.rightIntake.setPower(-0.4);
                }
                mecanumDrive(0.6, 12*s, -90, 0); //place foundation
                if(right){
                    Cosmo.leftIntake.setPower(0.0);
                    Cosmo.rightIntake.setPower(0.0);
                }
                Cosmo.clamp1.setPosition(0.76);
                Cosmo.clamp2.setPosition(0.32);
                mecanumDrive(0.8, 8*s, -90, -90); //strafe over
                mecanumDrive(1.0, -29*s, -91, 0);  //drive until tape is detected


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


    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

    public void grabSkystone() {


        if (teamIsRed) {  //TEAM IS RED
            if (left) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, -1.5 * s, 90, 0);
                mecanumDrivetoObject(0.4, -9 * s, 90, -90, 124);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, 19 * s, 90, 0);
                mecanumDrivetoTape(0.3, 6 * s, 90, 0);

            }

            if (center) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, -12.2 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, 31.8 * s, 90, 0);
                mecanumDrivetoTape(0.3, 12 * s, 90, 0);
            }

            if (right) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, -6 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                beforeClampCorrection1(0.2, 15, 90, 0, 355);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, 25 * s, 90, 0);
                mecanumDrivetoTape(0.3, 12 * s, 90, 0);

            }

        } else { //TEAM IS BLUE
            if (left) {  // GRAB FIRST SKYSTONE
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, 10.3 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                beforeClampCorrection1(0.2, -15, 90, 0, 270);  //225 - 275
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, -37 * s, 90, 0);
                mecanumDrivetoTape(0.3, -12 * s, 90, 0);
            }

            if (center) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, 16.1 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, -42 * s, 90, 0);
                mecanumDrivetoTape(0.3, -12 * s, 90, 0);

            }

            if (right) {
                mecanumTurn(1.0, 90); // turn right
                readyToGrab();
                mecanumDrive(0.75, 4.9 * s, 90, 0);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 124);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, -30 * s, 90, 0);
                mecanumDrivetoTape(0.3, -8 * s, 90, 0);

            }
        }

    }


    public void grabSkystoneAgain() {


        if (teamIsRed) {  //TEAM IS RED
            if (left) {
                mecanumDrive(0.8, -27 * s, 90, 0);
                sleep(200);
                mecanumDrive(0.7, 22*s, 90, 90);
                sleep(200);
                Cosmo.leftIntake.setPower(0.8);
                Cosmo.rightIntake.setPower(0.8);
                mecanumDrive(0.35, -3.8*s, 90, 0);
                Cosmo.leftIntake.setPower(0.0);
                Cosmo.rightIntake.setPower(0.0);
                mecanumDrive(0.45, 4.3*s, 90, 0);
                mecanumDrive(0.7, -20*s, 90, 90);
                mecanumDrive(0.9, 22 * s, 90, 0);
                mecanumDrivetoTape(0.3, 8 * s, 90, 0);

            }

            if (center) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, -15.6 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, 13.5 * s, 89, 0);
                mecanumDrivetoTape(0.3, 10 * s, 90, 0);
            }

            if (right) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, -10.4 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                beforeClampCorrection1(0.2, -15, 90, -90, 900);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, 10 * s, 90, 0);
                mecanumDrivetoTape(0.3, 10 * s, 90, 0);

            }

        } else { //TEAM IS BLUE
            if (left) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, 21.8 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                beforeClampCorrection1(0.2, -15, 90, 0, 825);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, -20 * s, 90, 0);
                mecanumDrivetoTape(0.3, -10 * s, 90, 0);


            }

            if (center) {
                Cosmo.clawMid.setPosition(0.0);
                mecanumDrive(0.8, 27 * s, 90, 0);
                Cosmo.clawTop.setPosition(0.98);  //ready to clamp
                sleep(200);
                Cosmo.clawBot.setPosition(0.36);
                sleep(250);
                mecanumDrivetoObject(0.4, -10 * s, 90, -90, 130);
                grabBlockWithClaw();
                mecanumDrive(0.8, 10 * s, 90, -90);
                mecanumDrive(0.9, -24 * s, 90, 0);
                mecanumDrivetoTape(0.3, -8 * s, 90, 0);
            }

            if (right) {
                mecanumDrive(0.8, 30 * s, 90, 0);
                mecanumTurn(1.0, -88); // turn right
                mecanumDrive(0.8, -2.3 * s, -90, 0);
                mecanumDrive(0.7, -16.2*s, -90, 90);
                Cosmo.leftIntake.setPower(0.8);
                Cosmo.rightIntake.setPower(0.8);
                mecanumDrive(0.35, -3.92*s, -90, 0);
                Cosmo.leftIntake.setPower(0.0);
                Cosmo.rightIntake.setPower(0.0);
                mecanumDrive(0.45, 4.35*s, -90, 0);
                mecanumDrive(0.7, 18*s, -90, 90);
                mecanumDrive(0.9, 21.8 * s, -90, 0);
                mecanumDrivetoTape(0.3, 15 * s, -90, 0);
            }
        }

    }


    public void readyToGrab(){
        Cosmo.clawBot.setPosition(0.50);
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

        if(teamIsRed){
            mecanumDrivetoObject(0.45, -13 * s, 90, -90, 124);

        }else {
            mecanumDrivetoObject(0.45, -10 * s, 90, -90, 124);

        }
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
        if(teamIsRed){
            mecanumDrive(0.8, 10.5 * s, 90, -90);
        } else {
            mecanumDrive(0.8, 7.5 * s, 90, -90);
        }
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


    public void beforeClampCorrection1(double speed, double distance, double robot_orientation, double drive_direction, double distanceToObject) {
        if(Cosmo.backSensor.getDistance(DistanceUnit.MM) > distanceToObject) {
            over = true;
            telemetry.addLine("OVER");
            distance = -distance;
        } else if (Cosmo.backSensor.getDistance(DistanceUnit.MM) < distanceToObject){
            under = true;
            telemetry.addLine("UNDER");
        }
        telemetry.update();
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
         // opmode has to be active


            if(teamIsRed) { // TEAM IS RED
                /* IF ROBOT IS TOO FAR FROM WALL - OVER DISTANCETOOBJECT*/

                while (opModeIsActive() && Cosmo.frontSensor.getDistance(DistanceUnit.MM) > distanceToObject && over) {         //  stop if close enough to block
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

                    telemetry.addData("Back Distance MM - ", Cosmo.frontSensor.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
                if(over){
                    Cosmo.leftFront.setPower(-lfbase);
                    Cosmo.leftRear.setPower(-lrbase);
                    Cosmo.rightFront.setPower(-rfbase);
                    Cosmo.rightRear.setPower(-rrbase);
                }
                over = false;

                /* IF ROBOT IS TOO CLOSE TO WALL - UNDER DISTANCETOOBJECT*/

                while (opModeIsActive() && Cosmo.frontSensor.getDistance(DistanceUnit.MM) < distanceToObject  && under) {         //  stop if close enough to block
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


                }
                if(under){
                    Cosmo.leftFront.setPower(-lfbase);
                    Cosmo.leftRear.setPower(-lrbase);
                    Cosmo.rightFront.setPower(-rfbase);
                    Cosmo.rightRear.setPower(-rrbase);
                }
                under = false;


            } else {// TEAM IS BLUE

                /* IF ROBOT IS TOO FAR FROM WALL - OVER DISTANCETOOBJECT*/

        while (opModeIsActive() && Cosmo.backSensor.getDistance(DistanceUnit.MM) > distanceToObject && over) {         //  stop if close enough to block
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

            telemetry.addData("Back Distance MM - ", Cosmo.backSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        if(over){
            Cosmo.leftFront.setPower(-lfbase);
            Cosmo.leftRear.setPower(-lrbase);
            Cosmo.rightFront.setPower(-rfbase);
            Cosmo.rightRear.setPower(-rrbase);
        }
        over = false;

        /* IF ROBOT IS TOO CLOSE TO WALL - UNDER DISTANCETOOBJECT*/

        while (opModeIsActive() && Cosmo.backSensor.getDistance(DistanceUnit.MM) < distanceToObject  && under) {         //  stop if close enough to block
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


        }
        if(under){
            Cosmo.leftFront.setPower(-lfbase);
            Cosmo.leftRear.setPower(-lrbase);
            Cosmo.rightFront.setPower(-rfbase);
            Cosmo.rightRear.setPower(-rrbase);
        }
        under = false;

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