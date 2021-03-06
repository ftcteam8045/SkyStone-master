package org.firstinspires.ftc.teamcode.Skystone;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;



@TeleOp(name = "MainTele", group = "8045")  // @Autonomous(...) is the other common choice
//@Disabled
public class mainTele extends OpMode {

    Hardware2019 Cosmo;
    private ElapsedTime runtime = new ElapsedTime();
    double timeLeft;
    // variables used during the configuration process


  //  public RevBlinkinLedDriver.BlinkinPattern teamColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;



//    double turnDirection;
    public double topSpeed = 0.5 ;


    //Back mode
    public boolean frontIsForward = false;
    public boolean rightbtnIsReleased = true;
    public boolean beginTrack = false;
    public boolean intakeOnF = false;
    public boolean intakeOnB = false;
    public boolean goIntakeF = false;
    public boolean goIntakeB = false;
    public boolean startTimer = true;

    //Drive type

    public boolean run = false;

//    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorDistance;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    public float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public double multiplier = 0.1818;


    @Override
    public void init() {
        Cosmo = new Hardware2019();
        Cosmo.init(hardwareMap);


        telemetry.addData("Status", "Initializing");
        telemetry.update();




        telemetry.addData("Finished", "Initializing");
        telemetry.update();


    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        timeLeft = 120 - runtime.seconds();


//        Color.RGBToHSV((int) (Cosmo.sensorColor.red() * SCALE_FACTOR),
//                (int) (Cosmo.sensorColor.green() * SCALE_FACTOR),
//                (int) (Cosmo.sensorColor.blue() * SCALE_FACTOR),
//                hsvValues);
//
//        // send the info back to driver station using telemetry function.
//        telemetry.addData("Alpha", Cosmo.sensorColor.alpha());
//        telemetry.addData("Red  ", Cosmo.sensorColor.red());
//        telemetry.addData("Green", Cosmo.sensorColor.green());
//        telemetry.addData("Blue ", Cosmo.sensorColor.blue());
//        telemetry.addData("Hue", hsvValues[0]);

//        // BACK mode

        if (gamepad1.right_stick_button) {
            if (rightbtnIsReleased) {
                rightbtnIsReleased = false;
                frontIsForward = !frontIsForward;
            }
        } else {
            rightbtnIsReleased = true;
        }

        /**  set lights color  **/
        /** 2M Distance Sensor Code  */




//        Color.RGBToHSV((int)(Cosmo.sensorColor.red() * 255), (int)(Cosmo.sensorColor.green() * 255), (int)(Cosmo.sensorColor.blue() * 255), hsvValues);
//        if (hsvValues[0] > grayRedBorder && hsvValues[0] < grayBlueBorder ) {
//            Cosmo.LEDDriver.setPattern(teamColor);
//        } else {
//            Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//        }

        telemetry.addData("Front Distance MM - ", Cosmo.frontSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Back Distance MM - ", Cosmo.backSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Clamp Side Distance MM - ", Cosmo.sideSensor.getDistance(DistanceUnit.MM));
        double irvoltagetest = Cosmo.batterySensor.getVoltage();
        double irdistancetest = -7.411 * Math.pow(irvoltagetest, 3) + 52.356 * Math.pow(irvoltagetest, 2) - 125.222 * irvoltagetest + 111.659;
        telemetry.addData("", "Sharp volts, Distance(cm): %4.2f  %4.1f", irvoltagetest, irdistancetest);
        double irvoltagetest2 = Cosmo.sharp2.getVoltage();
        double irdistancetest2 = -7.411 * Math.pow(irvoltagetest2, 3) + 52.356 * Math.pow(irvoltagetest2, 2) - 125.222 * irvoltagetest2 + 111.659;
        telemetry.addData("", "Sharp2 volts, Distance(cm): %4.2f  %4.1f", irvoltagetest2, irdistancetest2);
        /**  set drive speed  **/

        if (gamepad1.left_bumper) {
            topSpeed = 1.0;
        } else if (gamepad1.left_trigger > 0.1) {
            topSpeed = 0.3;
        }
        else {
            topSpeed = 0.6;
        }
//        /** set driving colors **/
//        if (topSpeed == 0.4) {
//            Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
//        } else {
//            Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        }
        /** DRIVE  HERE   */

        drivesmart(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);



        if(gamepad1.dpad_right) {
            Cosmo.clawBot.setPosition(0.80);
            Cosmo.clawMid.setPosition(0.85);
            sleep(200);
            Cosmo.clawTop.setPosition(0.25);
        }

        if(gamepad1.x) { //ready to clamp
            Cosmo.clawBot.setPosition(0.36);
            Cosmo.clawMid.setPosition(0.0);
            Cosmo.clawTop.setPosition(0.98);
        }
        if(gamepad1.start == false && gamepad2.start == false && gamepad1.a) { //clamp block
            Cosmo.clawBot.setPosition(0.275);
            sleep(200);
            Cosmo.clawTop.setPosition(0.59);
            sleep(100);
            Cosmo.clawMid.setPosition(0.205);
        }
        if(gamepad1.y) {  //lift block while clamping
            Cosmo.clawBot.setPosition(0.78);
            Cosmo.clawMid.setPosition(0.205);
            Cosmo.clawTop.setPosition(0.59);

        }
        if(gamepad1.start == false && gamepad2.start == false && gamepad1.b) { //drop block while lifted up
            Cosmo.clawBot.setPosition(0.74);
            Cosmo.clawMid.setPosition(0.15);
            Cosmo.clawTop.setPosition(0.98);

        }


        /** CONTROLS FOR FOUNDATION CLAMP SERVOS **/

        if(gamepad1.dpad_up == true){  //up

            Cosmo.clamp1.setPosition(0.76);
            Cosmo.clamp2.setPosition(0.32);

        }
        if(gamepad1.dpad_down == true){  //down

            Cosmo.clamp1.setPosition(0.18);
            Cosmo.clamp2.setPosition(0.89);

        }
        /** INTAKE CONTROLS IN TELE **/

        if(gamepad1.right_trigger == 0.0){

            goIntakeF = true;
        }

        if(!gamepad1.right_bumper){
            goIntakeB = true;
        }


        if(gamepad1.right_trigger > 0.1 && goIntakeF){
            intakeOnF = true;
        }
        if(gamepad1.right_bumper && goIntakeB){
            intakeOnB = true;
        }

        if(intakeOnB) {
            if (Cosmo.leftIntake.getPower() == 0.0) {

                Cosmo.leftIntake.setPower(-0.4);
                Cosmo.rightIntake.setPower(-0.4);
                intakeOnB = false;
                goIntakeB = false;

            } else {

                Cosmo.leftIntake.setPower(0.0);
                Cosmo.rightIntake.setPower(0.0);
                intakeOnB = false;
                goIntakeB = false;


            }
        }


        if(intakeOnF) {
            if (Cosmo.leftIntake.getPower() == 0.0) {

                Cosmo.leftIntake.setPower(1.0);
                Cosmo.rightIntake.setPower(1.0);
                intakeOnF = false;
                goIntakeF = false;

            } else {

                Cosmo.leftIntake.setPower(0.0);
                Cosmo.rightIntake.setPower(0.0);
                intakeOnF = false;
                goIntakeF = false;


            }
        }

        /** LIFT MOTOR CONTROLS FOR DRIVER 2 **/


        if(Math.abs(gamepad2.left_stick_y) > 0.04){  // LIFT AND LOWER MOTOR

            Cosmo.leftLift.setPower(gamepad2.left_stick_y);
        } else {
            Cosmo.leftLift.setPower(0);

        }

        if(Math.abs(gamepad2.right_stick_y) > 0.04){  // LIFT AND LOWER MOTOR

            Cosmo.rightLift.setPower(gamepad2.right_stick_y);
        } else {
            Cosmo.rightLift.setPower(0);

        }

        /** ARM CONTROLS FOR DRIVER 2 **/

        if(gamepad1.start == false && gamepad2.start == false && gamepad2.b){   // OPEN ARM CLAMP

            Cosmo.armGrabber.setPosition(1);

        }
        if(gamepad1.start == false && gamepad2.start == false && gamepad2.a){   // CLOSE ARM CLAMP

            Cosmo.armGrabber.setPosition(0);

        }




        /** DISTANCE SENSOR FOUNDATION TRACKING SOFTWARE **/

//        if(gamepad1.x){
//            if(Cosmo.dis1.getDistance(DistanceUnit.MM) < 250 || Cosmo.dis2.getDistance(DistanceUnit.MM) < 250) {
//                beginTrack = true;
//            }
//
//            if(beginTrack){
////            while(Cosmo.dis1.getDistance(DistanceUnit.MM) > 40 || Cosmo.dis2.getDistance(DistanceUnit.MM) > 40) {
////                Cosmo.leftFront.setPower(0.2);
////                Cosmo.leftRear.setPower(0.2);
////                Cosmo.rightFront.setPower(0.2);
////                Cosmo.rightRear.setPower(0.2);
////            }
//
//
//                if(Cosmo.dis1.getDistance(DistanceUnit.MM) > Cosmo.dis2.getDistance(DistanceUnit.MM)){
//
//
//                    while(Math.abs(Cosmo.dis1.getDistance(DistanceUnit.MM) - Cosmo.dis2.getDistance(DistanceUnit.MM)) > 5) {
//                        Cosmo.leftFront.setPower(0.2);
//                        Cosmo.leftRear.setPower(0.2);
//                        Cosmo.rightFront.setPower(-0.2);
//                        Cosmo.rightRear.setPower(-0.2);
//                    }
//
//                }
//
//                if(Cosmo.dis1.getDistance(DistanceUnit.MM) < Cosmo.dis2.getDistance(DistanceUnit.MM)){
//
//
//                    while(Math.abs(Cosmo.dis1.getDistance(DistanceUnit.MM) - Cosmo.dis2.getDistance(DistanceUnit.MM)) > 5) {
//                        Cosmo.leftFront.setPower(-0.2);
//                        Cosmo.leftRear.setPower(-0.2);
//                        Cosmo.rightFront.setPower(0.2);
//                        Cosmo.rightRear.setPower(0.2);
//                    }
//
//                }
//
//                    beginTrack = false;
//
//
//            }
//
//
//
//        }




        telemetry.update();
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * This method puts the current thread to sleep for the given time in msec.
     * It handles InterruptException where it recalculates the remaining time
     * and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param sleepTime specifies sleep time in msec.
     */
    public static void sleep(long sleepTime) {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }   //sleep


    public void drivesmart(double x, double y, double turn) {

        if (frontIsForward) {             // driving with the front facing forward

        } else {                            // driving with the rear facing forward
            y = -y;
            x = -x;
        }

        double lfpower;
        double lrpower;
        double rfpower;
        double rrpower;

        double rotation = turn;  // knock down the turn power -- NOT ANYMORE

        //Determine largest power being applied in either direction

        lfpower = ( y - x + rotation);
        lrpower = ( y + x + rotation);
        rfpower = ( y + x - rotation);
        rrpower = ( y - x - rotation);

        lfpower = lfpower * topSpeed;
        lrpower = lrpower * topSpeed;
        rfpower = rfpower * topSpeed;
        rrpower = rrpower * topSpeed;

        Cosmo.leftFront.setPower(lfpower);
        Cosmo.leftRear.setPower(lrpower);
        Cosmo.rightFront.setPower(rfpower);
        Cosmo.rightRear.setPower(rrpower);

    }
//


    public void stop() {
        Cosmo.leftFront.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
    }




}
