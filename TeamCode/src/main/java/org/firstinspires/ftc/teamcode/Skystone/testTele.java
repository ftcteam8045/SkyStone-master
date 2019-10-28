package org.firstinspires.ftc.teamcode.Skystone;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "testTele", group = "8045")  // @Autonomous(...) is the other common choice
@Disabled
public class testTele extends OpMode {

    testHardware Cosmo;
    private ElapsedTime runtime = new ElapsedTime();
    double timeLeft;
    // variables used during the configuration process



    //    double turnDirection;
    public double topSpeed = 0.5 ;


    //Back mode
    public boolean frontIsForward = false;
    public boolean rightbtnIsReleased = true;

    //Drive type

    public boolean run = false;

    //    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorDistance;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    public float values[] = hsvValues;






    public double multiplier = 0.1818;



    @Override
    public void init() {
        Cosmo = new testHardware();
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

//        // BACK mode

        if (gamepad1.right_stick_button) {
            if (rightbtnIsReleased) {
                rightbtnIsReleased = false;
                frontIsForward = !frontIsForward;
            }
        } else {
            rightbtnIsReleased = true;
        }

//        if(gamepad1.dpad_up){
//            Cosmo.fServoR.setPosition(0.8);
//            Cosmo.fServoL.setPosition(0.2);
//
//        }
//
//        if(gamepad1.dpad_down){
//            Cosmo.fServoR.setPosition(0.2);
//            Cosmo.fServoL.setPosition(0.8);
//
//        }

        /**  set lights color  **/
        /** 2M Distance Sensor Code  */

//        Color.RGBToHSV((int)(Cosmo.sensorColor.red() * 255), (int)(Cosmo.sensorColor.green() * 255), (int)(Cosmo.sensorColor.blue() * 255), hsvValues);
//        if (hsvValues[0] > grayRedBorder && hsvValues[0] < grayBlueBorder ) {
//            Cosmo.LEDDriver.setPattern(teamColor);
//        } else {
//            Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//        }


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

//        drivesmart(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);

        //telemetry.addData("range", String.format("%.01f mm", Cosmo.sensor.getDistance(DistanceUnit.MM)));


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


//    public void drivesmart(double x, double y, double turn) {
//
//        if (frontIsForward) {             // driving with the front facing forward
//
//        } else {                            // driving with the rear facing forward
//            y = -y;
//            x = -x;
//        }
//
//        double lfpower;
//        double lrpower;
//        double rfpower;
//        double rrpower;
//
//        double rotation = turn;  // knock down the turn power -- NOT ANYMORE
//
//        //Determine largest power being applied in either direction
//
//        lfpower = ( y - x + rotation);
//        lrpower = ( y + x + rotation);
//        rfpower = ( y + x - rotation);
//        rrpower = ( y - x - rotation);
//
//        lfpower = lfpower * topSpeed;
//        lrpower = lrpower * topSpeed;
//        rfpower = rfpower * topSpeed;
//        rrpower = rrpower * topSpeed;
//
//       // Cosmo.leftFront.setPower(lfpower);
//        Cosmo.leftRear.setPower(lrpower);
//       // Cosmo.rightFront.setPower(rfpower);
//        Cosmo.rightRear.setPower(rrpower);
//
//    }
////
//
//
//    public void stop() {
//        //Cosmo.leftFront.setPower(0.0);
//        Cosmo.leftRear.setPower(0.0);
//       // Cosmo.rightFront.setPower(0.0);
//        Cosmo.rightRear.setPower(0.0);
//    }




}
