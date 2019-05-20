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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Depot Auto Far Crater TensorFlow", group="7140")
@Disabled
public class Auto71404 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private DcMotor elevator = null;
    private Servo dumper = null;
    private Servo marker = null;
    public static final double MID_SERVO       =  0.5 ;
    public int pos = -1;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AdDTrBn/////AAABmb5FFrkrC0jqje1sqADLtJZBfquYYjCZ2e0HzCeq8wYoa0UyWsqECH2+Kl2Pv3ctvo02w7Q9WrSSMFXxt/ZWni5qw+Nls/kaW9DaCEfQGEauDiqa638WmZaJZRZk+Tab+p8ix2QedBrQA8s/mbwIF1nvUYyFjM/2hFvdy01trf3H/z0Ixy6Z52RQHkrE+f7QAsawi0OEDDus2Bpnhj3dwC4J5etS1FSVTLomJ+XDYsa16CMHWZYMhmy5N8FlcNYhr0i6haQOqldVYYh/XOShwWWoOO09le4D3Sq5hkZAoqWFgXW28d0CIzMG+Dk1ud5acYg3fFVYYTr+t/yQdbjhF1YCR5URlBenREFmbDZWgvA4";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;



    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        dumper = hardwareMap.get(Servo.class, "dumper");
        marker = hardwareMap.get(Servo.class, "marker");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dumper.setPosition(dumper.MAX_POSITION);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          leftDrive.getCurrentPosition(),
                          rightDrive.getCurrentPosition());
        telemetry.update();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                //leftParticle();
                                pos = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                pos = 3;
                                //rightParticle();
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                //centerParticle();
                                pos = 2;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        if (tfod != null) {
            tfod.shutdown();
        }

        if (pos == 1) {
            //leftParticle();
        }
        else if (pos == 2) {
            //centerParticle();
        }
        else if (pos == 3) {
            //rightParticle();
        }
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
/*        encoderDrive(DRIVE_SPEED,  -60,  -60, 5.0);  // S1: Forward 54 Inches with 5 Sec timeout
        marker.setPosition(marker.MIN_POSITION);
        sleep(1000);
        marker.setPosition(marker.MAX_POSITION);
        sleep(1000);
        encoderDrive(TURN_SPEED,   -12, 12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 75, 75, 10.0);  // S3: Reverse 75 Inches with 10 Sec timeout
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
*/
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            leftDrive.getCurrentPosition(),
                                            rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    //THIS ONE IS FINISHED
    private void centerParticle() {
        encoderDrive(DRIVE_SPEED,  -60,  -60, 5.0);  // S1: Forward 54 Inches with 5 Sec timeout
        dump();
        encoderDrive(TURN_SPEED,   -12, 12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 75, 75, 10.0);  // S3: Reverse 75 Inches with 10 Sec timeout
        sleep(1000);
    }

    private void rightParticle() {
        encoderDrive(DRIVE_SPEED,  -12,  -12, 4.0);  // S1: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   -20, 20, 4.0);  // S2: Turn Right 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -12, -12, 4.0);  // S3: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   20, -20, 4.0);  // S4: Turn Left 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -12, -12, 4.0);  // S5: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   20, -20, 4.0);  // S6: Turn Left 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -12, -12, 4.0);  // S7: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   -20, 20, 4.0);  // S8: Turn Right 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -40, -40, 4.0);  // S9: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        dump();
        encoderDrive(TURN_SPEED,   -12, 12, 4.0);  // S10: Turn Right 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED, 75, 75, 10.0);  // S11: Reverse 75 Inches with 10 Sec timeout
        sleep(250);
    }

    private void leftParticle() {
        encoderDrive(DRIVE_SPEED,  -12,  -12, 4.0);  // S1: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   20, -20, 4.0);  // S2: Turn Left 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -12, -12, 4.0);  // S3: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   -20, 20, 4.0);  // S4: Turn Right 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -12, -12, 4.0);  // S5: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   -20, 20, 4.0);  // S6: Turn Right 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -12, -12, 4.0);  // S7: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(TURN_SPEED,   20, -20, 4.0);  // S8: Turn Left 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED,   -40, -40, 4.0);  // S9: Forward 12 Inches with 4 Sec timeout
        sleep(250);
        dump();
        encoderDrive(TURN_SPEED,   -12, 12, 4.0);  // S10: Turn Right 20 Inches with 4 Sec timeout
        sleep(250);
        encoderDrive(DRIVE_SPEED, 75, 75, 10.0);  // S11: Reverse 75 Inches with 10 Sec timeout
        sleep(250);
    }

    private void dump() {
        marker.setPosition(marker.MIN_POSITION);
        sleep(500);
        marker.setPosition(marker.MAX_POSITION);
    }

}
