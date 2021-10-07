/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Odometry & Tensorflow", group = "Concept")
@Disabled
public class OdometrywithTensorFlow<TICKS_TO_INCHES> extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    Motor frontLeft = new Motor(hardwareMap,"fl", Motor.GoBILDA.RPM_312);
    Motor rearLeft = new Motor(hardwareMap,"rl", Motor.GoBILDA.RPM_312);
    Motor frontRight = new Motor(hardwareMap,"fr", Motor.GoBILDA.RPM_312);
    Motor rearRight = new Motor(hardwareMap,"rr", Motor.GoBILDA.RPM_312);

    private MecanumDrive drive;

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    // Creates a PIDFController with gains kP, kI, kD, and kF
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);



    /*
     * Here are the constructors for the other controllers
     */
    PIDController pid = new PIDController(kP, kI, kD);
    PDController pd = new PDController(kP, kD);
    PController p = new PController(kP);

    /*
    Setting Gains

        // set our gains to some value
        pidf.setP(0.37);
        pidf.setI(0.05);
        pidf.setD(1.02);

        // get our gain constants
        kP = pidf.getP();
        kI = pidf.getI();
        kD = pidf.getD();

        // set all gains
        pidf.setPIDF(kP, KI, kD, 0.7);

        // get all gain coefficients
        double[] coeffs = pidf.getCoefficients();
        kP = coeffs[0];
        kI = coeffs[1];
        kD = coeffs[2];
        kF = coeffs[3];

        // Calculates the output of the PIDF algorithm based on sensor
        // readings. Requires both the measured value
        double output = pidf.calculate(
        // and the desired setpoint
            motor.getCurrentPosition(), setpoint
        );


        // A sample control loop for a motor

            PController pController = new PController(kP);

        // We set the setpoint here.
        // Now we don't have to declare the setpoint
        // in our calculate() method arguments.
        pController.setSetPoint(1200);

        // perform the control loop
            //
             //The loop checks to see if the controller has reached
             // the desired setpoint within a specified tolerance
            // range

        while (!pController.atSetPoint()) {
                double output = pController.calculate(
                        m_motor.getCurrentPosition()  // the measured value
                );
                m_motor.setVelocity(output);
            }
        m_motor.stopMotor(); // stop the motor

        // NOTE: motors have internal PID control


     */


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    // declare odometry pods:

    // define our constants
    static  double TRACKWIDTH = 13.7;
    static  double TICKS_TO_INCHES = 15.3;
    static  double CENTER_WHEEL_OFFSET = 2.4;

    // create our encoders
    //MotorEx encoderLeft, encoderRight, encoderPerp;
    MotorEx encoderLeft = new MotorEx(hardwareMap, "left_encoder");
    MotorEx encoderRight = new MotorEx(hardwareMap, "right_encoder");
    MotorEx encoderPerp = new MotorEx(hardwareMap, "center_encoder");


    //encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
    //encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
    //encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

    // create the odometry object
    HolonomicOdometry holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

    // create the odometry subsystem
    OdometrySubsystem odometry = new OdometrySubsystem(holOdom);

    private static final String VUFORIA_KEY =
            "AQJxDWX/////AAABmZAjn7ozNEuhppnzlDica0AJcipDDeNYHfQI6CH2Isy1wj/oCUGUbtdSF9CLkAQAaV+3n7zYPf5qIihD3Re/jjq9gI/BFxtrCfqseW2DVG8B5pv55THYxl58JZA7C9I3pqbZoz8DOtWFoK26zJ5kp3wuPVJegBJAfsWFdoyurcj0m0+D6MQBGh67UrUdGT9Q/Y34bkvrKKOpNrDDawcb1Elbs694SOPCAO5jqe7v84ZN1ucAU57hcUqF6Qcov2GDNLSDhjD/hawisrWXsItfbsNa23xViioSN1jbri1d6bC6/g2WW/nkbsxAfngC/iWlZpMI0yz7KljgkqtkeDyWMnqfTWF5XM3W/Be6qW13/Yo2";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
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
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
