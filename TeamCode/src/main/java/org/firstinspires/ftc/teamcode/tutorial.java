package org.firstinspires.ftc.teamcode;

//Libraries
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tutorial")
public class tutorial extends OpMode {

    //For the motor config(Soon)
    DcMotor motor;
    double ticks = 1000.2; //Depends on the motor (Not sure yet)
    double newTarget;

    //When Init button is pressed
    @Override
    public void init() {
        //Sends the text to the device and update it
        telemetry.addData("Initialization:", "is a success");
        telemetry.update();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //When pressing the start button, loop the code
    @Override
    public void loop() {
        //When "A" button is pressed...
        if (gamepad1.a) {
            encoder(2); //Half of the ful rotation
        }

        if(gamepad1.b) {
            tracker();
        }
    }

    public void encoder(int turnage) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops and reset ticks Helos
        newTarget = ticks / turnage; // How small the turn should be
        motor.setTargetPosition((int) newTarget); // Make the half turn
        motor.setPower(.3); //With this much power
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs it
    }
    public void tracker(){
        //This resets the ticks back to origin
        motor.setTargetPosition(0);
        motor.setPower(.8);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}