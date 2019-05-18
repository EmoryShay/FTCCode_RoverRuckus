package org.firstinspires.ftc.teamcode.EmoryWasHere.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**Starts from depot:
 * lands, hits off correct gold mineral, turns and parks on crater
 * finished by emory on 11/20/18
 */
@Autonomous (name = "AutoCraterV5", group="Crater")


public class AutoCraterV5 extends RoverRuckusAuton {

    int step = 0;
    int substep = 0;
    double latchStart = 0;
    boolean liftHere = false, latchHere = false;

    public void init(){
        super.init();
    }

    public void init_loop(){
        super.init_loop();

        telemetry.addData("magOpen: ", magLatchOpen.getState());
        telemetry.addData("magClose: ", magLatchClose.getState());
        telemetry.addData("magUp: ", magSwitchUp.getState());
        telemetry.addData("magDown: ", magSwitchDown.getState());
        telemetry.addData("Shoulder:",pShoulder.getVoltage()/3.25);
        telemetry.addData("encoder left: ", left.getCurrentPosition());
        telemetry.addData("encoder right: ", right.getCurrentPosition());
    }

    public void loop(){

        super.loop();
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("encoder left: ", left.getCurrentPosition());
        telemetry.addData("encoder right: ", right.getCurrentPosition());
        telemetry.update();

        if (magSwitchUp.getState() && !liftHere) {
            liftUp();
        }
        else {
            liftMotor.setPower(0); //I know this is kind of wrong but we're not using the lift after this
            //if the latch isn't open
            liftHere = true;
            if(magLatchOpen.getState())
            {
                latch.setPower(0.75);
            } else {
                latch.setPower(0);
                goldAlign();
                telemetry.addData("order:", position);
                switch (step) {
                    //distance is 1.28 from blue/red line
                    case 0:disableDetector();
                        switch (position) {
                            case "LEFT":
                                switch (substep) {
                                    case 0: driveStraight("backward", 9, 0.8);
                                        break;
                                    case 1: gyroSmart(35); //83 //0.45
                                        break;
                                    case 2: driveStraight("backward", 22, 0.5);//20
                                        break;
                                    case 3: driveStraight("forward", 8, 0.5);
                                        break;
                                    case 4: gyroSmart(80); //84
                                        break;
                                    case 5: driveStraight("backward", 30, 0.5);//actually 28.5 //44 //38
                                        step++;
                                        break;
                                    default: requestOpModeStop();
                                        break;
                                }//end of switch substep
                                substep++;
                                break;
                            case "RIGHT":
                                switch (substep) {
                                    case 0: driveStraight("backward", 9, 0.5);
                                        break;
                                    case 1: gyroSmart(-30); //-88
                                        break;
                                    case 2: driveStraight("backward", 20, 0.5);
                                        break;
                                    case 3: driveStraight("forward", 10, 0.5);
                                        break;
                                    case 4: gyroSmart(82); //84
                                        break;
                                    case 5: driveStraight("backward", 39, 0.4);//actually 45 //90 //55
                                        step++;
                                        break;
                                    default: requestOpModeStop();
                                        break;
                                }//end of switch substep
                                substep++;
                                break;
                            default:
                                switch(substep){
                                    case 0: driveStraight("backward", 26, 0.5);
                                        break;
                                    case 1: driveStraight("forward", 6, 0.5);
                                        break;
                                    case 2: gyroSmart(82);//Rotation(82);
                                        break;
                                    case 3: driveStraight("backward", 38, 0.5);//actually 62.25//62 //57
                                        step++;
                                        break;
                                    default: requestOpModeStop();
                                        break;
                                }
                                substep++;
                                break;
                        }//end of switch order
                        break;
                    case 1:gyroSmart(101); //105
                        step++;
                        break;
                    case 2: driveStraight("backward", 19, 0.5);//actually 45 //20 //18
                        step++;
                        break;
                    case 3: gyroSmart(126);//Rotation(133);
                        step++;
                        break;
                    case 4: driveStraight("backward", 22, 0.5); //25 //30
                        step++;
                        break;
                    case 5: depositMarker(1500);//mervoShoulder(0.38, 1);//0.29 //.267
                        step++;
                        break;
                    case 6: driveStraight("forward", 44, 0.5); //25 //30 //35
                        step++;
                        break;
                    case 7: //gyro2Left(200, 0.33); //0.36 //315 //312
                        step++;
                        break;
                    case 8: gyroSmart(-30); //0.36 //315 //312
                        step++;
                        break;
                    case 9: driveStraight("backward", 6, 0.5);//15
                        step++;
                        break;
                    case 10: mervoShoulder(0.7); //.42
                        step++;
                        break;
                    default:
                        requestOpModeStop();
                        break;
                }

            }//end of switch order
        }
    }



    @Override
    public void stop() {
        super.stop();
    }
}

