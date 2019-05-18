package org.firstinspires.ftc.teamcode.EmoryWasHere.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**Starts from depot:
 * lands, hits off correct gold mineral, turns and parks on crater
 * finished by emory on 11/20/18
 */
@Autonomous (name = "AutoDepotV3", group="Depot")

public class AutoDepotV3_3 extends RoverRuckusAuton {

    int step = 0;
    int substep = 0;
    boolean liftHere = false;

    public void init(){
        super.init();
    }

    public void init_loop(){
        super.init_loop();

        telemetry.addData("magOpen: ", magLatchOpen.getState());
        telemetry.addData("magClose: ", magLatchClose.getState());
        telemetry.addData("magUp: ", magSwitchUp.getState());
        telemetry.addData("magDown: ", magSwitchDown.getState());
        telemetry.addData("shoulder:",pShoulder.getVoltage()/3.25);
        telemetry.addData("encoder left: ", left.getCurrentPosition());
        telemetry.addData("encoder right: ", right.getCurrentPosition());
    }

    public void loop(){
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("encoder left: ", left.getCurrentPosition());
        telemetry.addData("encoder right: ", right.getCurrentPosition());
        telemetry.addData("order: ", position);
        telemetry.update();

        if (magSwitchUp.getState() && !liftHere) {
            liftUp();
        }
        else {
            liftMotor.setPower(0);
            //if the latch isn't open
            liftHere = true;
            if(magLatchOpen.getState())
            {
                latch.setPower(0.75);
            } else {
                latch.setPower(0);
                goldAlign();
                switch (step) {
                    //distance is 1.28 from blue/red line
                    case 0: disableDetector();
                        switch (position) {
                            case "LEFT":
                                switch (substep) {
                                    case 0:
                                        driveStraight("backward", 12, 0.5);//65 full way to depot 8 to scan actually 20
                                        break;
                                    case 1:
                                        gyroSmart(34);//Rotation(35);//33
                                        break;
                                    case 2:
                                        driveStraight("backward",35, 0.5);//actually 66 //70
                                        break;
                                    case 3:
                                        Rotation(327);//gyroSmart(-33);//0.35 gyroSmart(-33);
                                    case 4:
                                        driveStraight("backward", 12, 0.5);
                                        step++;
                                        break;
                                    default:
                                        requestOpModeStop();
                                        break;

                                }//end of switch substep
                                substep++;
                                break;
                            case "RIGHT":
                                switch (substep) {
                                    case 0:
                                        driveStraight("backward", 12, 0.5);//65 full way to depot 8 to scan
                                        break;
                                    case 1:
                                        gyroSmart(-35);//Rotation(325);
                                        break;
                                    case 2:
                                        driveStraight("backward",30, 0.5);//actually 66 //70
                                        break;
                                    case 3:
                                        gyroSmart(35);//Rotation(35);
                                    case 4:
                                        driveStraight("backward", 23, 0.5);
                                        step++;
                                        break;
                                    default:
                                        requestOpModeStop();
                                        break;
                                }//end of switch substep
                                substep++;
                                break;
                            case "CENTER":
                                driveStraight("backward", 56, 0.5);//89
                                step++;
                                break;
                            default:
                                driveStraight("backward", 53, 0.5);
                                break;
                        }//end of switch order
                        break;
                    case 1:
                        depositMarker(1500);//mervoShoulder(0.363);//0.29 //.267
                        step++;
                        break;
                    case 2:
                        //mervoShoulder(0.25);
                        step++;
                        break;
                    case 3:
                        if(position.equals("LEFT")){
                            gyroSmart(110);//Rotation(110);//0.45 //-47 -60 //0.43
                        }
                        else{
                            gyroSmart(75);//Rotation(80);//0.45 //-47 -60 //0.43
                        }
                        step++;
                        break;
                    case 4:
                        if(position.equals("LEFT")){
                            driveStraight("backward", 8, 0.5);
                        }
                        else{
                            driveStraight("backward", 12, 0.5);
                        }

                        step++;
                        break;
                    case 5:
                        gyroSmart(116);//Rotation(116);//0.45 //-47 //-48 //0.35
                        step++;
                        break;
                    case 6:
                        if(position.equals("RIGHT")){
                            driveStraight("backward", 55, 0.5);//85 //90 //73
                        }
                        else if(position.equals("LEFT")){
                            driveStraight("backward", 40, 0.5);
                        }
                        else{
                            driveStraight("backward", 46, 0.5);//85 //90 //73
                        }

                        step++;
                        break;
                    case 7:
                        mervoShoulder(0.7);
                        step++;
                        break;
                    case 8:
                        requestOpModeStop();
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

