package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedSubsystem extends SubsystemBase {
    private static AddressableLED led_bar;
    // don't know if its better to declare a bunch of static buffers (20 x 3 bytes
    // each) for each "message", or iterate over a background buffer and swap with
    // active display
    private static AddressableLEDBuffer led_red_alliance;    
    private static AddressableLEDBuffer led_blue_alliance;
    private static AddressableLEDBuffer led_red_blue;   
    private static AddressableLEDBuffer led_blank;
    private static AddressableLEDBuffer led_cube_req;
    private static AddressableLEDBuffer led_cone_req;
    private static AddressableLEDBuffer led_dynamic_msg;    // possible blinking message
    private static int dynamic_count = 0;   // internal counter for dynamic method

    /**
     * 
     */
    public LedSubsystem() {

        // init port
        led_bar = new AddressableLED(0);
        led_bar.setLength(20);

        // allocate buffers

        led_red_alliance = new AddressableLEDBuffer(20);
        led_blue_alliance = new AddressableLEDBuffer(20);
        led_red_blue = new AddressableLEDBuffer(20);
        led_blank = new AddressableLEDBuffer(20);
        led_cube_req = new AddressableLEDBuffer(20);
        led_cone_req = new AddressableLEDBuffer(20);
        led_dynamic_msg = new AddressableLEDBuffer(20);

        // init message buffers
        for (int i = 0; i < 20; i++) {
            // pre-set all message buffers during init
            led_red_alliance.setLED(i, Color.kFirstRed);
            led_blue_alliance.setLED(i, Color.kFirstBlue);
            if ( ((i&3)==0) || ((i&3)==2)) { // alternate every 4 pixels
                led_red_blue.setLED(i, Color.kFirstRed);
            } else {
                led_red_blue.setLED(i, Color.kFirstBlue);
            }
            led_blank.setLED(i, Color.kBlack);
            led_dynamic_msg.setLED(i, Color.kBlack);
            led_cube_req.setLED(i, Color.kDarkViolet); // cube color req
            led_cone_req.setLED(i, Color.kGold); // cone color req
        }
        led_bar.setData(led_red_blue);
        led_bar.start(); // optionally stop during disable, start on enable transition?
    }

    /**
     * 
     */
    public void stop_led_bar() {
        led_bar.stop();
    }

    /**
     * 
     */
    public void start_led_bar() {
        led_bar.start();
    }


    public void set_our_alliance_solid(boolean isRedAlliance) {
        if ( isRedAlliance ) {
            led_bar.setData(led_red_alliance);
        } else {
            led_bar.setData(led_blue_alliance);
        }
        // if ( our_alliance == DriverStation.Alliance.Red ) {
        //     led_bar.setData(led_red_alliance);
        // } else if ( our_alliance == DriverStation.Alliance.Blue ) {
        //     led_bar.setData(led_blue_alliance);
        // } else {    // invalid
        //     led_bar.setData(led_red_blue);
        // }
    }

    /**
     * 
     */
    public void set_dynamic_message() {
        dynamic_count++;
        led_bar.setData(led_red_blue);
        // do something fancy to dynamic mesg here...
        // led_bar.setData(led_dynamic_msg);
    }

    /**
     * 
     */
    public void set_cube_req() {
        led_bar.setData(led_cube_req);
    }

    /**
     * 
     */
    public void set_cone_req() {
        led_bar.setData(led_cone_req);
    }

}
