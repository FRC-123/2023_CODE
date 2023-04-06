package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private static AddressableLED led_bar;
    // don't know if its better to declare a bunch of static buffers (20 x 3 bytes
    // each) for each "message", or iterate over a background buffer and swap with
    // active display
    private static LedState currenState;
    private static AddressableLEDBuffer led_red_alliance;    
    private static AddressableLEDBuffer led_blue_alliance;
    private static AddressableLEDBuffer led_red_blue;   
    private static AddressableLEDBuffer led_blank;
    private static AddressableLEDBuffer led_green;
    private static AddressableLEDBuffer led_cube_req;
    private static AddressableLEDBuffer led_cone_req;
    private static AddressableLEDBuffer led_dynamic_msg;    // possible blinking message
    private static int dynamic_count = 0;   // internal counter for dynamic method
    // Store what the last hue of the first pixel is
    private static int m_rainbowFirstPixelHue = 0; 

    static {
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
        led_green = new AddressableLEDBuffer(20);

        // init message buffers
        for (int i = 0; i < 20; i++) {
            // pre-set all message buffers during init
            led_red_alliance.setLED(i, Color.kRed);
            led_blue_alliance.setLED(i, Color.kBlue);
            led_green.setLED(i, Color.kGreen);
            if ( ((i&3)==0) || ((i&3)==2)) { // alternate every 4 pixels
                led_red_blue.setLED(i, Color.kRed);
            } else {
                led_red_blue.setLED(i, Color.kBlue);
            }
            led_blank.setLED(i, Color.kBlack);
            led_dynamic_msg.setLED(i, Color.kBlack);
            led_cube_req.setLED(i, Color.kViolet); // cube color req
            led_cone_req.setLED(i, Color.kYellow); // cone color req
        }
        led_bar.setData(led_red_blue);
        currenState = LedState.Alliance;
        led_bar.start(); // optionally stop during disable, start on enable transition?
    }
    /**
     * 
     */
    // public LedSubsystem() {

    //     // init port
    //     led_bar = new AddressableLED(0);
    //     led_bar.setLength(20);

    //     // allocate buffers

    //     led_red_alliance = new AddressableLEDBuffer(20);
    //     led_blue_alliance = new AddressableLEDBuffer(20);
    //     led_red_blue = new AddressableLEDBuffer(20);
    //     led_blank = new AddressableLEDBuffer(20);
    //     led_cube_req = new AddressableLEDBuffer(20);
    //     led_cone_req = new AddressableLEDBuffer(20);
    //     led_dynamic_msg = new AddressableLEDBuffer(20);
    //     led_green = new AddressableLEDBuffer(20);

    //     // init message buffers
    //     for (int i = 0; i < 20; i++) {
    //         // pre-set all message buffers during init
    //         led_red_alliance.setLED(i, Color.kFirstRed);
    //         led_blue_alliance.setLED(i, Color.kFirstBlue);
    //         led_green.setLED(i, Color.kGreen);
    //         if ( ((i&3)==0) || ((i&3)==2)) { // alternate every 4 pixels
    //             led_red_blue.setLED(i, Color.kFirstRed);
    //         } else {
    //             led_red_blue.setLED(i, Color.kFirstBlue);
    //         }
    //         led_blank.setLED(i, Color.kBlack);
    //         led_dynamic_msg.setLED(i, Color.kBlack);
    //         led_cube_req.setLED(i, Color.kDarkViolet); // cube color req
    //         led_cone_req.setLED(i, Color.kGold); // cone color req
    //     }
    //     led_bar.setData(led_red_blue);
    //     led_bar.start(); // optionally stop during disable, start on enable transition?
    // }

    /**
     * 
     */
    public static void stop_led_bar() {
        led_bar.stop();
    }

    /**
     * 
     */
    public static void start_led_bar() {
        led_bar.start();
    }

    /**
     * 
     */
    public static void set_blank_msg() {
        led_bar.setData(led_blank);
        currenState = LedState.Alliance;
    }

    /**
     * 
     */
    public static void set_red_blue_msg() {
        led_bar.setData(led_red_blue);
        currenState = LedState.Alliance;
    }

    /**
     * 
     */
    public static void set_green_msg() {
        led_bar.setData(led_green);
        currenState = LedState.Alliance;
    }


    /**
     * intended to be called once at start of teleop & auton enable
     */
    public static void set_our_alliance_solid() {
        DriverStation.Alliance our_alliance = DriverStation.getAlliance();
        if ( our_alliance == DriverStation.Alliance.Red ) {
            led_bar.setData(led_red_alliance);
        } else if ( our_alliance == DriverStation.Alliance.Blue ) {
            led_bar.setData(led_blue_alliance);
        } else {    // invalid
            led_bar.setData(led_red_blue);
        }
    // or with boolean isRedAlliance as argument...
        // if ( isRedAlliance ) {
        //     led_bar.setData(led_red_alliance);
        // } else {
        //     led_bar.setData(led_blue_alliance);
        // }
        currenState = LedState.Alliance;
    }

    private static void rainbow() {
        // For every pixel
        for (var i = 0; i < led_dynamic_msg.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / led_dynamic_msg.getLength())) % 180;
          // Set the value
          led_dynamic_msg.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }
    
    /**
     * 
     */
    public static void set_dynamic_message() {
        if ( dynamic_count < 1000 ) {
            dynamic_count++;
        }
        rainbow();  // or do something else fancy to dynamic mesg here...
        led_bar.setData(led_dynamic_msg);
        currenState = LedState.Alliance;
    }

    /**
     * 
     */
    public static void set_cube_req() {
        led_bar.setData(led_cube_req);
        currenState = LedState.Cube;
    }

    /**
     * 
     */
    public static void set_cone_req() {
        led_bar.setData(led_cone_req);
        currenState = LedState.Cone;
    }

    public static void toggle_cone() {
        if(currenState == LedState.Cone) {
            set_our_alliance_solid();
        }
        else {
            set_cone_req();
        }
    }

    public static void toggle_cube() {
        if(currenState == LedState.Cube) {
            set_our_alliance_solid();
        }
        else {
            set_cube_req();
        }
    }
}

enum LedState {
    Alliance,
    Cone,
    Cube
}