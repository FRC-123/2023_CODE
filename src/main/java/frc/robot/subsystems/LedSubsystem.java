package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedSubsystem extends SubsystemBase {
    private static AddressableLED led_bar;
    // don't know if its better to declare a bunch of static buffers (20 x 3 bytes
    // each) for each "message", or iterate over a background buffer and swap with
    // active display
    private static AddressableLEDBuffer led_our_alliance;
    private static AddressableLEDBuffer led_blank;
    private static AddressableLEDBuffer led_cube_req;
    private static AddressableLEDBuffer led_cone_req;
    private static DriverStation.Alliance our_alliance;


    /**
     * 
     */
    public LedSubsystem() {

        // init port
        led_bar = new AddressableLED(0);
        led_bar.setLength(20);

        our_alliance = DriverStation.getAlliance();     // maybe need to check this in disabled to see if it changed
// //    DriverStation.Alliance color;
// color = DriverStation.getAlliance();
// if(color == DriverStation.Alliance.Blue){
// //                       RobotContainer.isRedAlliance = false;
// }else {
// //                       RobotContainer.isRedAlliance = true;
// }

        // allocate buffers

        led_our_alliance = new AddressableLEDBuffer(20);
        led_blank = new AddressableLEDBuffer(20);
        led_cube_req = new AddressableLEDBuffer(20);
        led_cone_req = new AddressableLEDBuffer(20);

        // init message buffers
        for (int i = 0; i < 20; i++) {
            // pre-set all message buffers during init
            if ( our_alliance == DriverStation.Alliance.Red ) {
                led_our_alliance.setLED(i, Color.kFirstRed);
            } else if ( our_alliance == DriverStation.Alliance.Blue ) {
                led_our_alliance.setLED(i, Color.kFirstBlue);
            } else {    // invalid
                led_our_alliance.setLED(i, Color.kGreen);   // as a warning
            }
            led_blank.setLED(i, Color.kBlack);
            led_cube_req.setLED(i, Color.kDarkViolet); // cube color req
            led_cone_req.setLED(i, Color.kGold); // cone color req
        }
        led_bar.setData(led_blank);
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

    /**
     * 
     */
    public void set_our_alliance_solid() {
        led_bar.setData(led_our_alliance);
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
