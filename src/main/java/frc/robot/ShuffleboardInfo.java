// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import java.util.Map;
 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
 
/**
 * This is a singleton class that will hold all the Shuffleboard entries. This class
 * is where the Shuffleboard layout will be taken care of.  A singleton class is
 * a class instance where there is only one of them instantiated.  It allows a nice
 * communication method for each of the Subsystems to be able to talk to each other
 * through well-known Network Table entries.
 *
 * This should enable things like the Drivetrain directing the LED subsystem  
 * bits of information that the LEDs can communicate to the drive team.
 *
 * It will take care of giving access to all the various pieces of information
 */
public class ShuffleboardInfo {
 
    // There should be one tab for each subsystem or type of info to track
    private  final ShuffleboardTab testTab;
   
    // Create NetworkTableEntry for each data that is to be tracked/displayed
    private final NetworkTableEntry bottomClimbEntry, topClimbEntry;
    private final NetworkTableEntry drivetrainLeftFrontMotorEntry, drivetrainLeftRearMotorEntry, drivetrainRightFrontMotorEntry, drivetrainRightRearMotorEntry;
    private final NetworkTableEntry towerUpperMotorEntry, towerLowerMotorEntry, towerLowerSensorEntry, towerUpperSensorEntry;
    private final NetworkTableEntry shooterMotorEntry, shooterStateEntry;
    private final NetworkTableEntry intakeRotationMotorPosition, intakeRotationMotorSpeed, intakeMotorSpeed, intakeUpperLimitSwtich, intakeLowerLimitSwitch;

   
    // To better organize the data displayed, use Shuffleboard Layouts. These will all be setup in
    // ShuffleboardInfo except for the Layout for the Command buttons. These are used to unit test the functions
    // and need to be setup in RobotContainer as this is where the subsystems that the Commands use are created.
    // Create an accessor method for this Layout    
    private final ShuffleboardLayout commandLayout;
 
    // A private constructors is used to create a singleton, then provide some
    // sort of accessor method like getInstance(). Then the getinstance checks
    // whether or not we have an instantiated instance, and if not, then creates
    // it.
    private static ShuffleboardInfo instance = null;
 
    // Reminder: Needs to be a private constructor for it to be a Singleton!
    private ShuffleboardInfo() {
 
        // This is the tab where we will put our debug test data to make it easy to view everything at once
        // It could also have the data divided by subsystem for easier retrieval in the program
        testTab = Shuffleboard.getTab("Test");
 
        // For each subsystem or type of data being displayed, they are added to a group to more compactly display the data
        // and to more easily view data from one subsystem
        ShuffleboardLayout climberLayout = testTab.getLayout("Climber", BuiltInLayouts.kList).withPosition(5,0).withSize(1, 1).withProperties(Map.of("Label position", "HIDDEN"));
        ShuffleboardLayout drivetrainLeftLayout = testTab.getLayout("Drivetrain Left", BuiltInLayouts.kList).withPosition(3,0).withSize(1, 1).withProperties(Map.of("Label position", "HIDDEN"));
        ShuffleboardLayout drivetrainRightLayout = testTab.getLayout("Drivetrain Right", BuiltInLayouts.kList).withPosition(4,0).withSize(1, 1).withProperties(Map.of("Label position", "HIDDEN"));
        ShuffleboardLayout towerLayout = testTab.getLayout("Tower", BuiltInLayouts.kList).withPosition(2,0).withSize(1, 6);
        ShuffleboardLayout shooterLayout = testTab.getLayout("Shooter", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2).withProperties(Map.of("Label position", "HIDDEN"));
        ShuffleboardLayout intakeLayout = testTab.getLayout("Intake", BuiltInLayouts.kList).withPosition(7, 0).withSize(2, 4);

        // Create the layout for the Command buttons that are used for testing
        commandLayout = testTab.getLayout("Commands", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,8).withProperties(Map.of("Label position", "HIDDEN"));
 
        // Climber Entries
        topClimbEntry = climberLayout.add("Climb Top", 0).getEntry();
        bottomClimbEntry = climberLayout.add("Climb Bottom", 0).getEntry();
 
        // Tower
        towerUpperMotorEntry = towerLayout.add("Upper Speed", 0).getEntry();
        towerUpperSensorEntry = towerLayout.add("Upper Sensor", false).getEntry();
        towerLowerMotorEntry = towerLayout.add("Lower Speed", 0).getEntry();
        towerLowerSensorEntry = towerLayout.add("Lower Sensor", false).getEntry();
       
        // Drivetrain
        drivetrainLeftFrontMotorEntry = drivetrainLeftLayout.add("Drive LF", 0).getEntry();
        drivetrainRightFrontMotorEntry = drivetrainRightLayout.add("Drive RF", 0).getEntry();
        drivetrainLeftRearMotorEntry = drivetrainLeftLayout.add("Drive LR", 0).getEntry();
        drivetrainRightRearMotorEntry = drivetrainRightLayout.add("Drive RR", 0).getEntry();

        //shooter
        shooterMotorEntry = shooterLayout.add("Shooter motor speed", 0).getEntry();
        shooterStateEntry = shooterLayout.add("Shooter Ready", false).getEntry();
        
        //intake
        intakeRotationMotorPosition = intakeLayout.add("Intake rotation motor position", 0.0).getEntry();
        intakeMotorSpeed = intakeLayout.add("Intake motor speed", 0.0).getEntry();
        intakeRotationMotorSpeed = intakeLayout.add("Intake rotation motor speed", 0.0).getEntry();
        intakeLowerLimitSwitch = intakeLayout.add("lower limit switch", false).getEntry();
        intakeUpperLimitSwtich = intakeLayout.add("upper limit switch", false).getEntry();


    }
 
    // The public accessor method allows this to be a Singleton class
    public static ShuffleboardInfo getInstance(){
        if( instance == null ){
            instance = new ShuffleboardInfo();
        }
        return instance;
    }
 
 
 
    /****
    /* Create getters for all the NetworkTableEntry items and all private variables that users of this class may need
    *****/
    public NetworkTableEntry getBottomClimbEntry() {
        return bottomClimbEntry;
    }
    public NetworkTableEntry getTopClimbEntry() {
        return topClimbEntry;
    }
     public NetworkTableEntry getDrivetrainLeftFrontMotorEntry() {
        return drivetrainLeftFrontMotorEntry;
    }
    public NetworkTableEntry getDrivetrainLeftRearMotorEntry() {
        return drivetrainLeftRearMotorEntry;
    }
    public NetworkTableEntry getDrivetrainRightFrontMotorEntry() {
        return drivetrainRightFrontMotorEntry;
    }
    public NetworkTableEntry getDrivetrainRightRearMotorEntry() {
        return drivetrainRightRearMotorEntry;
    }
    public ShuffleboardLayout getCommandLayout() {
        return commandLayout;
    }
    public NetworkTableEntry getTowerLowerMotorEntry() {
        return towerLowerMotorEntry;
    }
    public NetworkTableEntry getTowerLowerSensorEntry() {
        return towerLowerSensorEntry;
    }
    public NetworkTableEntry getTowerUpperMotorEntry() {
        return towerUpperMotorEntry;
    }
    public NetworkTableEntry getTowerUpperSensorEntry() {
        return towerUpperSensorEntry;
    }
    public NetworkTableEntry getShooterMotorEntry() {
        return shooterMotorEntry;
    }
    public NetworkTableEntry getShooterStateEntry() {
        return shooterStateEntry;
    }
    public NetworkTableEntry getIntakeRotationMotorPosition(){
        return intakeRotationMotorPosition;
    }
    public NetworkTableEntry getIntakeRotationMotorSpeed(){
        return intakeRotationMotorSpeed;
    }
    public NetworkTableEntry getIntakeMotorSpeed(){
        return intakeMotorSpeed;
    }
    public NetworkTableEntry getIntakeUpperLimitSwitch(){
        return intakeUpperLimitSwtich;
    }
    public NetworkTableEntry getIntakeLowerLimitSwich(){
        return intakeLowerLimitSwitch;
    }
 
}
