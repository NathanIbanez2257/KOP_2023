    
package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

public final class Constants 
{

    public static final class DriveConstants 
    {
        public static final double 

        ksVolts = 0.56093,
        
        kvVoltSecondsPerMeter = 2.3578,
        
        kaVoltSecondsSquaredPerMeter = 0.25974,
        
        kPDriveVelcotiy = 3.0225,
        
        kTrackWidthMeters = Units.inchesToMeters(25),
        
        kMaxSpeedMetersPerSecond = 3,
        
        kMaxAccelerationMPSSquared = 3,

        kRamsesteB = 2,
        
        kRamseteZeta = .7,

        
        /*
        ksVolts = 0.69076,
        
        kvVoltSecondsPerMeter = 2.2127,
        
        kaVoltSecondsSquaredPerMeter = 0.30695,
        
        kPDriveVelcotiy = 3.0306,
        
        kTrackWidthMeters = Units.inchesToMeters(25),
        
        kMaxSpeedMetersPerSecond = 3,
        
        kMaxAccelerationMPSSquared = 3,

        kRamsesteB = 2,
        
        kRamseteZeta = .7,
        */

        kGearRatio = 10.71,
        
        kWheelRadiusInches = 3,


        kLinearDistanceConversionFactor = 
        (Units.inchesToMeters(1/(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10) * 2),
        
        
        
        kxConversion = (Math.PI * 6 * 10.71 * 10 * ( (10.864 / (1 / (Units.metersToInches(1)/ (Math.PI * 6)))  ))),

        kMeterConversion = (1 / ( (21934.08/ (2048 / (Units.metersToInches(1) / (Math.PI * 6)  ))) * (9.8 * (Units.metersToInches(1)/(Math.PI * 6)))  )),

        kMeterConversionAccurate = (1 / ((10.71 * Math.PI * 6) * ( (2048/(Math.PI * 6) / (100/(Units.metersToInches(1) / (Math.PI * 6)))) )) ),

        kMeterNew = (1/ ((10.71 * Math.PI *6) * 10 * 22.6910)),

        kMeterAndres = Math.PI /(2048*10.71*6),
        
        kCountsPerRev = 2048;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);










        //(98 * (Units.metersToInches(1)/(Math.PI * 6)))
        //22.3694273094

        // 6.56168333 
        // 10.14
         






        public static int 

        leftMotorID = 1, rightMotorID = 2;
        /*
        "counts per rotation": 2048.0,
        "encoder type": "Built-in",
        "encoding": false,
        "gearing denominator": 1.0,
        "gearing numerator": 0.14,
        */
    }



public static final int 




///////////////////////////////         Controllers         /////////////////////////////////////////////////

nathan = 0, gio = 1, daniel = 2,                                // controller ports

leftDriveAxis = 1, rightDriveAxis = 5,                          // controller axis 

fowardsAxis = 1, turnAxis = 0,




///////////////////////////////         Falcon Id's        /////////////////////////////////////////////////

rightFront = 10, rightRear = 11, leftFront = 13, leftRear = 12,      // drive ID's

PigeonID = 3,

shooterMotor = 16, cascadeMotor = 18,                                // cascade ID's

rotaryArmMotor = 15,                                                 // rotary arm ID



////////////////////////////////        Talon SRX       //////////////////////////////////////////////

indexMotor = 20,                                                     // indexer ID



///////////////////////////////           Controller Buttons        ////////////////////////////////////////






                    /////////////           Nathan Controls              ///////////////

rotaryForwardButton = 8, rotaryBackwardButton = 6,                      // rotary arms buttons 
rotaryForwardSlowButton = 11, rotaryBackwardSlowButton = 12,

cascadeDownButton = 5, cascadeUpButton = 7,                             // cascade buttons
cascadeDownSlowButton = 14,

// IndexerOutNathan = 2, IndexerInNathan = 1,                           // indexer



                    ////////////        Gio Controls            //////////////////

indexerOutGio = 6, indexerInGio = 5,                                                        // indexer buttons
indexerInMaxGio = 9, aimButtonPS = 14, shooterAdjustButton = 13,

shooterMax = 10, shooterClose = 7, shooterMid = 8,                                          // shooter controls
shooterFar = 4, shooterRev = 6, lowerHub =  1, lowerHub25 = 2,                              // shooter controls 

aimRobotButton2 = 7,                                                                        // aiming button    

indexerOutDaniel = 5, indexerInDaniel = 6,
shooterRevDaniel = 5,




///////////////////////////             Auto                ////////////////////////////////////

tarmacDistance = -3, initialMove = 6;



public static double



////////////////////////                 Vision Tracking                /////////////////////////////////

hubHeight = 8.667, limelightHeight = 2.25, bottomAngle = 25, targetDistance1 = 10,

targetHeight = 69, limeBaseHeight = 27, limeAngle = 27,

targetDistance = 137,



///////////////////////               Limelight Angle Tracking PIDS      ////////////////////////////////

shortAimKP = 0.01, shortAimKI = .025,

longAimKP = .007, longAimKI = 0.01,  

aimKD = 0.6, targetAngle = 0.00,



////////////////////////                  PIDS for drive                ////////////////////////////////

//aimKP = 0.05, aimKI = 0, aimKD1 = 0, 

distanceKP = 0.38, distanceKI = 0.09, distanceKD = 0.07,               

//aimKP = .09, aimKI = 0.0001, aimKD = 0, distanceKP = .1 distanceKD 0.01, dKP = .13



////////////////////////                 PIDS for shooter               ////////////////////////////////

shooterKP = -.1,                                                 // shooter KP



////////////////////////                     Encoder                    ////////////////////////////////

kEncoderDistancePerPulse = ((Math.PI * 5.75)/(24576 * 9.375)),



/////////////////////////////                Subsystem Speeds                ///////////////////////////////////

shooterMaxSpeed = 1, shooterCloseSpeed = .5, shooterMidSpeed = .6,                              // shooter speeds
shooterFarSpeed = .7, shooterRevSpeed = .4,  shooterLowerSpeed = .3, shooter25 = .25,           // shooter speeds                        

cascadeUpSpeed = .6, cascadeUpSlowSpeed = .55,                                                  // cascade speeds
cascadeDownSpeed = -.6, cascadeDownSlowSpeed = .55,                                             // cascade speeds

driveSpeed =.84,                                                                                // drive speeds

indexerSpeedMax = .8, indexerInSpeed = .5, indexerOutSpeed = -.5,                               // indexer speeds

rotaryArmFowardSpeed = .5, rotaryArmBackwardSpeed = -.5,                                        // rotary arm speeds 
rotaryArmFowardSlowSpeed = .45, rotaryArmBackwardSlowSpeed = -.45,                              // rotary arm speeds

shooterOut = -.5;                                                                               // shooter outake speeds



}

