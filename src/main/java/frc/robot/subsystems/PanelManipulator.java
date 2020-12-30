package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;

import as7262.AS7262;
import as7262.AS7262Registers;
import frc.robot.Constants;

import cyberlib.io.Xbox;
import cyberlib.utils.CheckFaults;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PanelManipulator extends Subsystem {

    protected PeriodicIO mPeriodicIO;
    private CheckFaults cf = new CheckFaults();

    private final TalonSRX mTalon;
    //private final Solenoid mDeploySolenoid;

    private Xbox operator = null;

    private static AS7262 mColorSensor;
    
    private boolean sensorLEDOn = false; 

    public enum SystemState {
        STOWING, OPEN_LOOP, POSITION_ROTATE, DISTANCE_ROTATE
    }

    public enum WantedState {
        STOW, OPEN_LOOP, POSITION_ROTATE, DISTANCE_ROTATE;
    }

    public enum PhysicalSolenoidState {
        EXTENDED(true), RETRACTED(false);

        private final boolean extended;

        private PhysicalSolenoidState(final boolean extended) {
            this.extended = extended;
        }

        public boolean getExtended() {
            return extended;
        }
    }

    public enum PanelColors {
        ERROR(-1), NULL(0), BLUE(1), GREEN(2), RED(3), YELLOW(4);

        private final int value;

        // each wheel color has a corresponding color so that it shows sequence
        private PanelColors(final int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }

        public String toString(){
            switch(value){
                case 0:
                    return "NULL";
                case 1:
                    return "BLUE";
                case 2:
                    return "GREEN";
                case 3:
                    return "RED";
                case 4:
                    return "YELLOW";
                default:
                    // System.out.println("ERROR in Color Sensor.");
                    return "ERROR";
            } 
        }
    }

    protected SystemState mSystemState = SystemState.STOWING;
    protected WantedState mWantedState = WantedState.STOW;
    private final boolean mLoggingEnabled = true;  // used to disable logging for this subsystem only
    @SuppressWarnings("unused")
    private int mListIndex;

    protected boolean mStateChanged;
    protected PhysicalSolenoidState mSolenoidLastState;

    private static String sClassName;
    private static int sInstanceCount;
    private static PanelManipulator sInstance = null;
    public  static PanelManipulator getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new PanelManipulator(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private PanelManipulator(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        // ports are unknown, so we just put in random numbers
        mTalon = TalonSRXFactory.createDefaultTalon(Constants.kPanelManipulatorTalonID); 
        //mDeploySolenoid = new Solenoid(Constants.kPanelManipulatorSolenoidID); 
        mColorSensor = new AS7262(I2C.Port.kOnboard, Constants.kAS7262RefreshRate);
        // operator = xbox;
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            mSystemState = SystemState.STOWING;
            mWantedState = WantedState.STOW;
            mStateChanged = true;
            mSolenoidLastState = PhysicalSolenoidState.RETRACTED;
        }

        @Override
        public void onLoop(final double timestamp) {
            synchronized (PanelManipulator.this) {
                SystemState newState;
                switch (mSystemState) {
                    case STOWING:
                        newState = handleStowing();
                        break;
                    case OPEN_LOOP:
                        newState = handleOpenLoop();
                        break;
                    case POSITION_ROTATE:
                        newState = handlePositionRotate();
                        break;
                    case DISTANCE_ROTATE:
                        newState = handleDistanceRotate();
                        break;
                    default:
                        newState = SystemState.STOWING;
                        break;
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }


        @Override
        public void onStop(final double timestamp) {
            mTalon.set(ControlMode.PercentOutput, 0.0);
            //mDeploySolenoid.set(PhysicalSolenoidState.RETRACTED.getExtended());
        }
    };

    // public boolean getFaults(){
	// 	return cf.getSRXFaults(mTalon, "WoF");
    // }
    
    public void clearFaults(){
        cf.clearFaults(mTalon);
    }

    public SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case STOW:
                return SystemState.STOWING;
            case OPEN_LOOP:
                return SystemState.OPEN_LOOP;
            case POSITION_ROTATE:
                return SystemState.POSITION_ROTATE;
            case DISTANCE_ROTATE:
                return SystemState.DISTANCE_ROTATE;
            default:
                System.out.println("ERROR: PANEL_MANIPULATOR_STATE_ERROR");
                return SystemState.STOWING;
        }
    }

    private SystemState handleStowing() {  
        // just retract and don't move
        mPeriodicIO.beltDemand = 0;
        mPeriodicIO.solenoidDemand = PhysicalSolenoidState.RETRACTED;

        return defaultStateTransfer();
    }

    private SystemState handleOpenLoop() {  
        mPeriodicIO.beltDemand = Constants.kPanelManipulatorSpinSpeed;
        mPeriodicIO.solenoidDemand = PhysicalSolenoidState.EXTENDED;

        return defaultStateTransfer();
    }

    private PanelColors mWantedColor = PanelColors.NULL;
    private SystemState handlePositionRotate() {  // 

        // get driver station color if it has not been stored in mWantedColors yet
        if (mWantedColor == PanelColors.NULL) {
            mWantedColor = toPanelColor(DriverStation.getInstance().getGameSpecificMessage());

            // if no color found don't deploy
            if (mWantedColor == PanelColors.NULL) {
                mWantedState = WantedState.STOW;
            }
        }
        System.out.println(mWantedColor);
        // if wantedColor is known extend solenoid and calculate belt demand
        if (mWantedColor != PanelColors.NULL) {
            // extend solenoid

            // calc belt demand
            final int x = mWantedColor.getValue() - mPeriodicIO.colorMNReading.getValue();
            if (Math.abs(x) >= 2) {
                mPeriodicIO.solenoidDemand = PhysicalSolenoidState.EXTENDED;
                mPeriodicIO.beltDemand = Math.copySign(Constants.kPanelManipulatorSpinSpeed, -x); 
            } else if (x != 0) {
                mPeriodicIO.solenoidDemand = PhysicalSolenoidState.EXTENDED;
                mPeriodicIO.beltDemand = Constants.kPanelManipulatorSpinSpeed;
            } else { // Panel is set to wantedColor (positioning complete)
                if (operator != null){
                    operator.rumble(Constants.kJoystickRumbleFrequency, Constants.kJoystickRumbleDuration);
                }
                mWantedColor = PanelColors.NULL;
                mWantedState = WantedState.STOW;
            }
        }
        return defaultStateTransfer();
    }
    
    // converts a string to a PanelColor
    private PanelColors toPanelColor(String color) {
        color = color.toLowerCase();
        if (color.equals("blue")) {
            return PanelColors.BLUE;
        } else if (color.equals("green")) {
            return PanelColors.GREEN; 
        } else if (color.equals("red")) {
            return PanelColors.RED;
        } else if (color.equals("yellow")){
            return PanelColors.YELLOW;
        }
        return PanelColors.NULL;
    }

    // keeps track of how many rotations has been done
    private double mRotationCount = 0;

    // keeps track of what was the last color that was read 
    private PanelColors lastColor = PanelColors.NULL;
    private SystemState handleDistanceRotate() { 

        // if the robot is not in position, don't do anything
        if( mPeriodicIO.colorMNReading == PanelColors.NULL){
            mWantedState = WantedState.STOW;
        } else {

            // if this is the first time, record where we started
            if (mStateChanged){
                lastColor = mPeriodicIO.colorMNReading;
            }

            // if the wheel has not been rotated for more than 3.5 rotations
            if (mRotationCount < Constants.kRotationControlLimit){
                // extend solenoid and spin the
                mPeriodicIO.solenoidDemand = PhysicalSolenoidState.EXTENDED;
                mPeriodicIO.beltDemand = Constants.kPanelManipulatorSpinSpeed;

                // every time the robot reads a different color, increase the count by one eighth of an rotation
                if (mPeriodicIO.colorMNReading != lastColor){
                    mRotationCount += Constants.kRotationsPerColorChange;
                    lastColor = mPeriodicIO.colorMNReading; 
                }
            } else {
                // we are done, so go to stow
                if (operator != null){
                    operator.rumble(Constants.kJoystickRumbleFrequency, Constants.kJoystickRumbleDuration);
                }
                mWantedState = WantedState.STOW;
            }
        }
        return defaultStateTransfer();
    }

    // if the operator messes up, he can press a button to get a second chance
    public void resetRotationCount(){
        mRotationCount = 0;
    }

    private PanelColors readMNPanelColor(){
        if(mColorSensor.isConnected() & mColorSensor.isInitialized()){

            float[] blueMNErrors = {
                mPeriodicIO.violetMaxNormalized - Constants.kBlueMNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueMaxNormalized   - Constants.kBlueMNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenMaxNormalized  - Constants.kBlueMNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowMaxNormalized - Constants.kBlueMNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeMaxNormalized - Constants.kBlueMNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redMaxNormalized    - Constants.kBlueMNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            float[] greenMNErrors = {
                mPeriodicIO.violetMaxNormalized - Constants.kGreenMNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueMaxNormalized   - Constants.kGreenMNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenMaxNormalized  - Constants.kGreenMNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowMaxNormalized - Constants.kGreenMNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeMaxNormalized - Constants.kGreenMNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redMaxNormalized    - Constants.kGreenMNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            float[] redMNErrors = {
                mPeriodicIO.violetMaxNormalized - Constants.kRedMNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueMaxNormalized   - Constants.kRedMNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenMaxNormalized  - Constants.kRedMNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowMaxNormalized - Constants.kRedMNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeMaxNormalized - Constants.kRedMNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redMaxNormalized    - Constants.kRedMNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            float[] yellowMNErrors = {
                mPeriodicIO.violetMaxNormalized - Constants.kYellowMNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueMaxNormalized   - Constants.kYellowMNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenMaxNormalized  - Constants.kYellowMNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowMaxNormalized - Constants.kYellowMNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeMaxNormalized - Constants.kYellowMNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redMaxNormalized    - Constants.kYellowMNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            mPeriodicIO.blueMNDistance   = calcEuclidianDistance(blueMNErrors);
            mPeriodicIO.greenMNDistance  = calcEuclidianDistance(greenMNErrors);
            mPeriodicIO.redMNDistance    = calcEuclidianDistance(redMNErrors);
            mPeriodicIO.yellowMNDistance = calcEuclidianDistance(yellowMNErrors);

            int leastDistance = minIndexOfArray(new double[] {mPeriodicIO.blueMNDistance, 
                mPeriodicIO.greenMNDistance, mPeriodicIO.redMNDistance, mPeriodicIO.yellowMNDistance});

            switch(leastDistance) {
                case 0:
                    return PanelColors.BLUE;
                case 1:
                    return PanelColors.GREEN;
                case 2:
                    return PanelColors.RED;
                case 3:
                    return PanelColors.YELLOW;
            }
            
        } else {
            // System.out.println("ERROR: FAILED TO ACHIEVE COLOR SENSOR CONNECTION / INITIALIZATION");
        }
        return PanelColors.ERROR;
    }

    private PanelColors readSNPanelColor(){
        if(mColorSensor.isConnected() & mColorSensor.isInitialized()){

            float[] blueSNErrors = {
                mPeriodicIO.violetSumNormalized - Constants.kBlueSNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueSumNormalized   - Constants.kBlueSNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenSumNormalized  - Constants.kBlueSNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowSumNormalized - Constants.kBlueSNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeSumNormalized - Constants.kBlueSNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redSumNormalized    - Constants.kBlueSNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            float[] greenSNErrors = {
                mPeriodicIO.violetSumNormalized - Constants.kGreenSNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueSumNormalized   - Constants.kGreenSNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenSumNormalized  - Constants.kGreenSNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowSumNormalized - Constants.kGreenSNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeSumNormalized - Constants.kGreenSNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redSumNormalized    - Constants.kGreenSNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            float[] redSNErrors = {
                mPeriodicIO.violetSumNormalized - Constants.kRedSNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueSumNormalized   - Constants.kRedSNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenSumNormalized  - Constants.kRedSNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowSumNormalized - Constants.kRedSNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeSumNormalized - Constants.kRedSNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redSumNormalized    - Constants.kRedSNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            float[] yellowSNErrors = {
                mPeriodicIO.violetSumNormalized - Constants.kYellowSNStockReadings[AS7262Registers.AS726x_VIOLET],
                mPeriodicIO.blueSumNormalized   - Constants.kYellowSNStockReadings[AS7262Registers.AS726x_BLUE  ],
                mPeriodicIO.greenSumNormalized  - Constants.kYellowSNStockReadings[AS7262Registers.AS726x_GREEN ],
                mPeriodicIO.yellowSumNormalized - Constants.kYellowSNStockReadings[AS7262Registers.AS726x_YELLOW],
                mPeriodicIO.orangeSumNormalized - Constants.kYellowSNStockReadings[AS7262Registers.AS726x_ORANGE],
                mPeriodicIO.redSumNormalized    - Constants.kYellowSNStockReadings[AS7262Registers.AS726x_RED   ]
            };

            mPeriodicIO.blueSNDistance   = calcEuclidianDistance(blueSNErrors);
            mPeriodicIO.greenSNDistance  = calcEuclidianDistance(greenSNErrors);
            mPeriodicIO.redSNDistance    = calcEuclidianDistance(redSNErrors);
            mPeriodicIO.yellowSNDistance = calcEuclidianDistance(yellowSNErrors);

            int leastDistance = minIndexOfArray(new double[] {mPeriodicIO.blueSNDistance, 
                mPeriodicIO.greenSNDistance, mPeriodicIO.redSNDistance, mPeriodicIO.yellowSNDistance});

            switch(leastDistance) {
                case 0:
                    return PanelColors.BLUE;
                case 1:
                    return PanelColors.GREEN;
                case 2:
                    return PanelColors.RED;
                case 3:
                    return PanelColors.YELLOW;
            }
            
        } else {
            // System.out.println("ERROR: FAilED TO ACHIEVE COLOR SENSOR CONNECTION / INITIALIZATION");
        }
        return PanelColors.ERROR;
    }

    @Override
    public void stop() {
        mTalon.set(ControlMode.PercentOutput, 0.0);
        //mDeploySolenoid.set(PhysicalSolenoidState.RETRACTED.getExtended());
    }

    public void setWantedState(final WantedState newWantedState) {
        mWantedState = newWantedState;
    }

    @Override
    public String getLogHeaders() {
        return  "PM.value";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return ""+mPeriodicIO.greenRaw;
        }
        return null;
    }

    @Override
    public void readPeriodicInputs() {
        
        // mPeriodicIO.beltCurrent = mTalon.getSupplyCurrent();

        mPeriodicIO.colorMNReading = readMNPanelColor();
        mPeriodicIO.colorSNReading = readSNPanelColor();

        if (mColorSensor.isConnected()){

            if (!sensorLEDOn){
                mColorSensor.enableDrvLed(true);
                sensorLEDOn = true;
            }

            float[] calibratedValues = mColorSensor.getCalibratedValues();

            mPeriodicIO.violetRaw = calibratedValues[AS7262Registers.AS726x_VIOLET];
            mPeriodicIO.blueRaw   = calibratedValues[AS7262Registers.AS726x_BLUE  ];
            mPeriodicIO.greenRaw  = calibratedValues[AS7262Registers.AS726x_GREEN ];
            mPeriodicIO.yellowRaw = calibratedValues[AS7262Registers.AS726x_YELLOW];
            mPeriodicIO.orangeRaw = calibratedValues[AS7262Registers.AS726x_ORANGE];
            mPeriodicIO.redRaw    = calibratedValues[AS7262Registers.AS726x_RED   ];

            float maxValue = maxOfArray(calibratedValues);

            mPeriodicIO.violetMaxNormalized = mPeriodicIO.violetRaw/maxValue;
            mPeriodicIO.blueMaxNormalized   = mPeriodicIO.blueRaw  /maxValue;
            mPeriodicIO.greenMaxNormalized  = mPeriodicIO.greenRaw /maxValue;
            mPeriodicIO.yellowMaxNormalized = mPeriodicIO.yellowRaw/maxValue;
            mPeriodicIO.orangeMaxNormalized = mPeriodicIO.orangeRaw/maxValue;
            mPeriodicIO.redMaxNormalized    = mPeriodicIO.redRaw   /maxValue;

            float colorSum = sumOfArray(calibratedValues);

            mPeriodicIO.violetSumNormalized = mPeriodicIO.violetRaw/colorSum;
            mPeriodicIO.blueSumNormalized   = mPeriodicIO.blueRaw  /colorSum;
            mPeriodicIO.greenSumNormalized  = mPeriodicIO.greenRaw /colorSum;
            mPeriodicIO.yellowSumNormalized = mPeriodicIO.yellowRaw/colorSum;
            mPeriodicIO.orangeSumNormalized = mPeriodicIO.orangeRaw/colorSum;
            mPeriodicIO.redSumNormalized    = mPeriodicIO.redRaw   /colorSum;

        mPeriodicIO.colorSensorConnected = mColorSensor.isConnected();
        mPeriodicIO.colorSensorInitialized = mColorSensor.isInitialized();

        }
    }

    @Override
    public void writePeriodicOutputs() {
        
        mTalon.set(ControlMode.PercentOutput, mPeriodicIO.beltDemand);

        if(mPeriodicIO.solenoidDemand != mSolenoidLastState){
            //mDeploySolenoid.set(mPeriodicIO.solenoidDemand == PhysicalSolenoidState.EXTENDED);
            mSolenoidLastState = mPeriodicIO.solenoidDemand;
        }
    }

    @Override
    public void outputTelemetry() {
        // if (Constants.kDebuggingOutput){
        if (true){
            SmartDashboard.putNumber("PM/panel manipulator belt demand", mPeriodicIO.beltDemand);
            SmartDashboard.putNumber("PM/panel manipulator belt current", mPeriodicIO.beltCurrent);

            SmartDashboard.putBoolean("PM/panel manipulator solenoid extended", mPeriodicIO.solenoidDemand.getExtended());

            SmartDashboard.putString ("PM/Color sensor MN reading", mPeriodicIO.colorMNReading.toString());
            SmartDashboard.putString ("PM/Color sensor SN reading", mPeriodicIO.colorSNReading.toString());
            
            // test code
            
            SmartDashboard.putNumber("PM/VioletRaw:", mPeriodicIO.violetRaw);
            SmartDashboard.putNumber("PM/BlueRaw:"  , mPeriodicIO.blueRaw  );
            SmartDashboard.putNumber("PM/GreenRaw:" , mPeriodicIO.greenRaw );
            SmartDashboard.putNumber("PM/YellowRaw:", mPeriodicIO.yellowRaw);
            SmartDashboard.putNumber("PM/OrangeRaw:", mPeriodicIO.orangeRaw);
            SmartDashboard.putNumber("PM/RedRaw:"   , mPeriodicIO.redRaw   );

            SmartDashboard.putNumber("PM/Violet MN:", mPeriodicIO.violetMaxNormalized);
            SmartDashboard.putNumber("PM/Blue MN:"  , mPeriodicIO.blueMaxNormalized  );
            SmartDashboard.putNumber("PM/Green MN:" , mPeriodicIO.greenMaxNormalized );
            SmartDashboard.putNumber("PM/Yellow MN:", mPeriodicIO.yellowMaxNormalized);
            SmartDashboard.putNumber("PM/Orange MN:", mPeriodicIO.orangeMaxNormalized);
            SmartDashboard.putNumber("PM/Red MN:"   , mPeriodicIO.redMaxNormalized   );

            SmartDashboard.putNumber("PM/Violet SN:", mPeriodicIO.violetSumNormalized);
            SmartDashboard.putNumber("PM/Blue SN:"  , mPeriodicIO.blueSumNormalized  );
            SmartDashboard.putNumber("PM/Green SN:" , mPeriodicIO.greenSumNormalized );
            SmartDashboard.putNumber("PM/Yellow SN:", mPeriodicIO.yellowSumNormalized);
            SmartDashboard.putNumber("PM/Orange SN:", mPeriodicIO.orangeSumNormalized);
            SmartDashboard.putNumber("PM/Red SN:"   , mPeriodicIO.redSumNormalized   );

            SmartDashboard.putNumber("PM/Blue MN Distance:"  , mPeriodicIO.blueMNDistance  );
            SmartDashboard.putNumber("PM/Green MN Distance:" , mPeriodicIO.greenMNDistance );
            SmartDashboard.putNumber("PM/Red MN Distance:"   , mPeriodicIO.redMNDistance   );
            SmartDashboard.putNumber("PM/Yellow MN Distance:", mPeriodicIO.yellowMNDistance);

            SmartDashboard.putNumber("PM/Blue SN Distance:"  , mPeriodicIO.blueSNDistance  );
            SmartDashboard.putNumber("PM/Green SN Distance:" , mPeriodicIO.greenSNDistance );
            SmartDashboard.putNumber("PM/Red SN Distance:"   , mPeriodicIO.redSNDistance   );
            SmartDashboard.putNumber("PM/Yellow SN Distance:", mPeriodicIO.yellowSNDistance);

            SmartDashboard.putBoolean("PM/Color Sensor Connected:"  , mPeriodicIO.colorSensorConnected  );
            SmartDashboard.putBoolean("PM/Color Sensor Initialized:", mPeriodicIO.colorSensorInitialized);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }
    
    public static class PeriodicIO {
        // inputs
        public PanelColors colorMNReading = PanelColors.NULL; // TODO: Check type
        public PanelColors colorSNReading = PanelColors.NULL;
        public double beltCurrent = 0.0;

        public float violetRaw = 0.0f;
        public float blueRaw   = 0.0f;
        public float greenRaw  = 0.0f;
        public float yellowRaw = 0.0f;
        public float orangeRaw = 0.0f;
        public float redRaw    = 0.0f;

        public float violetMaxNormalized = 0.0f;
        public float blueMaxNormalized   = 0.0f;
        public float greenMaxNormalized  = 0.0f;
        public float yellowMaxNormalized = 0.0f;
        public float orangeMaxNormalized = 0.0f;
        public float redMaxNormalized    = 0.0f;

        public double blueMNDistance   = 0.0;
        public double greenMNDistance  = 0.0;
        public double redMNDistance    = 0.0;
        public double yellowMNDistance = 0.0;

        public float violetSumNormalized = 0.0f;
        public float blueSumNormalized   = 0.0f;
        public float greenSumNormalized  = 0.0f;
        public float yellowSumNormalized = 0.0f;
        public float orangeSumNormalized = 0.0f;
        public float redSumNormalized    = 0.0f;

        public double blueSNDistance   = 0.0;
        public double greenSNDistance  = 0.0;
        public double redSNDistance    = 0.0;
        public double yellowSNDistance = 0.0;

        public boolean colorSensorConnected = false;
        public boolean colorSensorInitialized = false;
        
        // outputs
        public PhysicalSolenoidState solenoidDemand = PhysicalSolenoidState.RETRACTED;
        public double beltDemand = 0;
        
    }

    private float maxOfArray(float[] array){
        float currMax = array[0];
        for (int i = 1; i < array.length; i++){
            currMax = Math.max(currMax, array[i]);
        }
        return currMax;
    }

    private float sumOfArray(float[] array){
        float sum = array[0];
        for (int i = 1; i < array.length; i++){
            sum += array[i];
        }
        return sum;
    }

    private double calcEuclidianDistance(float[] array){
        float sumOfSquares = 0f;
        for (float distance1D : array){
            sumOfSquares += distance1D*distance1D;
        }
        return Math.sqrt(sumOfSquares);

    }

    private int minIndexOfArray(double[] array){
        int currMinIndex = 0;
        for (int i = 1; i < array.length; i++){
            if(array[i] < array[currMinIndex]){
                currMinIndex = i;
            }
        }
        return currMinIndex;
    }
}