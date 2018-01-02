//
//  QbotClass.h
//  Qbots_Simulator
//
//  Created by Alvaro Cassinelli on 12/12/17.
//
//

#ifndef QbotClass_h
#define QbotClass_h

#define WITH_MASS // comment to use direct position update (no integration not mass).
    // IMPORTANT NOTE: when there is mass, the particle itself work as a "resonator" having a proper frequency. Otherwise, we need to introduce an arbitrary field sampling frequency.

#define MAX_SPEED_FACTOR 0.15 //  maxParticleSpeed = MAX_SPEED_FACTOR*speedWave;



// to do: QbotParameters here, to better "wrap" them and save/load from XML file:
//typedef struct  {
//    //...
//} QbotParameters;

class Qbot {
    
public:
    
    static const float OMEGA;// = 20.0; // in Hz
    static const float WAVENUMBER;// = 3.0; // not exactly 2.PI/lambda, that would be in far field (circular propagating wave)
    
    enum ShapeQbot {SHAPE_HEXAGON, SHAPE_SQUARE, SHAPE_DISK};
    
    Qbot();
    Qbot(ofVec2f _pos);
    Qbot(ofVec2f _pos, float _phase, bool _emission, bool _motile);
    ~Qbot();
    
    void init(uint16_t _ID, ofVec2f _initPos, float _initPhase, bool _emission, bool _motion);
    
    // Setters and getters:
    void setId(uint16_t _ID) {ID = _ID;}
    uint16_t getId() {return ID;}
    
    void setOmega(float _omega) {omega = _omega;}
    float getOmega() {return omega;}
    
    void setwaveNumberor(float _k) {waveNumber = _k;}
    float getwaveNumberor() {return waveNumber;}
    
    void setFactorMotion(float _factor) {factorMotion = _factor;}
    float getFactorMotion() {return factorMotion;}
    
    void setEmissionMode(bool _emission) {emissionMode = _emission;}
    
    void setMotionState(bool _motion) {motionState = _motion;}
    void setSensingMode(bool _synchUpdate) {
        synchUpdate = _synchUpdate;
    }
    
    void setShape(ShapeQbot _shape) {shape = _shape;}
    void setRadius(float _radius) {radius = _radius;}
    
    // METHODS FOR COMPUTATION:
    void update_DijN_TijN();
    float computeLengthDijNK(Qbot* ptrQj, uint16_t K);
    float computeLocalField();
    ofVec2f computeFieldGradient();
    void computeLocalFieldAndGradient(float& field, ofVec2f& fieldGrad);

    
    // METHODS FOR UPDATE:
    ofVec2f computeNewPosition();
    float computeNewOffsetPhase();
    void update();
    
    // METHODS FOR DRAWING:
    void drawBot();
    void drawTrajectory();
    void drawDijN();
    void drawGradient();
    
    bool mobileQbot;
    bool synchUpdate;
    
    // ========= Public STATIC variables and methods =========
    static void begin();
    
    static vector<Qbot*> vectorPtrQbots; // the whole array of bots
    static uint16_t numQbots; // commodity variable - must ensure it is equal to the output of getNumParticles(), meaning it must be updated when the vectorPtrQbots change size.
    
    //static void createSwarm(uint16_t _numQbots) { ... }
    
    static void add(Qbot* newPtrQbot);
    static uint16_t getNumParticles() {return vectorPtrQbots.size();}
    
    static float getIteration() {return N;}
    
    static float getClock() {return clock;}
    static void resetClock() { clock = 0; }
    static void setIncrementTime(float _timeStep) {timeStep = _timeStep;}
    
    static void setAllOmega(float _omega) {
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            vectorPtrQbots[i]->setOmega(_omega);
        }
    }
    
    static void setAllwaveNumberor(float _k) {
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            vectorPtrQbots[i]->setwaveNumberor(_k);
        }
    }
    
    static void setAllFactorMotion(float _factor) {
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            vectorPtrQbots[i]->setFactorMotion(_factor);
        }
    }
    
     static void addQbot(ofVec2f _pos, float _phase, bool _emission, bool _motile);
    static void addQbot(float x, float y, float _phase, bool _emission, bool _motile);
    static void deleteQbotInRange(uint16_t x, uint16_t y, uint16_t radiusRange);
    static void deleteQbot(Qbot* qbot);
    
    static void deleteAllQbots();
    static void updateAll();
    static void drawAll();
    
    static void freezeEvolution() {modeEvolution=false;}
    static void resumeEvolution() {modeEvolution=true;}
    static void toogleEvolution() {modeEvolution=!modeEvolution;}
    
    static void toggleMotionStateAll() {motionStateAll=!motionStateAll;}
    static void setMotionStateInRange(int x, int y, uint16_t radiusRange, bool _motion);
    
    static void toggleModeBots() {modeBots=!modeBots;}
    static void toggleModeLines() {modeLines=!modeLines;}
    static void toggleModeTrajectory() {modeTrajectory=!modeTrajectory;}
    static void toggleModeGradient() {modeGradient=!modeGradient;}
    
private:
    
    uint16_t ID;
    
    // Note: the wavelenght and angular frequency COULD be different for each particle, but in general will be the same.
    float omega;
    float waveNumber;
    float speedWave; // = omega/waveNumber
    float dX_Wave; // = speedWave*timeStep;
    float sqTimeStep;//; = timeStep*timeStep;
    float sqSpeedWave;//; = speedWave*speedWave;
    float maxParticleSpeed;// = MAX_SPEED_FACTOR*speedWave;
    float sqMaxParticleSpeed;// = maxParticleSpeed* maxParticleSpeed
    float maxdXParticle;
    float sqMaxdXParticle;

    float timePeriodWave;
    float timeCompute;
    
    
    // gradient/field sampling frequency and phase:
    float relativeSamplingFreq; // in "omega" units
    float relativeSamplingPhase;
    
    
    // Variables to tweak the evolution (position and phase of emitter). Not static because they COULD be different for each robot.
    float factorMotion;
    float mass;
    ofVec2f speed; // rem: for the time being, simple euler computation (not verlet)
    //float damp;
    
    // For storing the trajectories and offset phases: we cannot use a fixed length array. Better to use vectors:
    //vector<ofVec2f> X;// we need to store the history of the positions
    ofPolyline X;
    vector<float> offsetPhase; // we need to store the history of the offset phases too for each particle.
    
    // Past times for computing the contributions (retarded fields) from others particles (in the past). For each Qbot, there are numQbots-1 stored times,
    // meaning that we know exactly the size of the storage, but if we ADD or delete robots, we need to resize the array. Hece vector<> is a better option.
    vector<int32_t> TijN; // in the future, a float. For now, an integer.
    // NOTE: no intersection means we leave the value to ZERO (restart scan)
    
    vector<ofVec2f> DijN; //vector defined by the position of the robot j at time TijN (past time, thetime of intersection of light cone of j with the trajectory of THIS robot "i"), and the position of THIS robot at time N (present). Also with a size equal to numQbots-1.
    
    //vector<float> PijN; //the phase of the emitter robot j
    
    // Local field calculation & computation of its local gradient:
    float localField;
    ofVec2f fieldGradient;
    
    // THINGS for DRAWING and CONTROL:
    ShapeQbot shape;
    float radius;
    float angleGradient;
    
    // Modes for individual behaviour:
    bool motionState;
    bool emissionMode; // when false, the robot does not emits its own waves.
    bool attenuationMode; // wave attenuation or not.
    
    // STATIC METHODS & VARIABLES =================================================================
    // ATTN: static member functions can only access static member variables & methods
    // Also, static variables can't be initialised inside the class.
    
    static uint16_t indexID;
    
    static float maxField, minField;
    
    // static ofVec2f dX, dY; // vectors to compute the gradient by finite differences.
    
    // Common clock for all particles:
    static bool modeEvolution;
    static float clock; // Incremented by timeStep every at every update()
    static float timeStep; // artificial time step (the update will be done at constant real time intervals, independent form this)
    static uint16_t N; // iteration (clock, if not reset some time later on, is equal to N*timeStep.
    
    //Modes for global behaviour:
    static bool motionStateAll; // when false, the position is not updated (useful to check the field in a grid).
    
    // Modes & parameters for drawing:
    static ofRectangle borderRectangle;
    
    // DRAWING MODES:
    static bool modeBots;
    static bool modeLines; // draw or not the DijN lines
    static bool modeTrajectory;
    static bool modeGradient;
    
    // Better to leave to the main program the control of the visualsation real speed:
    // static float updateTimer, updatePeriod;
};


#endif /* QbotClass_h */


/*
 
 Old comments / pieces of code ================================
 
 (1) More accurate method:
 vector<array<float, 3>> T; // for each Qbot, there will be 3x(N-1) stored times for each other particle influence (the factor 3 comes from the fact we need to calculate the field at three points to compute the gradient). We access it as: T[i][]
 (2) A little less accurate methods, storing only one value T[i] and:
 (a) Compute the gradient as
 
 
 static float n;  // Commodity value: clock = n.timeStep;
 // This will be useful to access the array of positions/phases (it has to be equal to the size of vector<ofVec2f> X;
 
 
 // For the "past times" array Tj, we know the size (it is 3x(N-1), where N is the number of Qbots). Since the number of Qbots is set outside the class, we need to use dynamic allocation (for the first dimensoin). We can use normal "C" arrays, or a vector container (not an array container, because it is not dynamically allocated).
 
 
 ofVec2f Qbot::computeFieldGradient(const ofVec2f& _pos) {
 
 float h = 1.0;
 float fo=computeField(_pos);
 float fX=computeField(_pos+h*dX);
 float fY=computeField(_pos+h*dY);
 
 return(ofVec2f((fX-fo)/h,(fY-fo)/h));
 }
 
 
 */
