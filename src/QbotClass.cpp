//
//  QbotClass.cpp
//  Qbots_Simulator
//
//  Created by Alvaro Cassinelli on 12/12/17.
//
//

#include "ofMain.h"
#include "QbotClass.h"


// Initialization/definition of static variables (do we need this? we can do it in the init method?)

uint16_t Qbot::indexID = 0;

// TODO: better use speed wave and omega as parameter?
const float Qbot::OMEGA = .4; // in Hz
const float Qbot::WAVENUMBER = 2*PI/300; // lambda in pixels
// speed of the wave: omega/waveNumber

//ofVec2f Qbot::dX = ofVec2f(1, 0); // in pixels?
//ofVec2f Qbot::dY = ofVec2f(0, 1); // in pixels?

bool Qbot::modeEvolution = false; // PRESS ENTER to start evolution.
uint16_t Qbot::N = 0;
float Qbot::clock = 0; // Common clock for the emission field
float Qbot::timeStep = 0.3;//0.3; // artificial time step

float Qbot::maxField=-1;
float Qbot::minField=1;

vector<Qbot*> Qbot::vectorPtrQbots; // the whole array of bots
uint16_t Qbot::numQbots=0;

// BEHAVIOUR MODES:
bool Qbot::motionStateAll = true;

// DRAWING MODES & PARAMETERS:
bool Qbot::modeBots = true;
bool Qbot::modeTrajectory = true;
bool Qbot::modeLines  = false;
bool Qbot::modeGradient = false;

ofRectangle Qbot::borderRectangle;

// initialization of the CLASS Qbot (set some global parameters):
void Qbot::begin() {
    borderRectangle=ofRectangle(0,0,ofGetWidth(), ofGetHeight());
}


// CLASS METHODS for INSTANTIATED OBJECTS (non static):

Qbot::Qbot() { // use the static variable "indexID" to increment ID and randomize inital position
    ofVec2f randomPos = ofVec2f(ofRandomWidth(), ofRandomHeight());
    init(indexID, randomPos, 0, true, true, false);
    indexID++;
}

Qbot::Qbot(ofVec2f _pos) {
    init(indexID, _pos, 0, true, true, false);
    indexID++;
}

Qbot::Qbot(ofVec2f _pos, float _phase, bool _emission, bool _motile, bool _synch) {
    init(indexID, _pos, _phase, _emission, _motile, _synch);
    indexID++;
}

Qbot::~Qbot() {
    indexID--;
    // automatically call destructor of vector arrays position and phase.
}

void Qbot::init(uint16_t _ID, ofVec2f _initPos, float _initPhase, bool _emission, bool _motionState, bool _synch) {
    
    ID = _ID;
    
    shape = SHAPE_DISK; //SHAPE_HEXAGONE; //SHAPE_RECTANGLE;
    radius = 6;
    
    X.clear(); // reminder: X is an ofPolyline object (clear() is defined though).
    X.addVertex(_initPos);
    // note: 3rd coordinate is 0 (will register the time from start, i.e N*timeStep)
    
    offsetPhase.clear();
    offsetPhase.push_back(_initPhase);
    
    sqTimeStep = timeStep*timeStep;
    
    omega = OMEGA;
    waveNumber = WAVENUMBER;
    speedWave = omega/waveNumber;
    sqSpeedWave = speedWave*speedWave;
    dX_Wave = speedWave*timeStep; // displacement of the wave in a time step
    timePeriodWave = 2.0*PI/omega;
    
    // continuous or periodic (synchronous) field sampling:
    samplingFreq = 0.5 /timePeriodWave;
    samplingPeriod = 1.0/samplingFreq;
    samplingOffsetPhaseDeg = 180;
    samplingTimeOffset = samplingOffsetPhaseDeg / 360.0 * samplingPeriod;
    lastTimeSample = 0;
    
    maxParticleSpeed = MAX_SPEED_FACTOR*speedWave;
    sqMaxParticleSpeed = maxParticleSpeed*maxParticleSpeed;
    
    maxdXParticle = MAX_SPEED_FACTOR*dX_Wave;
    sqMaxdXParticle = sqMaxParticleSpeed*sqTimeStep;
                    // = (MAX_SPEED_FACTOR*dX_Wave)^2

    localField = 0;
    fieldGradient = ofVec2f(0,0);
    
    factorMotion = 4.0;//1000000.0;
    mass = 0.5;
    
    // Initial speed:
    //speed = ofVec2f(0,0);
    speed = ofVec2f(300*(ID%2-0.5),0); // pixels/timStep
    
    // Default modes:
    attenuationMode = false;
    emissionMode = _emission;
    motionState = _motionState;
    synchMode = _synch;
    
}

// STATIC CLASS METHODS:

void Qbot::addQbot(float x, float y,  float _phase, bool _emission, bool _motile, bool _synch) {
    // use a struct to store Qbot variables? (and have a "default" bot?):
    Qbot* newPtrQbot = new Qbot(ofVec2f(x, y), _phase, _emission, _motile, _synch);
    Qbot::add(newPtrQbot);
}

void Qbot::addQbot(ofVec2f _pos,  float _phase, bool _emission, bool _motile, bool  _synch) {
    // use a struct to store Qbot variables? (and have a "default" bot?):
    Qbot* newPtrQbot = new Qbot(_pos, _phase, _emission, _motile, _synch);
    Qbot::add(newPtrQbot);
}


void Qbot::add(Qbot* newPtrQbot) {
    
    //  ATTENTION!!:
    //    - N is not the number of iterations, but the INDEX of the iteration, starting from 0 (i.e, the number of iterations is N+1).
    //    - In case the Qbot was added when N>0, we need to resize(N+1) the position and phase arrays. We will set their position as fixed until that time, and phase equal to the initial one:
    newPtrQbot->X.resize(N+1); // unfortunately, the ofPolyline method resize does not have the option of a second parameter to fill the whole array with a default value, so we need to do it "by hand":
    for (uint16_t i=1 ; i<N+1; i++) {
        newPtrQbot->X[i] = newPtrQbot->X[0];
    }
    
    //    for (uint16_t i=0 ; i<N+1; i++) {
    //        cout << "pos: " << newPtrQbot->X[i] << endl;
    //    }
    
    newPtrQbot->offsetPhase.resize(N+1, newPtrQbot->offsetPhase[0]);
    
    // The new Qbot does not know the number of Qbots that already exist in the "swarm", so we need to resize things that need to use this value here (or in the "setup" method outside the class, but it is not nice).
    newPtrQbot->TijN.resize(vectorPtrQbots.size(), -1); // negative indicate either no computation or no intersection
    newPtrQbot->DijN.resize(vectorPtrQbots.size(), {0,0});
    
    // Also, each time we add a Qbot, ALL the others have to add a new element to the TijN and DijN arrays (let's do it before adding the new bot to the array):
    for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
        vectorPtrQbots[i]->TijN.push_back(-1);
        vectorPtrQbots[i]->DijN.push_back({0,0});
    }
    
    // Finally, add the Qbot to the array and all should be good:
    vectorPtrQbots.push_back(newPtrQbot);
    
    numQbots =  vectorPtrQbots.size();
    
    
    //    cout << "Qbot "<< newPtrQbot->ID << " added." << endl;
    //    cout << "Todal number of Qbots: " << numQbots << endl;
    //    cout << " added at iter "<< N << endl;
    
}


void Qbot::deleteQbot(Qbot* qbot) {
    
    uint16 l = qbot->ID; //index of the robot to delete in vectorPtrQbots
    
    // * Lets first delete all the unnecessary data on the OTHER robots:
    for (auto it2 = vectorPtrQbots.begin(); it2!= vectorPtrQbots.end(); ) {
        uint16 k = (*it2)->ID;
        
        vector<int32_t>& auxTijN = (*it2)->TijN;
        vector<ofVec2f>& auxDijN = (*it2)->DijN;
        
        //if (l!=k) { // otherwise do nothing
        if ((*it2) != qbot) {
            auxTijN.erase(auxTijN.begin() + l - ( l > k ));
            auxDijN.erase(auxDijN.begin() + l - ( l > k ));
            
        }
    }
    
    // * Then, delete the bot:
    // ATTN!! before deleting the pointer from the vector<>, we need to free the memory of the object pointed by it:
    delete(vectorPtrQbots[l]);
    // then we delete the pointer from the list (luckily, an STL vector enables easy random insertion/deletion)
    vectorPtrQbots.erase(vectorPtrQbots.begin() + l);
    
}

void Qbot::deleteQbotInRange(uint16_t x, uint16_t y, uint16 radiusRange) {
    ofVec2f mousePos = ofVec2f(x, y);
    //for (uint16_t l=0; l < numQbots ; l++) {
    for (auto it1 = vectorPtrQbots.begin(); it1!= vectorPtrQbots.end(); ) {
        
        //if (vectorPtrQbots[l]->X[N].squareDistance(mousePos) < mouseRadiusInfluence*mouseRadiusInfluence) {
        
        ofVec2f XProj = ofVec2f((*it1)->X[N]);
        if (XProj.squareDistance(mousePos) < radiusRange*radiusRange)
            deleteQbot(*it1);
        
    }
}

void Qbot::deleteAllQbots() {
    // first delete each Qbot object instantiated with "new":
    for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
        delete(vectorPtrQbots[i]); // will call the destructor of Qbot
    }
    // Then clear the vector of Qbot pointers:
    vectorPtrQbots.clear();
}

void Qbot::setMotionStateInRange(int x, int y, uint16_t radiusRange, bool _motion) {
    ofVec2f mousePos = ofVec2f(x, y);
    //for (uint16_t l=0; l < numQbots ; l++) {
    for (auto it1 = vectorPtrQbots.begin(); it1!= vectorPtrQbots.end(); ) {
        
        //if (vectorPtrQbots[l]->X[N].squareDistance(mousePos) < mouseRadiusInfluence*mouseRadiusInfluence) {
        
        ofVec2f XProj = ofVec2f((*it1)->X[N]);
        
        if (XProj.squareDistance(mousePos) < radiusRange*radiusRange) (*it1)->setMotionState(_motion);
    }
}

void Qbot::updateAll() {
    
    if (modeEvolution) {
        
        // First update the Qbots:
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            vectorPtrQbots[i]->update();
        }
        
        // AFTER that, update clock and iteration counter N (we want to start with N=0):
        
        clock+=timeStep; // In principle, this is be equal to (X.size()-1)*timeStep, but it can be reset or changed at will (offset it)
        
        N++ ;
        // Note1: N is the iteration, starting from 0, not the number of iterations.
        // Note2: the clock can be reset, but this does not mean the "iteration" is reset, so we can start with clock = 0 but have a "past" trajectory.
        
    }
    
    //cout << "Time:  " <<  clock << endl;
}

void Qbot::drawAll() {
    
    
    if (modeLines)
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            // draw only if the bot is mobile:
            if (vectorPtrQbots[i]->motionState) vectorPtrQbots[i]->drawDijN();
        }
    
    if (modeTrajectory)
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            // draw only if the bot is mobile:
            if (vectorPtrQbots[i]->motionState) vectorPtrQbots[i]->drawTrajectory();
        }
    
    if (modeBots)
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            vectorPtrQbots[i]->drawBot();
        }
    
    if (modeGradient)
        for (uint16_t i=0 ; i<vectorPtrQbots.size(); i++) {
            vectorPtrQbots[i]->drawGradient();
        }
    
    //Mouse "influence" disk (TODO: finish this and correct the delete method)
    //ofNoFill();
    // ofFill();
    // ofSetColor(0,255,0, 50);
    // ofSetCircleResolution(20);
    // ofDrawCircle(ofGetMouseX(), ofGetMouseY(), 30);
    
    // Borders:
    // TODO: different shapes for the confinement (disk...)
    
    ofNoFill();
    ofSetColor(255,0,0, 255); ofSetLineWidth(5);
    
    
    ofPushMatrix();
    
    // attn! rectangle mode set to center:
    //ofTranslate(ofGetWidth()/2, ofGetHeight()/2);
    //ofDrawRectangle(borderRectangle);
    
    ofTranslate(ofGetWidth()/2, ofGetHeight()/2,-0.5*N);
    ofDrawBox(ofGetWidth(), ofGetHeight(), -N);
    
    ofPopMatrix();
    
}

void Qbot::drawBot() {
    
   
    // Mantain dynamic range for color:
    //float hueValue = ofMap(localField, minField, maxField, 0, 255);
    // Note: The logistic function 1/(1+exp(-x)) covers more or
    //       less all the range from 0 to 1 for x between -6 and 6.
    float auxField = 2.0/(expf(-1.0*localField/maxField*1.5)+1.0)-1.0; // from -1 to 1
   // float hueValue = ofMap(auxField, -1.0, 1.0, 0, 255);
    
    float hueValue = ofMap(auxField, -1.0, 1.0, 0, 255.0);

    // cout << "local: " << localField << ", min: " << minField <<", max:" << maxField << " || hue: "<< hueValue << endl;
    
    float auxRadius = radius+emissionMode*radius/5*(1+cos(omega*clock + offsetPhase.back()));
    
    ofPushMatrix();
    ofTranslate(X[N].x, X[N].y, (attenuationMode? 5000.0 : 10.0)*localField);
    
    switch(shape){
        case SHAPE_DISK: // used to represent the bot (bad mix of shape and type... make another enum!)
            ofSetColor(ofColor::fromHsb(hueValue, 255, 255));
            ofFill();
            ofSetCircleResolution(8);
            ofDrawCircle(0,0, auxRadius);
            
            ofSetColor(255); ofNoFill();
            ofSetLineWidth(1);
            ofDrawCircle(0,0, auxRadius+2);
            //ofDrawSphere(0,0,auxRadius);
            break;
            
        case SHAPE_SQUARE:
            ofSetColor(ofColor::fromHsb(hueValue,255, 255, 100));
            ofFill();
            ofTranslate(0,0,-1);
            ofDrawRectangle(0,0,auxRadius,auxRadius);
            break;
            
        case SHAPE_HEXAGON:
            ofSetColor(ofColor::fromHsb(hueValue, 255, 255, 100));
            ofFill();
             ofTranslate(0,0,-1);
            ofSetCircleResolution(6);
            ofDrawCircle(0,0,auxRadius);
            break;
    }
    
    //    if (emissionMode) {
    //        ofSetColor(255,0,0);
    //        ofDrawCircle(0,0, radius/2);
    //    } else {
    //        ofSetColor(0,0,0);
    //        ofDrawCircle(0,0, radius/2);
    //    }
    
    // Rotation from current gradient of the field:
    // ofRotateZ(ofVec2f(1,0).angle(fieldGradient)); // in deg
    
    //Disk:
    //ofDrawCircle(0,0, 5);
    //Ellipse: scaling of large axis as a function of the norm of the gradient
    // ofDrawEllipse(0,0, radiusL + 7 * fieldGradient.length(), radiusS);
    
    // Rectangle:
    // ofDrawRectangle(0,0, radiusL + 5 * fieldGradient.length(), radiusS);
    
    //Fat dot:
    //ofSetLineWidth(10);
    //ofDrawLine(0,0,1,1);
    
    ofPopMatrix();
}

void Qbot::drawGradient() {
    // Rotation from current gradient of the field:
    ofSetColor(255,0,0);
    ofSetLineWidth(3);
    
    ofPushMatrix();
    ofTranslate(X[N]); //ofTranslate(X.back());
    
    ofVec2f arrow = (attenuationMode? 500000.0 : 500.0)*fieldGradient;
    if (arrow.lengthSquared() > 2500) arrow.scale(50);
    ofDrawLine({0,0}, arrow);
    //ofDrawArrow({0,0}, 15*fieldGradient);
    
    ofPopMatrix();
}

void Qbot::drawDijN() {
    
    // (1) draw light cone in the past:
//    ofPushMatrix();
    //  ofTranslate(X[N]);
//    //ofScale(1,1,-1);
//    //ofTranslate(0,0,-N/3);
//    ofRotateX(90);
//    ofSetColor(200,200,100,50);
//    ofDrawCone(0,0,-N/3*timeStep/2, N/3*speedWave, -N/3*timeStep);
//    ofPopMatrix();
    
    // (2) draw connection line (spatiotemporal path for the field):
    ofPushMatrix();
    ofSetColor(0,150,250,100);
    ofSetLineWidth(2);
    ofTranslate(X[N].x, X[N].y,0);
    for (uint16_t j=0; j < numQbots-1 ; j++) {
        if (TijN[j]>0) ofDrawLine(0,0,0, DijN[j].x, DijN[j].y, TijN[j]-N);
    }
    ofPopMatrix();
}

void Qbot::drawTrajectory() {
    ofPushMatrix();
    ofTranslate(0,0,-N);
    ofSetColor(150,200,0,200);
    ofSetLineWidth(3);
    X.draw();
    ofPopMatrix();

}

void Qbot::update() {
    
    ofVec2f newPos = X[N]; // initialized with old position in case we don't change it (but need to store it)
    float newOffPhase = offsetPhase.back(); // old phase
    
    
    // The global clock is not updated here ("clock" is a static member variable shared by all the instances of the class). It has to be updated in the "updateAll" static method.
    //cout << "Time:  " <<  X.size() -1 << endl;
    
    update_DijN_TijN();
    
    //(0) Compute the local field (useful to offset the phase of the emitter, change the color of the robot or something else):
    //    localField = computeLocalField();
    // NOTE: localField is a value that can, at most, be equal (in absolute value) to the sum of all emission amplitudes (damped or not). This is probably a rare event, but for starters I will assume this can happen true. Also for starters, we will assume all the emission amplitudes equal to 1, and no dumping. So, the localField value lies between -numQbots and numQbots.
    
    //(1) Compute the gradient of the field to update the position - and offset phase?. Also to draw the gradient as a vector.
    //    fieldGradient = computeFieldGradient();
    
    // Slightly faster:
    // computeLocalFieldAndGradient(localField, fieldGradient);

    //(3) Update positions by moving proportionnaly to the field gradient:
    // Note: even though X is an array of "vertices" (ofPoint, i.e. ofVec3d), the last coordinate will be ignored, and always equal to 0. Otherwise, we could use it to encode the clock, so as to draw in 3d. But this complicated things when computing DijN. I will use ANOTHER array, X3D, for 3d drawing, not used very often, so I will copy everything if necessary.
    
    //(3) Update offset phase (HOW???)
    // ...make a mode for this (on/off):

    
    if (!synchMode) computeLocalFieldAndGradient(localField, fieldGradient); // continuous update
    else if (clock + samplingTimeOffset - lastTimeSample  > samplingPeriod) {
        // Compute field, gradient, and evolve things only at a certain sample frequency/phase
    
            computeLocalFieldAndGradient(localField, fieldGradient);
            
            lastTimeSample = clock + samplingTimeOffset;
    } // otherwise, don't recalculate the local field.


    // Finally, update position and phase (from new or same field):
    if (motionStateAll && motionState) {
        newPos = computeNewPosition();
        newOffPhase = computeNewOffsetPhase();
    }

    
    X.addVertex(newPos.x, newPos.y, N+1); // N is not changed yet (this is done in the updateAll method), but the length of X has been incremented by one, and it means we "are" at iteration N+1 (length is N+2).
    offsetPhase.push_back(newOffPhase);
    
    // Other things:
    // To mantain dynamic range for colors (we can assume: minField = -maxField):
    if (localField > maxField) maxField = localField;
    else if (localField < minField) minField = localField;
    
    // IMPORTANT: update clock (done for all the bots at the same time, so this is in the updateAll static method)
    //clock+=timeStep;
    
}

ofVec2f Qbot::computeNewPosition() { // from current position and current fieldGradient
   
    // NOTE: the third coordinate of X[k] encodes time. While this is not necessary here, it is practical for drawing the trajectory in space-time (plus the ofPolyline is in fact: vector<ofVec3d>
    // However, when mixing ofVec2d and ofVec3d, when possible the compiler will do a cast to 2d.
    // ofVec2f XProjN = ofVec2f(X[N]);
    
    // HERE: idea, use a function pointer, and define it using a CAS parser addon, such as: https://github.com/xavivives/ofxExprtk
    
    ofVec2f forceFromGradient = - 1.0 * factorMotion*fieldGradient; // (away from gradient)
    
    
#ifdef WITH_MASS
    // NOTE: consider Runge-Kutha / Verlet integration instead of this...
    
    // 1) New Speed:
    ofVec2f dSpeed = forceFromGradient/mass*timeStep;
    speed = speed + dSpeed;
    
    // We need to ensure sub-luminal speed:
    if (speed.lengthSquared() > sqMaxParticleSpeed) speed.scale(maxParticleSpeed);

    //2) New Position:
    ofVec2f dX = speed * timeStep;
    
    // Note: if we just limit the effective displacement to be subliminal, but we don't constrain the value of the speed,
    // then things will be different: the speed can get larger and larger, and then the displacement will always be equal to its
    //maximum value!
   // if (dX.lengthSquared() > sqMaxdXParticle) dX = dX.scale(maxParticleSpeed);
    
#else  // WITHOUT MASS:
    
    ofVec2f dX = forceFromGradient;
    if (dX.lengthSquared() > sqMaxdXParticle) dX = dX.scale(maxdXParticle);
    
#endif 
    ofVec2f newPos = X[N] + dX;
    
    // Check borders:
    //(1) stop or bounce:
    if (newPos.x > ofGetWidth()) newPos.x = ofGetWidth();
    if (newPos.x < 0) newPos.x = 0;
    if (newPos.y > ofGetHeight()) newPos.y = ofGetHeight();
    if (newPos.y < 0) newPos.y = 0;
    //(2) toroidal topology:
    // ...DIFFICULT!
   
    return(newPos);
}

float Qbot::computeNewOffsetPhase() {
    // HOW??? TO DO
    return(1.0);
}


float Qbot::computeLocalField() {
    
    //cout << TijN.size() << " " << numQbots -1 << endl;
    
    float field = 0;
    uint16_t j = 0;
    for (uint16_t l=0; l < numQbots ; l++) {
        // go through all the bots, minus "this" one and those that do not emmit:
        if ( l!=ID && vectorPtrQbots[l]->emissionMode )   {
            // NOTE: skip one element but make index j for DijN and TijN contiguous (there is no IiiN entry, since the motion is not SUPERLUMINAL, otherwise this robot, in the past, can contribute to the field in the present (and perhaps many times, as could do the other too by the way).
            
            // cout << " * field from Qbot[" << l << "] : ";
            
            if (TijN[j]>0) {// in this case, particle j contributes to the field
                
                // float d = DijN[j].length();
                //cout << " OK / " << "TijN[" << j << "] = " << TijN[j] << endl;
                
                // With TijN, we can check any state and variable of the robot j:
                float phaseQbotj = vectorPtrQbots[l]->offsetPhase[TijN[j]];
                
                float sqd = DijN[j].lengthSquared();
                float attenuation = (attenuationMode? 1.0 /sqd : 1.0);
                field+= attenuation*cos( omega*TijN[j]*timeStep + phaseQbotj );
                
            } //else cout << " -- " << endl;
            
            j++;
        }
    }
    
//     cout << " * field on Qbot[" << ID << "] : ";
//     cout << localField << endl;
    
    return(field);
}

ofVec2f Qbot::computeFieldGradient() {
    ofVec2f fieldGrad(0,0);
    uint16_t j = 0;
    for (uint16_t l=0; l < numQbots ; l++) {
        if (l!=ID && vectorPtrQbots[l]->emissionMode ) {
            if (TijN[j]>0) { // otherwise particle j did not contribute to the field
                
                float phaseQbotj = vectorPtrQbots[l]->offsetPhase[TijN[j]];

                float d = DijN[j].length();
                float attenuation = (attenuationMode? 1.0 /(d*d) : 1.0);
                
                fieldGrad -= attenuation*waveNumber*sin(omega*TijN[j]*timeStep + phaseQbotj )*DijN[j]/d;
                
            }
            j++;
        }
    }
    //cout << fieldGrad.length() << endl;
    return(fieldGrad);
}


// small optimization: field and gradient simultaneously:
void Qbot::computeLocalFieldAndGradient(float& field, ofVec2f& fieldGrad) {
    
    fieldGrad.set(0,0);
    field = 0;
    uint16_t j = 0;
    for (uint16_t l=0; l < numQbots ; l++) {
         if ( l!=ID && vectorPtrQbots[l]->emissionMode )   {
           
            if (TijN[j]>0) {
                float phaseQbotj = vectorPtrQbots[l]->offsetPhase[TijN[j]];
                
                float d = DijN[j].length();
                float attenuation = (attenuationMode? 1.0/(d*d) : 1.0);
                
                field+= attenuation*cos( omega*TijN[j]*timeStep + phaseQbotj );
                fieldGrad -= attenuation*waveNumber*sin(omega*TijN[j]*timeStep + phaseQbotj )*DijN[j]/d;
            }
            
            j++;
        }
    }
    
}



void Qbot::update_DijN_TijN() {
    
    // cout << "*** Calcul Qbot[" << ID << "] ***" << endl;
    
    //NOTE: if there is no intersection between the light cone of particle j with the trajectory of particle i at time N, then we will set TijN to a negative value, AND we will skip particle j's retarded potential contribution to the field at i.
    
    // Present iteration (i.e., the number of points in the trajectory of the other bots (ptrQj->X.size() - 1) that SHOULD equal to the number of points of THIS trajectory).
    // NOTE: should always be equal to N = X.size() - 1, and always > 0 because we construct the bot with an inital position.
    
    ofVec2f XProjN = ofVec2f(X[N]) ;
    //cout << XProjN << endl;
    Qbot* ptrQj;
    
    uint16_t j = 0; // I use this (and increment only when l!=ID) to skip "this" robot in the loop.
    for (uint16_t l=0; l < numQbots ; l++) { // Should execute TijN.size() times, by excluding THIS bot (index ID or "i")
        
        ptrQj = vectorPtrQbots[l];
        
        if (l!=ID && ptrQj->emissionMode) { // otherwise do nothing (own field does not influence the particle, or the other particule is not an emitter)
            
          //  cout << " * Qbot i=" << ID << " : TijN from Qbot j=" << l << " ---> ";
            
            // We update TijN[j] to K, only if we find a value of K such that DijNK/(tN - tK)-speedWave and DijN(K+1)/(tN - t(K+1))-speedWave have an OPPOSITE sign.
            
            // NOTE: tN is equal to tN = N*timeStep, not necessarily equal to the clock used to compute the temporal part of the sinusoidal emitted field (omega*clock).
            // This means that tN - tK is just (N-K)*timeStep (and in integer steps: TijNK = N-K)
            
            int16_t K;// = TijN[j];
            
                // Start searching from origin or from the latest intersection index:
            if (TijN[j]<0) {
                K=0; // no previous intersection => search from t0
               // cout << " [start search from 0]  /  ";
            }
            else K = TijN[j]; // K should be < N-1 by construction, otherwise something went wrong while programming the update function.
                
                // Start searching the new K:
                
                ofVec2f XProjK = ofVec2f(ptrQj->X[K]);
                ofVec2f XProjK1 = ofVec2f(ptrQj->X[K+1]);
                
                // float valK0 = computeLengthDijNK(ptrQj,K)/(N-K)/timeStep-speedWave;
                // float valK1 = computeLengthDijNK(ptrQj,K+1)/(N-K-1)/timeStep-speedWave;
                // or:
                //float valK0 = XProjN.distance(XProjK) / ((N-K)*timeStep) - speedWave;
                //float valK1 = XProjN.distance(XProjK1) / ((N-K-1)*timeStep) - speedWave;
                
                // Or, much faster, use the squares:
                uint16_t NK=N-K;
                float valK0_sQ = XProjN.squareDistance(XProjK) / (NK*NK*sqTimeStep) - sqSpeedWave;
                float valK1_sQ = XProjN.squareDistance(XProjK1) / ((NK-1)*(NK-1)*sqTimeStep) - sqSpeedWave;
                
                NK--;
                while ( ( valK0_sQ*valK1_sQ > 0 ) && ( K++ < N ) ) {// we can discard K=N, because speed is finite - unless both points are at the exact same place!
                    valK0_sQ = valK1_sQ;
                    XProjK = ofVec2f(ptrQj->X[K]);
                    
                    //valK1 = XProjN.distance(XProjK)/(N-K)/timeStep-speedWave;
                    //valK1_sQ = XProjN.squareDistance(XProjK) / ((N-K)*(N-K)*sqTimeStep) - sqSpeedWave;
                    
                    valK1_sQ = XProjN.squareDistance(XProjK) / (NK*NK*sqTimeStep) - sqSpeedWave;
                    NK--;
                }
                
                // Did we find a good K (STRICTLY smaller than N)?
                if (K < N) {
                    TijN[j] = K; // simple method (would be better to search the (non necessarily integer) time TijN[j] by doing a LINEAR INTERPOLATION between ptrQj->X[K] and ptrQj->X[K+1]
                    //ofVec2f XiN = X.back(); // or X[X.size()-1]
                    //ofVec2f XjK = ptrQj->X[K];
                    DijN[j] = ptrQj->X[K] - X[N]; //ofVec2f(ptrQj->X[K] - X[N]); // do we need to use a constructor and then the copy constructor? probably no. The conversion from 3d to 2d is done in the copy contructor directly I guess.
                    
                 //   cout << "OK! K= " << K << "  <  N= " << N << endl;
                }
            else
            {
              //  cout << "Ouch! K= " << K << "  >  N= " << N << endl;
            }
            
            j++;
            
        }
    }
}


float Qbot::computeLengthDijNK(Qbot* ptrQj, uint16_t K) {
    // Computes the distance from the robot j at time K (XjK), and THIS robot
    // (called Xi) at the present time, called N, i.e XiN (or X.back()).
    
    // First, ensure the parameters are not out of range:
    // Attn: numQbots has to be properly mantained (should be equal to vectorPtrQbots.size() here).
    float dijNK = -1; // return -1 if error
    if ( ptrQj->ID < numQbots && K < ptrQj->X.size()) {
        
        ofVec2f XiN = X[N];//X.back(); // or X[X.size()-1]
        ofVec2f XjK = ptrQj->X[K];
        dijNK =  XiN.distance(XjK);
    }
    
    // cout <<  "length dijNK " << dijNK << endl;
    
    return dijNK;
}
