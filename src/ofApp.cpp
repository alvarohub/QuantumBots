#include "ofApp.h"

//------------------------------------------------------------

void ofApp::setup(){
    
    cam3D = false;
    
    ofSetCircleResolution(10);
    ofSetRectMode(OF_RECTMODE_CENTER);
    ofEnableSmoothing();
    
    // this uses depth information for occlusion
    // rather than always drawing things on top of each other
    ofEnableDepthTest();
    
    
    ofBackground(0);
    ofSetFrameRate(60); // if vertical sync is off, we can go a bit fast... this caps the framerate at 60fps.
    
    // Read XML file for setting configuration:
    // TO DO...
    
    updatePeriod = 5; // in ms
    updateTimer = ofGetElapsedTimeMillis();
    
    
    Qbot::begin();
    // intantiation of the array of Qbots:
    //        for (uint16_t i = 0; i < 250; i++) {
    //             Qbot* myQbot=new Qbot(); // (i, initPos);
    //            Qbot::add(myQbot);
    //        }
    
    // GRID of Qbots,
    //    int numSide = 5;
    //    float stepX = 190; //ofGetWidth()/(numSide +1);
    //    float stepY= 200; //ofGetHeight()/(numSide +1);
    //    for (uint16_t i = 1; i < numSide+1; i++) {
    //        for (uint16_t j = 1; j < numSide +1; j++) {
    //            ofVec2f pos = ofVec2f(stepX*(i-numSide/2)+ofRandom(-10,10)+ofGetWidth()/2, stepY*(j-numSide/2)+ofRandom(-10,10)+ofGetHeight()/2);
    //            Qbot* myQbot=new Qbot(pos);
    //            Qbot::add(myQbot);
    //        }
    //    }
    
    
    // Qbot* myQbot=new Qbot( ofVec2f(ofGetWidth()/2,ofGetHeight()/2) , 0, false);
    // Qbot::add(myQbot);
    
    /* barrier of fixed Qbots:
     numSide = 20;
     for (uint16_t i = 1; i < numSide+1; i++) {
     ofVec2f initPos = ofVec2f(i*ofGetWidth()/(numSide +1), 0);
     Qbot* myQbot=new Qbot(initPos, 0, false);
     Qbot::add(myQbot);
     initPos = ofVec2f(i*ofGetWidth()/(numSide +1), ofGetHeight());
     myQbot=new Qbot(initPos, 0, false);
     Qbot::add(myQbot);
     }
     for (uint16_t j = 0; j < numSide+1; j++) {
     ofVec2f initPos = ofVec2f(0,j*ofGetHeight()/(numSide +1));
     Qbot* myQbot=new Qbot(initPos, 0, false);
     Qbot::add(myQbot);
     initPos = ofVec2f(ofGetWidth(),j*ofGetHeight()/(numSide +1));
     myQbot=new Qbot(initPos, 0, false);
     Qbot::add(myQbot);
     }
     */
    
    //cout << "Number Qbots: " << Qbot::getNumParticles() << endl;
    
    Qbot::resetClock();
    
}

//--------------------------------------------------------------

void ofApp::exit() {
    Qbot::deleteAllQbots();
}

//--------------------------------------------------------------
void ofApp::update(){
    // lets update with a real time period (but display with a framerate controlled by the os):
    if ( ofGetElapsedTimeMillis() - updateTimer > updatePeriod) {
        
        Qbot::updateAll();
        
        updateTimer = ofGetElapsedTimeMillis();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    // ofBackground(0, 0, 0);
    
        if (cam3D) { // mouse pressed
            
            // this sets the camera's distance from the object
            //cam.setDistance(1000);
            // cam.setAutoDistance(true);
            cam.disableOrtho();
            
            cam.begin();
            
            ofScale(1, -1, 1); // flip the y axis and zoom in a bit
            ofTranslate(-ofGetWidth()/2,-ofGetHeight()/2,0);
    
            Qbot::drawAll();
    
            cam.end();
        
        } else {
            ofSetupScreenOrtho(ofGetWidth(),ofGetHeight(),
                               -1E10, 1E4);
            Qbot::drawAll();

            
            // or:
            //cam.reset();
            //cam.enableOrtho();
            //cam.begin();
            //Qbot::drawAll();
            //cam.end();
        }
    
    
    // Qbot::printStates();
    ofSetColor(255, 90, 60);
    ofFill();
    //ofDrawBitmapString("Time: "+ofToString(Qbot::getClock()), 10, 10);
    ofDrawBitmapString("Iteration: "+ofToString(Qbot::getIteration()), 10, 10);
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    switch(key) {
            
            // * Add Qbots (four flavours here):
            // Reminder: contructor with full parameters:
            //    Qbot(ofVec2f _pos, float _phase, bool _emission, bool _motile = true)
        case 'a': // emitter and mobile qbot
            Qbot::addQbot(mouseX, mouseY, 0, true, true);
            break;
            
        case 's': // non-emitter and mobile qbot:
            Qbot::addQbot(mouseX, mouseY, 0, false, true);
            break;
            
        case 'z': // emitter and fixed qbot:
            Qbot::addQbot(mouseX, mouseY, 0, true, false);
            break;
            
        case 'x': // non-emitter and fixed qbot:
            
            // discretized grid?:
            // Qbot::addQbot(20*round(mouseX/20), 20*round(mouseY/20), 0, false, false);
            Qbot::addQbot(mouseX, mouseY, 0, false, false);
            break;
            
            // 3d interaction mode:
        case 'c':
            if(cam.getMouseInputEnabled())
                cam.disableMouseInput();
            else
                cam.enableMouseInput();
            break;
            
            // Drawing modes:
        case '1':
            Qbot::toggleModeBots(); // show/hide bots
            break;
        case '2':
            Qbot::toggleModeGradient(); // show/hide gradient
            break;
        case '3':
            Qbot::toggleModeTrajectory(); // show/hide trajectory
            break;
            
        case '0':
            Qbot::toggleModeLines(); // show/hide DijN
            break;
            
            // Global control on the swarm of Qbots:
        case ' ':
            Qbot::toggleMotionStateAll();
            break;
            
            // Global control on evolution:
        case OF_KEY_RETURN:
            Qbot::toggleClockState();
            //stopClock();
            break;
            
            // Actions affecting Qbots in the "influence" area of the mouse pointer:
            
            // * Fix the Qbot (clock contiues to run):
        case 'm':
            Qbot::setMotionStateInRange(mouseX, mouseY, 10, false);
            break;
            // * Release the Qbot:
        case 'M':
            Qbot::setMotionStateInRange(mouseX, mouseY, 10, true);
            break;
            
            
            
            // * Delete the Qbot in range:
        case 'd':
            Qbot::deleteQbotInRange(mouseX, mouseY, 10);
            break;
            
        case OF_KEY_BACKSPACE:
            Qbot::deleteAllQbots();
            break;
            
            
            // Create "square patch field checker, i.e. a small grid of fixed robots (translated in real time by the mouse), that DO NOT EMIT ANY FIELD.
        case 'p':
        {
            uint8_t numSideX = 10;
            uint16_t sizePatch = 200; // in pixels (or proportion of the bounding rectangle)
            float dHex = 4.0/3*sizePatch/numSideX;
            float hHex = 1.0*dHex*sqrt(3)/2;
            uint8_t numSideY = 2.0*sizePatch/hHex;
            for (uint16_t i = 0; i < numSideY ; i++) {
                for (uint16_t j = 0; j < numSideX / 2; j++) {
                    ofVec2f initPos =
                    ofVec2f(1.5*dHex*j+1.5*dHex*(i%2)/2 + mouseX - sizePatch/2,
                            0.5*hHex * i  + mouseY - sizePatch/2);
                    Qbot* myQbot=new Qbot(initPos, 0, false, false); // non emitting and fixed for now
                    myQbot->setRadius(0.3*dHex);
                    myQbot->setShape(Qbot::SHAPE_HEXAGON);
                    myQbot->setEmissionMode(false);
                    Qbot::add(myQbot);
                }
            }
            // YET TODO: move the grid in real time (make a "translate method" for the robots. Also, delete this grid properly when needed.
        }
            break;
            
        case '@': // HEXAGONAL grid covering the whole area:
        {
            uint8_t numSideX = 40;
            uint16_t sizePatch = ofGetWidth(); // in pixels (or proportion of the bounding rectangle)
            float dHex = 4.0/3*sizePatch/numSideX;
            float hHex = 1.0*dHex*sqrt(3)/2;
            uint8_t numSideY = 2.0*sizePatch/hHex;
            for (uint16_t i = 0; i < numSideY+2 ; i++) {
                for (uint16_t j = 0; j < numSideX / 2 + 1; j++) {
                    ofVec2f initPos =
                    ofVec2f(1.5*dHex*j+1.5*dHex*(i%2)/2 + ofGetWidth()/2 - sizePatch/2 + dHex/4,
                            0.5*hHex * i  + ofGetHeight()/2 - sizePatch/2 ); //+ hHex/2
                    
                    if (initPos.x <ofGetWidth() + dHex/2 && initPos.y < ofGetHeight() + hHex/2 ) {
                        Qbot* myQbot=new Qbot(initPos, 0, false, false); // non emitting and fixed for now
                        myQbot->setRadius(0.5*dHex-2); //0.3*dHex);
                        myQbot->setShape(Qbot::SHAPE_HEXAGON);
                        myQbot->setEmissionMode(false);
                        Qbot::add(myQbot);
                    }
                }
            }
            // YET TODO: move the grid in real time (make a "translate method" for the robots. Also, delete this grid properly when needed.
        }
            break;
            
        case '[': // square grid covering the whole area:
        {
            uint8_t numSide = 30;
            uint16_t sizePatch = ofGetWidth(); // in pixels (or proportion of the bounding rectangle)
            float dSquare = 1.0*sizePatch/numSide;
            for (uint16_t i = 0; i < numSide+1 ; i++) {
                for (uint16_t j = 0; j < numSide+1; j++) {
                    ofVec2f initPos = ofVec2f(j*dSquare +dSquare/2 + ofGetWidth()/2 -sizePatch/2, i*dSquare +dSquare/2 + ofGetHeight()/2 - sizePatch/2);
                   
                     Qbot* myQbot=new Qbot(initPos, 0, false, false);// non emitting and fixed
                        myQbot->setRadius(dSquare-2); //0.3*dHex);
                        myQbot->setShape(Qbot::SHAPE_SQUARE);
                        myQbot->setEmissionMode(false);
                        Qbot::add(myQbot);
                }
            }
            // YET TODO: move the grid in real time (make a "translate method" for the robots. Also, delete this grid properly when needed.
        }
            break;
            
            
        case 'f':
            ofToggleFullscreen();
            break;
            
    }
    
    // TO DO:
    //  * method to make a bot to toggle its emission mode;
    //  * ... and of course, many method to change parameters of emission or sensing (periodic sensing, random period sensing, synchronous sensing or not, different sampling rate, etc).
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    //    switch (button) {
    //        case OF_MOUSE_BUTTON_1:
    //            Qbot::addQbot(x, y);
    //            break;
    //        case OF_MOUSE_BUTTON_2:
    //            // Delete bot that is closer to the mouse cursor, and inside the range of influence of it:
    //            // ... TO DO
    //            break;
    //    }
    cam3D = true;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    cam3D = false;
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
