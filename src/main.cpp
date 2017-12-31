#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
    // Setup the GL context:
    //  * this kicks off the running of my app
    // can be OF_WINDOW or OF_FULLSCREEN
    // pass in width and height too:
	
    ofSetupOpenGL(800,800,OF_WINDOW);
    //ofSetupOpenGL(600,600,OF_FULLSCREEN);
	
	ofRunApp(new ofApp());

}
