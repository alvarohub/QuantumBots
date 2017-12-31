#pragma once

#include "ofMain.h"
#include "QbotClass.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
        float updateTimer, updatePeriod;
    
        //Added "core" methods & objects:
        // exit() : called at the moment before the app is terminated. This is useful for
        //doing cleanup stuff or making sure files are saved before the app terminates
        void exit(); // this will be necessary to properly clear the heap on the vector of Qbots.
    
        ofEasyCam cam; // add mouse controls for camera movement
    bool cam3D;
		
};
