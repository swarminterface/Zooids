#pragma once
#include "ofMain.h"
#include "ofxBox2d.h"
#include "ofxGui.h"
#include "ofxNetwork.h"
#include "RobotManager.h"

// -------------------------------------------------

class testApp : public ofBaseApp {
	
public:
	
	void setup();
	void update();
	void draw();
	
    
    void scaleToRobotWorld();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void resized(int w, int h);
    
    int getPt( int n1 , int n2 , float perc );
    

private:
    RobotManager myRobots;
    vector<RobotState>* robots;
    bool isFirstTime;
    
    int nbControlPts;
    float numRobotsInCurve;
    vector <ofVec2f> curvePoints;
    
};

