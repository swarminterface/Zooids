#include "ofApp.h"

#define dimensionX 0.8128f
#define dimensionY 0.508f
#define NB_CONTROL_PTS 3
#define robotSpacing 0.02f // initial spacing between robots in inactive/waiting mode
#define robotSize 0.027f // approximate robot radius

//#define NB_ROBOTS 15
//--------------------------------------------------------------
void testApp::setup() {
    ofSetFrameRate(60);
    myRobots.initialize();
    isFirstTime = true;
    
    for(int i=0; i<NB_CONTROL_PTS;i++){
        curvePoints.push_back(ofVec2f(dimensionX*(float)i/(float)NB_CONTROL_PTS, dimensionY/(float)NB_CONTROL_PTS));
        if(i<NB_CONTROL_PTS-1){
            curvePoints.push_back(ofVec2f(dimensionX*(float)i/(float)NB_CONTROL_PTS+0.05f, dimensionY/(float)NB_CONTROL_PTS+0.05f));
        }
    }
    
    nbControlPts = 3;
    numRobotsInCurve = 3.0f; // one more than each segment
    
    for(int j=0;j<2*(NB_CONTROL_PTS-1);j+=2){
        // first each part of curve (between two points)
        for( float i = (1.000f/numRobotsInCurve) ; i < 1 ; i += (1.000f/numRobotsInCurve) ){
            // The Green Line
            int xa = getPt( curvePoints.at(j).x , curvePoints.at(j+1).x , i );
            int ya = getPt( curvePoints.at(j).y , curvePoints.at(j+1).y , i );
            int xb = getPt( curvePoints.at(j+1).x , curvePoints.at(j+2).x , i );
            int yb = getPt( curvePoints.at(j+1).y , curvePoints.at(j+2).y , i );
            
            // The Black Dot
            int x = getPt( xa , xb , i );
            int y = getPt( ya , yb , i );
            
            curvePoints.push_back(ofVec2f(x, y));
        }
    }
}

//--------------------------------------------------------------
void testApp::update() {
    
    ////////////////////////////////////////////////////////////////////////////////////
    int counter= 5;
    // first part of curve
    for( float i = (1.000f/numRobotsInCurve) ; i < 1 ; i += (1.000f/numRobotsInCurve) )
    {
        
        // The Green Line
        int xa = getPt( curvePoints.at(0).x , curvePoints.at(1).x , i );
        int ya = getPt( curvePoints.at(0).y , curvePoints.at(1).y , i );
        int xb = getPt( curvePoints.at(1).x , curvePoints.at(2).x , i );
        int yb = getPt( curvePoints.at(1).y , curvePoints.at(2).y , i );
        
        // The Black Dot
        int x = getPt( xa , xb , i );
        int y = getPt( ya , yb , i );
        
        
        ofVec2f position = ofVec2f(x,y);
        curvePoints.at(counter).set(position);
        
        counter++;
    }
    
    //second point of curve
    for( float i = (1.000f/numRobotsInCurve) ; i < 1 ; i += (1.000f/numRobotsInCurve) )
    {
        
        // The Green Line
        int xa = getPt( curvePoints.at(2).x , curvePoints.at(3).x , i );
        int ya = getPt( curvePoints.at(2).y , curvePoints.at(3).y , i );
        int xb = getPt( curvePoints.at(3).x , curvePoints.at(4).x , i );
        int yb = getPt( curvePoints.at(3).y , curvePoints.at(4).y , i );
        
        // The Black Dot
        int x = getPt( xa , xb , i );
        int y = getPt( ya , yb , i );
        
        ofVec2f position = ofVec2f(x,y);
        curvePoints.at(counter).set(position);
        
        counter++;
    }
    
    
    if (myRobots.receiveRobotInformation())
    {
        //do things with the new robot positions
        robots = myRobots.getRobots();
        
        for(int i =0; i <robots->size(); i++){
            
            if(i<curvePoints.size()){
                
                if( i < 5){
                    if(robots->at(i).getTouch()>0){
                        ofVec2f position = robots->at(i).getPosition();
                        ofVec2f newPosition = ofVec2f(ofMap(position.x, dimensionX, 0, 0, ofGetWidth() ), ofMap(position.y, dimensionY, 0, 0, ofGetHeight() ));
                        
                        curvePoints.at(i).set(newPosition);
                        
                        ofVec2f position2 = curvePoints.at(i);
                        ofVec2f positionTransformed = ofVec2f( ofMap(position2.x, 0, ofGetWidth(), dimensionX, 0 ),  ofMap(position2.y, 0, ofGetHeight(), dimensionY, 0 ) );
                        myRobots.updateRobot(i, positionTransformed, ofColor::red);
                        
                    } else{
                        //new bit
                        ofVec2f position = robots->at(i).getPosition();
                        ofVec2f newPosition = ofVec2f(ofMap(position.x, dimensionX, 0, 0, ofGetWidth()), ofMap(position.y, dimensionY, 0, 0, ofGetHeight()));
                        
                        curvePoints.at(i).set(newPosition);
                        //end new bit
                        
                        ofVec2f position2 = curvePoints.at(i);
                        ofVec2f positionTransformed = ofVec2f( ofMap(position2.x, 0, ofGetWidth(), dimensionX, 0 ),  ofMap(position2.y, 0, ofGetHeight(), dimensionY, 0 ) );
                        myRobots.updateRobot(i, positionTransformed, ofColor::red);
                        
                    }
                }
                
                else if (i < (5 + numRobotsInCurve*2)){
                    
                    ofVec2f position2 = curvePoints.at(i);
                    ofVec2f positionTransformed = ofVec2f( ofMap(position2.x, 0, ofGetWidth(), dimensionX, 0 ),  ofMap(position2.y, 0, ofGetHeight(), dimensionY, 0 ) );
                    myRobots.updateRobot(i, positionTransformed, ofColor::blue);
                    
                }
            }
            else{
                
                float x, y;
                
                int k = i - 15  ;
                float s = robotSize + robotSpacing;
                
                y = dimensionY;
                if (k%2 == 1) {
                    // center/right
                    x = dimensionX / 2 + s*(k+1)/2;
                } else {
                    // left
                    x = dimensionX / 2 - s*(k)/2;
                }
                
                myRobots.updateRobot(i, ofVec2f(x,y), ofColor::black);
            }
            
        }
        
        
        
        //don't forget to send the updates
        myRobots.sendRobotUpdates();
        
    }
    
}


//--------------------------------------------------------------
void testApp::draw() {
    
}

//--------------------------------------------------------------
void testApp::keyPressed(int key) {
    
}

//--------------------------------------------------------------
void testApp::keyReleased(int key) {
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ) {
    
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button) {
}

//--------------------------------------------------------------
void testApp::resized(int w, int h){
}

int testApp::getPt( int n1 , int n2 , float perc )
{
    int diff = n2 - n1;
    
    return n1 + ( diff * perc );
}


