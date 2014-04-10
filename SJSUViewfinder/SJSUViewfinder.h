//
//  SJSUViewfinder.h
//  SJSUViewfinder
//
//  Created by Henry Tran on 9/9/13.
//  Copyright (c) 2013 Henry Tran. All rights reserved.
//

#ifndef __SJSUViewfinder__SJSUViewfinder__
#define __SJSUViewfinder__SJSUViewfinder__

#include <iostream>
#include "irrlicht.h"

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "Homography.h"

/*
 
 Should be able to run regardless of platform.
 
 */

class SJSUViewfinder {
    
public:
    SJSUViewfinder(irr::IrrlichtDevice *device);
    
    void init();
    void run();
    void processFrame(cv::Mat &frame);
    
    cv::Mat loadImage(irr::core::stringc filename);
    void drawCvImage(cv::Mat image, irr::core::vector2di pos);
    void drawCvImage(cv::Mat image, irr::core::vector2di pos, irr::core::vector2di size);
    void drawCvImage(cv::Mat image, irr::core::recti rect);
    
private:
    irr::IrrlichtDevice*        device;
    irr::video::IVideoDriver*   driver;
    irr::scene::ISceneManager*  smgr;
    irr::gui::IGUIEnvironment*  guienv;
    irr::io::IFileSystem*       filesystem;
    
    cv::Mat shiba;
    cv::Mat cvFrame;
    cv::Mat cvLastFrame;
    irr::video::ITexture *irrFrame;
    irr::video::ITexture *cvImage;
    
    irr::scene::ISceneNode* node;
    irr::scene::IAnimatedMeshSceneNode* node2;
    irr::scene::ICameraSceneNode* camera;
    
    Homography homography;
    
    cv::Mat H;
    irr::core::vector3df irrPos;
    irr::core::vector3df irrAng;
    irr::core::vector2di corners[4];
    
// opencv camera processing is multithreaded, so I lock each thread to make it seem like one single thread
// very very slow, I should plan on removing this eventually
// on the computer, it isn't multithreaded, so it's not used for the computer.
#ifdef _IRR_IPHONE_PLATFORM_
    bool jump;
    NSLock *viewfinderLock;
#endif
    
};

#endif /* defined(__SJSUViewfinder__SJSUViewfinder__) */
