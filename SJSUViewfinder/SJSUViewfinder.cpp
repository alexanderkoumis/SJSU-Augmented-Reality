//
//  SJSUViewfinder.cpp
//  SJSUViewfinder
//
//  Created by Henry Tran on 9/9/13.
//  Copyright (c) 2013 Henry Tran. All rights reserved.
//

#include "SJSUViewfinder.h"
#include "Globals.h"

using namespace irr;
using namespace irr::core;
using namespace irr::gui;
using namespace irr::video;
using namespace irr::scene;
using namespace irr::io;

SJSUViewfinder::SJSUViewfinder(irr::IrrlichtDevice *device)
: device(device) {
    
	driver = device->getVideoDriver();
	smgr = device->getSceneManager();
	guienv = device->getGUIEnvironment();
    filesystem = device->getFileSystem();
    
}

void SJSUViewfinder::init() {
    // load up the shiba inu picture
    shiba = loadImage("media/shiba_profile.png"); // we use our custom "loadImage" function to load it from the iphone.
    
    homography.train(shiba);
    
#ifdef _IRR_IPHONE_PLATFORM_
    this->viewfinderLock = [[NSLock alloc] init];
    jump = 0;
#endif
    
    //create the webcam texture to draw in irrlicht
    irrFrame = driver->addTexture(dimension2d<u32>(512, 512), "cvFrame", video::ECOLOR_FORMAT::ECF_A8R8G8B8);
    cvImage = driver->addTexture(dimension2d<u32>(256, 256), "cvImage", video::ECOLOR_FORMAT::ECF_A8R8G8B8);
    
	// create a camera at 0, 0, 0, looking at position 0, 0, 1 (up)
    camera = smgr->addCameraSceneNode(0, vector3df(0,0,0), vector3df(0,0,1));
    
    // calculate the fov
    // atan(cy/fx)*2
    camera->setFOV(atan(getCamIntrinsic()(1,2)/getCamIntrinsic()(0,0))*2);
    
	// load up the mesh
    IAnimatedMesh* mesh = smgr->getMesh("media/earth.x");
    if (!mesh) {
        // idk do something, work out an error system.
    }
    
	// add a "node" to the scene using that mesh
    node = smgr->addMeshSceneNode( mesh );
    
	// if the "node" has been successfully created...
	if (node)
	{
		// tell the program that the "node" does not receive any lighting (it's completely bright)
		// the default animation is to stand
		// the material that the mesh uses is media/sydney.bmp
		// the "node" will spawn at 0, 0, 60.
		node->setMaterialFlag(EMF_LIGHTING, false);
		//node->setMD2Animation(scene::EMAT_STAND);
		node->setMaterialTexture( 0, driver->getTexture("media/earth.png") );
        node->setPosition(vector3df(0, 0, 500));
        node->setScale(vector3df(10, 10, 10));

	}
	else
	{
		// error, can't load the node
	}
    
	scene::ILightSceneNode* light1 =
    smgr->addLightSceneNode(0, core::vector3df(0,0,0),
                            video::SColorf(0.5f, 1.0f, 0.5f, 0.0f), 800.0f);
    
	light1->setDebugDataVisible ( scene::EDS_BBOX );

}

void SJSUViewfinder::run() {
    
	// "frame" is the cv::Mat that we work with.
	// we'll set it later on in the code
	// but for now we're using the last frame captured (just in case we can't use the current frame.)
    cv::Mat frame = cvLastFrame;
    
#ifdef _IRR_IPHONE_PLATFORM_
    // Since the iPhone is multithreaded, data can be easily corruptable.
    // We use a lock to disable other threads for running certain codes.
    if (jump && [this->viewfinderLock tryLock])
#endif
    {
		// check if the current frame is not empty
        if (!cvFrame.empty())
        {
			// we want to work with a small image
            cv::resize(cvFrame, cvFrame, cv::Size(cvFrame.cols/2, cvFrame.rows/2));
                        
            cv::Mat cloned = cvFrame.clone();
            cv::undistort(cloned, cvFrame, getCamIntrinsic(), cv::Mat());
			
			// store the smaller image into the "working" frame, as well as the last frame.
            frame = cvFrame.clone();
            cvLastFrame = cvFrame.clone();
            
            homography.analyze(frame);
            homography.extract(H, corners, &irrPos, &irrAng, 0);
        }
#ifdef _IRR_IPHONE_PLATFORM_
        jump = !jump;
        [this->viewfinderLock unlock];
#endif
    }
    
	// now we start drawing here.
    driver->beginScene(true, true, SColor(0,100,100,100));
    
	// this will let us draw in 2D.
    driver->enableMaterial2D();
    
    if (!frame.empty()) {
        drawCvImage(frame, vector2di(0, 0), vector2di(frame.cols*2.4, frame.rows*2.4));
        node->setPosition(irrPos* 25);
        node->setRotation(irrAng);
        
        for (int i = 0; i < 4; i++)
            driver->draw2DLine(vector2di((float)corners[i].X*(1.2*2), (float)corners[i].Y*(1.2*2)),
                               vector2di((float)corners[(i+1)%4].X*(1.2*2), (float)corners[(i+1)%4].Y*(1.2*2)));
    }
    
	// we're all done with the 2D stuff.
    driver->enableMaterial2D(false);
    
	// now we draw the rest (the stuff that isn't opencv.
    smgr->drawAll(); // scenemanager holds all the 3D models
    guienv->drawAll(); // guienv holds all 2D panels (buttons, windows, etc)
    
	// and now we're all done.
    driver->endScene();
    
	// this calculates the FPS.
    IGUIElement *stat = device->getGUIEnvironment()->
    getRootGUIElement()->getElementFromId ( 100 );
    if ( stat )
    {
        stringw str = L"FPS: ";
        str += (s32)device->getVideoDriver()->getFPS();
        
        stat->setText ( str.c_str() );
    }
    
}

/**
 * Process camera frame. (Not synchronized to "run", will grab frames whenever there is one available.)
 */
void SJSUViewfinder::processFrame(cv::Mat &frame) {
    
#ifdef _IRR_IPHONE_PLATFORM_
    if (!jump && [this->viewfinderLock tryLock])
#endif
    {

		
#ifndef _IRR_IPHONE_PLATFORM_
		// this is to fabricate proper iphone size camera
		if (!frame.empty()) {
            cv::resize(frame, frame, cv::Size(648, (double)frame.rows/frame.cols*648));
			cv::Mat mat = frame.clone();
			cv::resize(mat, mat, cv::Size(648, 1152));
			cv::rectangle(mat, cv::Rect(0, 0, 648, 1152), cv::Scalar(0, 0, 0), -1);
			frame(cv::Rect(0, 0, 648, (double)frame.rows/frame.cols*648)).copyTo(mat(cv::Rect(0, 0, 648, (double)frame.rows/frame.cols*648)));
			frame = mat;
		}
#endif

        cvFrame = frame.clone();
        
#ifdef _IRR_IPHONE_PLATFORM_
        jump = !jump;
        [this->viewfinderLock unlock];
#endif
    }
    
}

/*
 * We use irrlicht's filesystem interface to load an image into opencv.
 * This is because the iPhone has some wonky functions for loading files
 * and I'd rather use irrlicht commands since they work for all platforms.
 */
cv::Mat SJSUViewfinder::loadImage(irr::core::stringc filename) {
    
    IReadFile *file = filesystem->createAndOpenFile("media/shiba_profile.png");
    char *buffer = new char[file->getSize()];
    cv::Mat input(file->getSize(), 1, CV_8UC1, buffer, cv::Mat::AUTO_STEP);
    file->read(buffer, file->getSize());
    cv::Mat img = cv::imdecode(input, 1);
    file->drop();
    delete buffer;
    
    return img;
    
}

void SJSUViewfinder::drawCvImage(cv::Mat image, vector2di pos) {
    drawCvImage(image, recti(pos.X, pos.Y, pos.X + image.cols, pos.Y + image.rows));
}

void SJSUViewfinder::drawCvImage(cv::Mat image, vector2di pos, vector2di size) {
    drawCvImage(image, recti(pos.X, pos.Y, pos.X + size.X, pos.Y + size.Y));
}

void SJSUViewfinder::drawCvImage(cv::Mat image, recti rect) {
    
    cv::Mat imageClone = image.clone();
    cv::resize(imageClone, imageClone, cv::Size(256, 256));
    
#ifndef _IRR_IPHONE_PLATFORM_
    // we need to convert the color so that Irrlicht can understand it.
    // iPhone already converts the color by default, so this is when it's not on an iPhone.
    cv::cvtColor(imageClone, imageClone, cv::COLOR_RGB2RGBA);
#endif
    
    // copy the pixel data from the cv::Mat to an empty irrlicht texture.
    char *data = (char *)cvImage->lock();
    memcpy(data, imageClone.data, imageClone.total()*imageClone.elemSize());
    cvImage->unlock();
    
    driver->draw2DImage(cvImage, rect, recti(0, 0, 256, 256));
    
}
