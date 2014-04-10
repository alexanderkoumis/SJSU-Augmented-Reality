//
//  Homography.h
//  SJSUViewfinder
//
//  Created by Henry Tran on 9/16/13.
//  Copyright (c) 2013 Henry Tran. All rights reserved.
//

#ifndef __SJSUViewfinder__Homography__
#define __SJSUViewfinder__Homography__

#include <iostream>

#include "irrlicht.h"

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"

/**
 * Homography class is used to find a flat image in a scene and to extract valuable data from the scene.
 *
 **/

class Homography {
public:
    Homography();
    
    // train the homographier using a static image.
    void train(cv::Mat& object);
    
    // analyze a scene to find our object inside it.
    void analyze(cv::Mat& scene);
    
    // extract the information we found from analyzing the scene.
    // returns true if successful
    bool extract(cv::Mat &h, irr::core::vector2di* corners = NULL, irr::core::vector3df *position = NULL, irr::core::vector3df *angles = NULL, int refine = 0);
    
private:
    void computeKeypointAndDescriptors(cv::Mat inputImage, std::vector<cv::KeyPoint>& kp, cv::Mat& descriptors);
    
    cv::SURF surf;
    cv::SIFT sift;
    cv::ORB orb;
    cv::FREAK freak;
    cv::BFMatcher bruteForcer;
    
    // store all surf keypoint matches here
    std::vector< cv::DMatch > matches;
    
    // descriptors and keypoints
    cv::Mat objectDescriptors;
    std::vector<cv::KeyPoint> objectKeyPoints;
    cv::Mat sceneDescriptors;
    std::vector<cv::KeyPoint> sceneKeyPoints;
    
    // size of the object image
    cv::Size objectSize;
    
    // scene
    cv::Mat sceneCopy;
    
    // intrinsic camera data (should be different for every camera)
    cv::Matx33f camIntrinsic;
};

#endif /* defined(__SJSUViewfinder__Homography__) */
