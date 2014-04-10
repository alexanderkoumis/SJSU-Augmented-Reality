//
//  Homography.cpp
//  SJSUViewfinder
//
//  Created by Henry Tran on 9/16/13.
//  Copyright (c) 2013 Henry Tran. All rights reserved.
//

#include "Homography.h"
#include "Globals.h"

Homography::Homography() {
    objectSize = cv::Size(0, 0);
    surf = cv::SURF(500);
    sift = cv::SIFT(500);
    orb = cv::ORB();
    //freak = cv::FREAK();
    bruteForcer = cv::BFMatcher(cv::NORM_L2, false);
}

void Homography::train(cv::Mat& object) {
    cv::Mat objectCopy;
    cv::cvtColor(object, objectCopy, cv::COLOR_BGR2GRAY);
    objectSize = cv::Size(object.cols, object.rows);
    
    // compute the surf keypoints and descriptors
    computeKeypointAndDescriptors(objectCopy, objectKeyPoints, objectDescriptors);
}

void Homography::analyze(cv::Mat& scene) {
    sceneCopy = scene.clone();
    if (sceneCopy.channels() > 1) {
        cv::cvtColor(sceneCopy, sceneCopy, cv::COLOR_BGR2GRAY);
    }
    
    sceneKeyPoints.clear();
    computeKeypointAndDescriptors(sceneCopy, sceneKeyPoints, sceneDescriptors);
    
    // make sure the scene image contains something and that we have descriptors
    if (!sceneCopy.empty() && objectDescriptors.total() > 3 && sceneDescriptors.total() > 3)
    {
        
        // find matches using bruteforcer
        std::vector< cv::DMatch > bfMatches;
        bruteForcer.match( objectDescriptors, sceneDescriptors, bfMatches );
        
        // clear the matches that we had from last time
        matches.clear();
        
        // make sure we have 3 matches to work with. 3 is good because it is all that is required to make a plane
        if ( bfMatches.size() > 3 )
        {
            //
            std::vector< cv::DMatch >::iterator i = bfMatches.begin();
            cv::DMatch last = *i;
            for (i++; i != bfMatches.end(); i++)
            {
                if (last.distance < 0.6*(*i).distance)
                    matches.push_back(last);
                
                last = *i;
            }
        }
        
    }
}

bool Homography::extract(cv::Mat &H, irr::core::vector2di *corners, irr::core::vector3df *position, irr::core::vector3df *angles, int refine) {
    
    if ( matches.size() > 3 && objectKeyPoints.size() < sceneKeyPoints.size() )
    {
        std::vector<cv::Point2f> objectPoints;
        std::vector<cv::Point2f> scenePoints;
        
        // get the keypoints from the goodmatches
        for( int i = 0; i < matches.size(); i++ )
        {
            objectPoints.push_back( objectKeyPoints[ matches[i].queryIdx ].pt );
            scenePoints.push_back( sceneKeyPoints[ matches[i].trainIdx ].pt );
        }
        
        // find the homography of the keypoints.
        H = cv::findHomography( objectPoints, scenePoints, CV_RANSAC );
        
        std::vector<cv::Point2f> obj_corners(4);;
        std::vector<cv::Point2f> scene_corners(4);
        obj_corners[0] = cvPoint( 0,          0 );
        obj_corners[1] = cvPoint( objectSize.width, 0 );
        obj_corners[2] = cvPoint( objectSize.width, objectSize.height );
        obj_corners[3] = cvPoint( 0,          objectSize.height );
        
        // get the 2D points for the homography corners
        perspectiveTransform( obj_corners, scene_corners, H);
        
        if (refine > 0) {
            cv::Mat sceneCopyCopy = sceneCopy.clone();
            cv::warpPerspective(sceneCopy, sceneCopy, H, objectSize, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
            cv::Mat H2;
            analyze(sceneCopy);
            if (extract(H2, NULL, NULL, NULL, refine - 1)) {
                H *= H2;
                perspectiveTransform( obj_corners, scene_corners, H);
            }
        }
        
        // give the caller the corners of the 2D plane
        if (corners != NULL)
            for (int i = 0; i < 4; i++) {
                corners[i] = irr::core::vector2di(scene_corners[i].x, scene_corners[i].y);
            }
        
        // init the rotation and translation vectors
        cv::Mat raux(3, 1, CV_64F), taux(3, 1, CV_64F);
        
        // calculating 3D points
        float maxSize = std::max(objectSize.width, objectSize.height);
        float unitW = objectSize.width / maxSize;
        float unitH = objectSize.height / maxSize;
        
        // get the rotation and translation vectors
        std::vector<cv::Point3f> scene_3d_corners(4);
        scene_3d_corners[0] = cv::Point3f(-unitW, -unitH, 0);
        scene_3d_corners[1] = cv::Point3f( unitW, -unitH, 0);
        scene_3d_corners[2] = cv::Point3f( unitW,  unitH, 0);
        scene_3d_corners[3] = cv::Point3f(-unitW,  unitH, 0);
        cv::solvePnP(scene_3d_corners, scene_corners, getCamIntrinsic(), cv::Mat(), raux, taux);
        
        // give the caller the 3D plane position and angle
        if (position != NULL)
            position->set(taux.at<double>(0, 0), -taux.at<double>(1, 0), taux.at<double>(2, 0));
        if (angles != NULL)
            angles->set(-raux.at<double>(0, 0) * irr::core::RADTODEG, raux.at<double>(1, 0) * irr::core::RADTODEG, -raux.at<double>(2, 0) * irr::core::RADTODEG);
        
        return true;
    }
    
    return false;
}


void Homography::computeKeypointAndDescriptors(cv::Mat inputImage, std::vector<cv::KeyPoint>& kp, cv::Mat& descriptors)
{
    cv::FernClassifier
    //cv::FAST(inputImage, kp, 100);
    //orb.detect(inputImage, kp);
    //surf.compute(inputImage, kp, descriptors);
    //orb(inputImage, cv::Mat(), kp, descriptors);
    sift(inputImage, cv::Mat(), kp, descriptors);
    //freak.compute(inputImage, kp, descriptors);
    //cv::FAST(inputImage, kp, 100);
    //surf.compute(inputImage, kp, descriptors);
    //surf(inputImage, cv::Mat(), kp, descriptors);
    
}

