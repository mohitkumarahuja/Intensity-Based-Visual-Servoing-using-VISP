#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>
#include <visp/vpFeatureLuminance.h>
#include <visp/vpParseArgv.h>
#include <visp/vpImageSimulator.h>
#include <stdlib.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageConvert.h>
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>
#include <visp/vpImageSimulator.h>

#define  Z 2.0

int main(int argc, const char ** argv){

    //Simulation of the 3D scene
    cv::Mat imageCV;
    imageCV = cv::imread("/home/mscv/Desktop/VisualServoingPractical3/VirtualVisualServoingPractical_Intensity/Marilyn.png",0);

    //    cv::imshow("aaa",imageCV);
    //    cv::waitKey(0);

    vpImage<unsigned char> Itexture(1188,781) ;
    vpImageConvert::convert(imageCV, Itexture);

    //       /* vpDisplayX d;
    //        d.init(Itexture);
    //        vpDisplay::display(Itexture);
    //        vpDisplay::flush(Itexture);
    //        vpDisplay::getClick(Itexture);




    vpColVector X[4];
    for (int i = 0; i < 4; i++) X[i].resize(3);
    // Top left corner
    X[0][0] = -0.3;
    X[0][1] = -0.215;
    X[0][2] = 0;
    // Top right corner
    X[1][0] = 0.3;
    X[1][1] = -0.215;
    X[1][2] = 0;
    // Bottom right corner
    X[2][0] = 0.3;
    X[2][1] = 0.215;
    X[2][2] = 0;
    //Bottom left corner
    X[3][0] = -0.3;
    X[3][1] = 0.215;
    X[3][2] = 0;

    //Simulation of the camera
    vpImageSimulator camera;
    camera.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION) ;
    camera.init(Itexture, X);

    //Desired and Initial camera poses
    vpHomogeneousMatrix cdMo(0,0,2,vpMath::rad(0),vpMath::rad(0),vpMath::rad(0));
    vpHomogeneousMatrix cMo(0,0,2,vpMath::rad(15),vpMath::rad(-5),vpMath::rad(10));

    //Desired and current
    vpImage<unsigned char> I(240,320);
    vpImage<unsigned char> Id(240,320);
    vpImage<unsigned char> Idiff(240,320);

    //Displays (current, desired and differences)
    vpDisplayX dI, dId, dIdiff;
    dId.init(Id, 20, 10, "Desired image") ;
    dI.init(I, 40+(int)I.getWidth(), 10, "Current image") ;
    dIdiff.init(Idiff, 40+(int)I.getWidth()*2, 10, "Difference image") ;

    //Camera Intrinsic parameters
    vpCameraParameters cam(870, 870, 160, 120);

    // Simulation of the robot (here a simulated free flying camera)
    vpRobotCamera robot ;
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;


    //**********************************************************************************
    //*******************************TODO by Students***********************************
    //**********************************************************************************
    //set robot position
    robot.setPosition(cMo);

    camera.setCameraPosition(cdMo);
    camera.getImage(Id, cam); // acquire desired image
    vpDisplay::display(Id);
    vpDisplay::flush(Id);

    camera.setCameraPosition(cMo);
    camera.getImage(I, cam); // acquire current image
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpImageTools::imageDifference(Id, I, Idiff); //diff
    vpDisplay::display(Idiff);
    vpDisplay::flush(Idiff);


    vpFeatureLuminance vpFeatLumCur, vpFeatLumDes;
    // current image features
    vpFeatLumCur.init(I.getRows(), I.getCols(), Z);
    vpFeatLumCur.setCameraParameters(cam);
    vpFeatLumCur.buildFrom(I);

    // desired image features
    vpFeatLumDes.init(Id.getRows(), Id.getCols(), Z);
    vpFeatLumDes.setCameraParameters(cam);
    vpFeatLumDes.buildFrom(Id);

    vpColVector err;

    vpMatrix interactMat;
    vpColVector roboVel;
    double errNorm, lambda = 10;


    while(1){
        //acquire adn display current image
        camera.setCameraPosition(cMo);
        I=0;
        camera.getImage(I, cam);
        vpDisplay::display(I);
        vpDisplay::flush(I);


        // get image diff
        vpImageTools::imageDifference(Id, I, Idiff); //diff
        vpDisplay::display(Idiff);
        vpDisplay::flush(Idiff);


        //build current visual features
        vpFeatLumCur.buildFrom(I);


        //compute error
        err = vpFeatLumCur.error(vpFeatLumDes);


        //compute interaction matrix
        interactMat =  vpFeatLumCur.interaction();



        //compute control velocities
        roboVel = -lambda * interactMat.pseudoInverse() * err;

        //set robot vel
        robot.setVelocity(vpRobot::CAMERA_FRAME, roboVel);

        //get position
        robot.getPosition(cMo);


        errNorm = err.euclideanNorm();
        std::cout << errNorm << std::endl;

        if (errNorm < 500){
            vpDisplay::getClick(I);
            vpDisplay::getClick(Id);
            vpDisplay::getClick(Idiff);
        }


    }






    return 0;


}

