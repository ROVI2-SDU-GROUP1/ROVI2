/*
*   Class for visually adjusting and finding the appropiate HSV segmentation values
*/

#include "HsvAdjust.hpp"

HsvAdjust::HsvAdjust()
{
    namedWindow("Original", cv::WINDOW_NORMAL);
    namedWindow("Colour Segmentation", cv::WINDOW_NORMAL);
    cv::resizeWindow("Original", 500,500);
    cv::resizeWindow("Colour Segmentation", 500,500);

    cv::createTrackbar( "Hue min", "Colour Segmentation", &hSliderMin, hSlider, &HsvAdjust::on_trackbar, (void *)this);
    cv::createTrackbar( "Hue max", "Colour Segmentation", &hSliderMax, hSlider,  &HsvAdjust::on_trackbar, (void *)this);
    cv::createTrackbar( "S min", "Colour Segmentation", &sSliderMin, sSlider,  &HsvAdjust::on_trackbar, (void *)this);
    cv::createTrackbar( "S max", "Colour Segmentation", &sSliderMax, sSlider,  &HsvAdjust::on_trackbar, (void *)this);
    cv::createTrackbar( "V min", "Colour Segmentation", &vSliderMin, vSlider,  &HsvAdjust::on_trackbar, (void *)this);
    cv::createTrackbar( "V max", "Colour Segmentation", &vSliderMax, vSlider,  &HsvAdjust::on_trackbar, (void *)this);
    hSliderMin = 0;
    hSliderMax = 180;
	sSliderMin = 0;
	sSliderMax = 255;
	vSliderMin = 0;
	vSliderMax = 255;

}

void HsvAdjust::on_trackbar( int something, void *userdata)
{
    ((HsvAdjust *)(userdata))->on_trackbar(something);
}

void HsvAdjust::on_trackbar( int)
{
    cv::inRange(imghsv, cv::Scalar(hSliderMin, sSliderMin, vSliderMin), cv::Scalar(hSliderMax, sSliderMax, vSliderMax), dst);
    imshow( "Colour Segmentation", dst );
}

void HsvAdjust::hsvSegmentation(Mat img)
{
    cout << __LINE__ << endl;
    imshow("Original", img);
    cout << __LINE__ << endl;
    cout << __LINE__ << endl;
    cv::cvtColor(img, imghsv, CV_BGR2HSV);
    cout << __LINE__ << endl;



    cout << __LINE__ << endl;

    cout << __LINE__ << endl;


    // Print the found HSV values
    cout << "Hue min: " << hSliderMin << endl;
    cout << "Hue max: " << hSliderMax << endl;
    cout << "Sat min: " << sSliderMin << endl;
    cout << "Sat max: " << sSliderMax << endl;
    cout << "Val min: " << vSliderMin << endl;
    cout << "Val max: " << vSliderMax << endl;
    cout << __LINE__ << endl;

}

/*
*   Grabs an image from the webcam and lets the user adjust HSV values for segmenting the image.
*/
void HsvAdjust::segmentFromWebcam()
{
    Mat cameraFrame;
    VideoCapture camera(0); // Open default camera 0

    if (!camera.isOpened())
    {
        cout << "ERROR: Cannot open camera!" << endl;
        return;
    }

    while (1)
    {
        camera.read(cameraFrame);   // Grab image from webcam

        imshow("Webcam", cameraFrame);

        if( waitKey(25) == 27 ) // Frame rate: 25ms, stop capturing by pressing ESC.
            break;
    }

    // Perform hsv segmentation on the captured picture
    hsvSegmentation(cameraFrame);
}

void HsvAdjust::grabWebcamImage(string savePath)
{
    Mat cameraFrame;
    VideoCapture camera(0); // Open default camera 0

    if (!camera.isOpened())
    {
        cout << "ERROR: Cannot open camera!" << endl;
        return;
    }

    while (1)
    {
        camera.read(cameraFrame);   // Grab image from webcam

        imshow("Webcam", cameraFrame);

        if( waitKey(25) == 27 ) // Frame rate: 25ms, stop capturing by pressing ESC.
            break;
    }

    imwrite(savePath, cameraFrame); // Save image to disk
}
