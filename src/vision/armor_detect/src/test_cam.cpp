#include "vision/RMVideoCapture.hpp"
#include "vision/Settings.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;


RMVideoCapture::CAM_PARA campara;

Mat img;
RMVideoCapture *pcap2;

static void setpara(int para,void *p)
{
    if (p==NULL)
        return;
    RMVideoCapture *pcap=(RMVideoCapture *)p;
    pcap->cam_para=campara;
    pcap->setpara();
}

int main(int argc, char** argv) 
{
    
    RMVideoCapture cap2("/dev/video0");
    pcap2=&cap2;
    cap2.info();
    cout<<"\n  Pre Parameter  \n"<<endl;
    cap2.changeVideoFormat(640, 480, 0);
    cap2.getCurrentSetting();
    
    campara=cap2.cam_para;
    
    namedWindow("img", 1);  
    createTrackbar("gain", "img",&campara.gain,100,setpara);  
    createTrackbar("exposure", "img",&campara.exposure,100,setpara,pcap2);  
    createTrackbar("brightness", "img",&campara.brightness,128,setpara,pcap2);  
    createTrackbar("whiteness", "img",&campara.whiteness,500,setpara,pcap2);  
    createTrackbar("saturation", "img",&campara.saturation,128,setpara,pcap2);  
    createTrackbar("contrast", "img",&campara.contrast,64,setpara,pcap2); 
    
    setTrackbarPos("gain", "img", cap2.cam_para.gain);
    setTrackbarPos("exposure", "img", cap2.cam_para.exposure);
    setTrackbarPos("brightness", "img", cap2.cam_para.brightness);
    setTrackbarPos("whiteness", "img", cap2.cam_para.whiteness);
    setTrackbarPos("saturation", "img", cap2.cam_para.saturation);
    setTrackbarPos("contrast", "img", cap2.cam_para.contrast);
    
    
    
    setpara(0,NULL);
    cap2.startStream();
    char c='1';
    
    char * config_file_name = "../param_config.xml";
    Settings setting(config_file_name);
    ArmorDetector armor_detector(setting.armor);
    Mat template_img = imread(setting.template_image_file);
    Mat small_template_img = imread(setting.small_template_image_file);
    armor_detector.initTemplate(template_img, small_template_img);
    armor_detector.setPara(setting.armor);
    
    FileStorage fs(setting.intrinsic_file_480, FileStorage::READ);
    Mat cam_matrix_480, distortion_coeff_480;
    fs["Camera_Matrix"] >> cam_matrix_480;
    fs["Distortion_Coefficients"] >> distortion_coeff_480;
    FileStorage fs1(setting.intrinsic_file_720, FileStorage::READ);
    Mat cam_matrix_720, distortion_coeff_720;
    fs1["Camera_Matrix"] >> cam_matrix_720;
    fs1["Distortion_Coefficients"] >> distortion_coeff_720;
    
    while(1)
    {
        cap2>>img;
        Mat img_undistort;
        if (img.rows==480)
            undistort(img,img_undistort,cam_matrix_480,distortion_coeff_480);
        else 
            undistort(img,img_undistort,cam_matrix_720,distortion_coeff_720);
        RotatedRect rect;
        rect = armor_detector.getTargetAera(img_undistort);
        cout<<rect.center<<endl;
        Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++){
            line(img_undistort, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 3);
        }
        imshow("img",img_undistort);
        c=waitKey(1);
        if (c=='q')
            break;
    }
    cap2.closeStream();  
    cout<<"\n  Current Parameter  \n"<<endl;
    cap2.getCurrentSetting();
}
