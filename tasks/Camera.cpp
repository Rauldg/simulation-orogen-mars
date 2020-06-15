/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Camera.hpp"
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp>
#include <base-logging/Logging.hpp>

using namespace mars;

Camera::Camera(std::string const& name)
    : CameraBase(name)
{
}

Camera::Camera(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraBase(name, engine)
{
}

Camera::~Camera()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Camera.hpp for more detailed
// documentation about them.




bool Camera::configureHook()
{
    
    if (! mars::CameraPlugin::configureHook())
        return false;
    
    LOG_DEBUG("Configuring");
    // If nothing is set, then use default value: false
    if (_isTIR.get())
        LOG_DEBUG("Is a TIR %s", _name.get().c_str());
        isTIR = _isTIR.get();
    
    return true;
    
}



bool Camera::startHook()
{
    
    if (! mars::CameraPlugin::startHook())
        return false;
    
    image = new base::samples::frame::Frame(width,height,8,base::samples::frame::MODE_RGB);

    if (isTIR)
    {
        tirImage = new base::samples::DistanceImage(width,height);
        tirImage->setSize(width, height);
        t_ro_ptr.reset(tirImage);
    }
    ro_ptr.reset(image);
    marsImage.resize(width * height);
    
    return true;
    
}



void Camera::updateHook()
{
    
    mars::CameraPlugin::updateHook();
    

    

    
}



void Camera::errorHook()
{
    
    mars::CameraPlugin::errorHook();
    

    

    
}



void Camera::stopHook()
{
    
    mars::CameraPlugin::stopHook();
    

    

    
}



void Camera::cleanupHook()
{
    
    mars::CameraPlugin::cleanupHook();
    

    

    
}

double Camera::hsvTemperature(cv::Vec3b hsv_v) const
{
    double t_h, t_s, t_v, res;
    if (hsv_v[0]>135.0)
    {
      t_h = (5.0/12.0)*hsv_v[0] - 56.25;
    }
    else
    {
      t_h = -(50.0/135.0)*hsv_v[0] + 50.0;
    }
    t_s = (100.0/255.0)*hsv_v[1] - 50.0;
    t_v = (100.0/255.0)*hsv_v[2] - 50.0;
    res = std::min(0.5*t_h + 0.25*t_s + 0.25*t_v, 50.0);
    res = std::max(res, 0.0);
    return res;
}

void Camera::getData()
{	
    camera->getImage(marsImage);

    image = ro_ptr.write_access();

    if (isTIR)
    {

        tirImage = t_ro_ptr.write_access();
        cv::Mat cv_im_rgba(width, height, CV_8UC4, marsImage.data()); //open_cv creates initially brg images
        cv::Mat cv_im_rgb;
        cv::cvtColor(cv_im_rgba, cv_im_rgb, cv::COLOR_RGBA2RGB);
        cv::Mat cv_im_hsv;
        cv::cvtColor(cv_im_rgb, cv_im_hsv, cv::COLOR_RGB2HSV);
        cv::Mat tir(width, height, CV_64F);
        cv::Mat cv_tir_display(width, height, CV_8UC3);

        for (int i = 0; i < cv_im_hsv.rows; i++)
        {
          for (int j = 0; j < cv_im_hsv.cols; j++)
          {
            tir.at<double>(i,j) = hsvTemperature(cv_im_hsv.at<cv::Vec3b>(i, j));
            cv_tir_display.at<cv::Vec3b>(i,j) = {
              (int)((tir.at<double>(i,j) + 50.0)*(2.55)),
              (int)((tir.at<double>(i,j) + 50.0)*(2.55)),
              (int)((tir.at<double>(i,j) + 50.0)*(2.55))
            };
          }
        }

        //double min, max;
        //cv::minMaxLoc(tir, &min, &max);
        //LOG_DEBUG("Temperature range: (%f, %f)", min, max);

        cv::applyColorMap(cv_tir_display, cv_tir_display, cv::COLORMAP_JET);

        LOG_DEBUG("TIR image constructed");
        uint8_t *image_dst = image->getImagePtr();
        uint8_t i_tir = 0;
        for(int i=height-1;i>=0;--i)
        {
            for(int i2=0;i2<width;++i2)
            {
                *(image_dst++) = cv_tir_display.at<cv::Vec3b>(i,i2)[2];
                *(image_dst++) = cv_tir_display.at<cv::Vec3b>(i,i2)[1];
                *(image_dst++) = cv_tir_display.at<cv::Vec3b>(i,i2)[0];
                tirImage->data[i_tir] = tir.at<double>(i,i2) + 273.15; //to Kelvin from centigrades
                i_tir++;
            }
        }
        LOG_DEBUG("TIR image converted to output format");

        //set attributes
        tirImage->time = getTime();

        t_ro_ptr.reset(tirImage);
        _tir_image.write(t_ro_ptr);
        LOG_DEBUG("TIR image sent");
    }
    else
    {
        //copy image data
        //data format is ARGB therefore we have to skip every 4th byte
        //to convert it into RGB
        //image is flipped
        const mars::sim::Pixel *image_src = marsImage.data();

        uint8_t *image_dst = image->getImagePtr();
        for(int i=height-1;i>=0;--i)
        {
            image_src = marsImage.data()+width*i;
            for(int i2=0;i2<width;++i2)
            {
                *(image_dst++) = image_src->r;
                *(image_dst++) = image_src->g;
                *(image_dst++) = image_src->b;
                ++image_src;
            }
        }
    }

    //set attributes
    image->time = getTime();
    image->received_time = image->time;
    image->frame_status = base::samples::frame::STATUS_VALID;
    
    ro_ptr.reset(image);

    _frame.write(ro_ptr);
        
}
