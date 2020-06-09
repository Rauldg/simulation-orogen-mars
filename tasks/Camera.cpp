/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Camera.hpp"

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
    
    // If nothing is set, then use default value: false
    if (_isTIR.get())
        isTIR = _isTIR.get();
    
    return true;
    
}



bool Camera::startHook()
{
    
    if (! mars::CameraPlugin::startHook())
        return false;
    
    image = new base::samples::frame::Frame(width,height,8,base::samples::frame::MODE_RGB);
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

void Camera::getData()
{	
    camera->getImage(marsImage);

    image = ro_ptr.write_access();
    
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
    //set attributes
    image->time = getTime();
    image->received_time = image->time;
    image->frame_status = base::samples::frame::STATUS_VALID;
    
    ro_ptr.reset(image);

    if(isTIR)
    {
        LOG_DEBUG("I am a TIR Camera!");
    }

    _frame.write(ro_ptr);
}
