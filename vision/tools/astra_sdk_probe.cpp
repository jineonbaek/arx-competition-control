// Native OpenNI parameter audit. No firmware, emitter, flash, robot or CAN writes.
#include <OpenNI.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <stdexcept>
#include <filesystem>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

static_assert(sizeof(OniDeviceInfo)==1028,"Use the matching Orbbec headers, not stock OpenNI headers");
static_assert(sizeof(OBCameraParams)==120,"Unexpected calibration ABI");
struct GuardedParams { OBCameraParams value; std::array<unsigned char,32> guard; };

bool intact(const GuardedParams& data) {
    return std::all_of(data.guard.begin(),data.guard.end(),[](unsigned char c){return c==0xa5;});
}

void parameters(openni::Device& device,const char* phase) {
    GuardedParams data;std::memset(&data,0xa5,sizeof(data));int size=sizeof(data.value);
    auto status=device.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS,&data.value,&size);
    std::cout<<"PARAMETER_PHASE "<<phase<<" status="<<status<<" returned_bytes="<<size<<" guard_ok="<<intact(data)<<"\n";
    if (!intact(data)) throw std::runtime_error("SDK wrote beyond the requested parameter buffer");
    if(status!=openni::STATUS_OK) {std::cout<<"ERROR "<<openni::OpenNI::getExtendedError()<<"\n";return;}
    auto* bytes=reinterpret_cast<unsigned char*>(&data.value);
    std::cout<<"RAW_HEX ";for(size_t i=0;i<sizeof(data.value);++i)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(bytes[i]);
    std::cout<<std::dec<<"\n";
    if(size!=sizeof(data.value))return;
    auto print=[](const char* key,const float* values,int count){std::cout<<key;for(int i=0;i<count;++i)std::cout<<" "<<values[i];std::cout<<"\n";};
    print("DEPTH_INTRINSICS",data.value.l_intr_p,4);print("COLOR_INTRINSICS",data.value.r_intr_p,4);
    print("ROTATION",data.value.r2l_r,9);print("TRANSLATION",data.value.r2l_t,3);
    print("DEPTH_DISTORTION",data.value.l_k,5);print("COLOR_DISTORTION",data.value.r_k,5);
}

void text_property(openni::Device& device,int property,const char* key) {
    char value[256]={};int size=sizeof(value);auto rc=device.getProperty(property,value,&size);
    value[sizeof(value)-1]=0;
    std::cout<<key<<" status="<<rc<<" bytes="<<size<<" value="<<(rc==openni::STATUS_OK?value:openni::OpenNI::getExtendedError())<<"\n";
}

int main(int argc,char** argv) {
    bool stream_frames=false;
    if(argc==2 && std::string(argv[1])=="--self-test") {
        GuardedParams test;std::memset(&test,0xa5,sizeof(test));assert(intact(test));test.guard[0]=0;assert(!intact(test));
        std::memset(&test.value,0xff,sizeof(test.value));assert(!std::isfinite(test.value.l_intr_p[0]));
        std::cout<<"PASS matching ABI, buffer guard and all-FF detection; no SDK initialization\n";return 0;
    }
    if(argc==2 && std::string(argv[1])=="--stream")stream_frames=true;
    else if(argc!=1){std::cerr<<"Usage: astra_sdk_probe [--self-test|--stream]\n";return 2;}
    auto lock_path=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"web/logs/camera-preview.lock";
    std::filesystem::create_directories(lock_path.parent_path());
    int lock_fd=open(lock_path.c_str(),O_CREAT|O_RDWR,0664);
    if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)!=0){
        if(lock_fd>=0)close(lock_fd);
        std::cerr<<"Camera owner holds the capture lock; stop camera preview before this probe. No SDK initialized.\n";return 3;
    }
    auto version=openni::OpenNI::getVersion();
    std::cout<<"SDK_VERSION "<<version.major<<"."<<version.minor<<"."<<version.maintenance<<"."<<version.build<<"\n"<<std::flush;
    if(openni::OpenNI::initialize()!=openni::STATUS_OK){std::cerr<<openni::OpenNI::getExtendedError()<<"\n";close(lock_fd);return 1;}
    int result=0;
    {
        openni::Array<openni::DeviceInfo> devices;openni::OpenNI::enumerateDevices(&devices);int found=-1;
        for(int i=0;i<devices.getSize();++i){
            auto& d=devices[i];std::cout<<"DEVICE "<<d.getUri()<<" "<<d.getVendor()<<" "<<d.getName()<<" vid="<<d.getUsbVendorId()<<" pid="<<d.getUsbProductId()<<"\n";
            if(d.getUsbVendorId()==0x2bc5 && d.getUsbProductId()==0x0403){if(found!=-1){result=1;break;}found=i;}
        }
        openni::Device device;openni::VideoStream depth;
        if(found<0 || result || device.open(devices[found].getUri())!=openni::STATUS_OK){std::cerr<<"Expected one Astra Pro: "<<openni::OpenNI::getExtendedError()<<"\n";result=1;}
        else try {
            text_property(device,0,"FIRMWARE");text_property(device,3,"SERIAL_STANDARD");text_property(device,16,"SERIAL_VENDOR");
            OniVersion driver={};int driver_size=sizeof(driver);auto rc=device.getProperty(1,&driver,&driver_size);
            std::cout<<"DRIVER_VERSION status="<<rc<<" "<<driver.major<<"."<<driver.minor<<"."<<driver.maintenance<<"."<<driver.build<<"\n";
            parameters(device,"device_open");
            if(depth.create(device,openni::SENSOR_DEPTH)!=openni::STATUS_OK)throw std::runtime_error(openni::OpenNI::getExtendedError());
            openni::VideoMode mode;mode.setResolution(640,480);mode.setFps(30);mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
            if(depth.setVideoMode(mode)!=openni::STATUS_OK)throw std::runtime_error(openni::OpenNI::getExtendedError());
            auto actual=depth.getVideoMode();std::cout<<"PROFILE "<<actual.getResolutionX()<<"x"<<actual.getResolutionY()<<"@"<<actual.getFps()<<" pixel_format="<<actual.getPixelFormat()<<"\n";
            std::cout<<"DEPTH_FOV "<<depth.getHorizontalFieldOfView()<<" "<<depth.getVerticalFieldOfView()<<"\n";
            parameters(device,"depth_configured_not_started");
            for(const auto& pixel:std::array<std::array<int,2>,4>{{{{320,240}},{{0,240}},{{320,0}},{{639,479}}}}){
                float x=0,y=0,z=0;rc=openni::CoordinateConverter::convertDepthToWorld(depth,pixel[0],pixel[1],1000,&x,&y,&z);
                std::cout<<"SDK_CONVERTER_SYNTHETIC_Z_1000MM "<<pixel[0]<<" "<<pixel[1]<<" status="<<rc<<" xyz="<<x<<","<<y<<","<<z<<"\n";
            }
            if(stream_frames){
                if(depth.start()!=openni::STATUS_OK)throw std::runtime_error(openni::OpenNI::getExtendedError());
                openni::VideoFrameRef frame;openni::VideoStream* streams[]={&depth};
                for(int n=0;n<8;++n){int ready=0;if(openni::OpenNI::waitForAnyStream(streams,1,&ready,2000)!=openni::STATUS_OK || depth.readFrame(&frame)!=openni::STATUS_OK)throw std::runtime_error("Depth frame timeout");}
                auto* values=static_cast<const openni::DepthPixel*>(frame.getData());int valid=0;auto count=frame.getWidth()*frame.getHeight();
                for(int i=0;i<count;++i)if(values[i])++valid;
                std::cout<<"REAL_DEPTH_FRAME "<<frame.getWidth()<<"x"<<frame.getHeight()<<" index="<<frame.getFrameIndex()<<" valid="<<valid<<"/"<<count<<"\n";
                parameters(device,"depth_started");frame.release();depth.stop();
            }
        }catch(const std::exception& error){std::cerr<<"PROBE_ERROR "<<error.what()<<"\n";result=1;}
        depth.destroy();device.close();
    }
    openni::OpenNI::shutdown();close(lock_fd);return result;
}
