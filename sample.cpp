// compile against libStructure and opencv
// g++ sample.cpp -I/opt/StructureSDK-CrossPlatform-0.7.3-ROS/Libraries/Structure/Headers/ -lStructure `pkg-config opencv --libs`

#include <ST/CaptureSession.h>
#include <ST/CameraFrames.h>
#include <ST/DeviceManager.h>
#include <ST/Utilities.h>
#include <ST/MathTypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

ST::DepthFrame currentDepthFrame;
ST::ColorFrame currentVisibleFrame;
ST::InfraredFrame currentInfraredFrame;

#define VISIBLE_HEIGHT 480
#define VISIBLE_WIDTH 640
#define DEPTH_HEIGHT 960
#define DEPTH_WIDTH 1280
char status=1;

cv::Mat display(VISIBLE_HEIGHT, VISIBLE_WIDTH, CV_8UC3);
cv::Mat depth(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);
cv::Mat depth2(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC4);

double frametimePast[3]={1,1,1}, frametimeCurrent[3];
struct timeval start, stop;
unsigned int frames_count;

struct SessionDelegate: ST::CaptureSessionDelegate {
  void captureSessionEventDidOccur(ST::CaptureSession * session, ST::CaptureSessionEventId event) override {
    printf("Received capture session event %d (%s)\n", (int) event, ST::CaptureSessionSample::toString(event));
    switch (event) {
    case ST::CaptureSessionEventId::Booting:
      break;
    case ST::CaptureSessionEventId::Connected:
      printf("Starting streams...\n");
      printf("Sensor Serial Number is %s \n ", session->sensorSerialNumber());
      session->startStreaming();
      break;
    case ST::CaptureSessionEventId::Disconnected:
    case ST::CaptureSessionEventId::Error:
      printf("Capture session error\n");
      exit(1);
      break;
    default:
      printf("Capture session event unhandled\n");
    }
  }

  void captureSessionDidOutputSample(ST::CaptureSession * ,
    const ST::CaptureSessionSample & sample) override {
    //printf("Received capture session sample of type %d (%s)\n", (int)sample.type, ST::CaptureSessionSample::toString(sample.type));
    switch (sample.type) {
    case ST::CaptureSessionSample::Type::DepthFrame:
      //    printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
      break;
    case ST::CaptureSessionSample::Type::VisibleFrame:
      //  printf("Visible frame: size %dx%d\n", sample.visibleFrame.width(), sample.visibleFrame.height());
      break;
    case ST::CaptureSessionSample::Type::InfraredFrame:
      //printf("Infrared frame: size %dx%d\n", sample.infraredFrame.width(), sample.infraredFrame.height());
      break;
    case ST::CaptureSessionSample::Type::SynchronizedFrames:
      status = 0;
      // here is the actual work
      //printf("Synchronized frames: depth %dx%d visible %dx%d infrared %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height(), sample.visibleFrame.width(), sample.visibleFrame.height(), sample.infraredFrame.width(), sample.infraredFrame.height());
      if (sample.depthFrame.isValid()) {
		  if (frametimeCurrent[0] != sample.depthFrame.timestamp()){
        frametimeCurrent[0] = sample.depthFrame.timestamp();
        currentDepthFrame = sample.depthFrame;
      }}
      if (sample.visibleFrame.isValid()) {
        if (frametimeCurrent[1] != sample.visibleFrame.timestamp()){
        frametimeCurrent[1] = sample.visibleFrame.timestamp();
        currentVisibleFrame = sample.visibleFrame;
        gettimeofday(&stop,NULL);
		frames_count++;
      }}
   /*   if (sample.infraredFrame.isValid()) {
        if (frametimeCurrent[2] != sample.infraredFrame.timestamp()){
        frametimeCurrent[2] = sample.infraredFrame.timestamp();
        currentInfraredFrame = sample.infraredFrame;
      }}*/
      break;
    default:
      break;
    }
    
  }
  
};

int main(int argc, char **argv){
	ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = true;
    settings.structureCore.visibleEnabled = true;
    settings.structureCore.infraredEnabled = false; // ~8 fps on Jetson when turned off 
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::SXGA;
    settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_200Hz;

    SessionDelegate delegate;
    ST::CaptureSession session;
    session.setDelegate(&delegate);
    if (!session.startMonitoring(settings)) {
        printf("Failed to initialize capture session!\n");
        exit(1);
    }

	while(status){}
	sleep(1); // just to make sure all the frames are ready

	gettimeofday(&start,NULL);
    while (true) {
		if (frametimeCurrent[0] != frametimePast[0]) {
			frametimePast[0] = frametimeCurrent[0];
			memcpy(depth2.data, currentDepthFrame.convertDepthToRgba() ,sizeof(uchar) * DEPTH_WIDTH * DEPTH_HEIGHT * 4); //16-bit
			cv::imshow("depth", depth2);
			//memcpy(depth.data, currentDepthFrame.convertDepthToUShortInMillimeters() ,sizeof(uint16_t) * DEPTH_WIDTH * DEPTH_HEIGHT); //16-bit
			//printf("%f depth %dx%d\n", currentDepthFrame.timestamp(), currentDepthFrame.width(), currentDepthFrame.height());
		}
		if (frametimeCurrent[1] != frametimePast[1]) {
			frametimePast[1] = frametimeCurrent[1];
			//printf("%f visible %dx%d\n", currentVisibleFrame.timestamp(), currentVisibleFrame.width(), currentVisibleFrame.height());
			memcpy(display.data, currentVisibleFrame.rgbData(), sizeof(uchar) * VISIBLE_WIDTH * VISIBLE_HEIGHT * 3); // RGB
			cvtColor(display, display, cv::COLOR_RGB2BGR);
			cv::imshow("visible", display);
		}
		if (frametimeCurrent[2] != frametimePast[2]) {
			frametimePast[2] = frametimeCurrent[2];
			//printf("%f infrared %dx%d\n", currentVisibleFrame.timestamp(), currentInfraredFrame.width(), currentInfraredFrame.height());
		}
		// update fps counter every second
		if ((1e6*(stop.tv_sec-start.tv_sec) +stop.tv_usec -start.tv_usec) > 1e6){
			printf("fps: %d\n", frames_count);
			gettimeofday(&start,NULL);
			frames_count= 0;
			}
		cv::waitKey(1);
	}
}
