// compile against libStructure and opencv
// g++ cobast.cpp -I/opt/StructureSDK-CrossPlatform-0.7.3-ROS/Libraries/Structure/Headers/ -lStructure `pkg-config opencv --libs`

#include <iostream>
#include <ST/CaptureSession.h>
#include <ST/CameraFrames.h>
#include <ST/DeviceManager.h>
#include <ST/Utilities.h>
#include <ST/MathTypes.h>
#include <stdio.h>
#include <unistd.h>

ST::DepthFrame lastDepthFrame;
ST::ColorFrame lastVisibleFrame;
ST::InfraredFrame lastInfraredFrame;

struct SessionDelegate : ST::CaptureSessionDelegate {
    void captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) override {
        printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
        switch (event) {
            case ST::CaptureSessionEventId::Booting: break;
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

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) override {
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
                // here is the actual work
                printf("Synchronized frames: depth %dx%d visible %dx%d infrared %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height(), sample.visibleFrame.width(), sample.visibleFrame.height(), sample.infraredFrame.width(), sample.infraredFrame.height());
                if (sample.depthFrame.isValid()) {
        _lastDepthFrame = sample.depthFrame;
    }
    if (sample.visibleFrame.isValid()) {
        _lastVisibleFrame = sample.visibleFrame;
    }
    if (sample.infraredFrame.isValid()) {
        _lastInfraredFrame = sample.infraredFrame;
    }
                break;
            default: break;
        }
    }
};

int main(int argc, char **argv){
	ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = true;
    settings.structureCore.visibleEnabled = true;
    settings.structureCore.infraredEnabled = true;
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
    settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_200Hz;

    SessionDelegate delegate;
    ST::CaptureSession session;
    session.setDelegate(&delegate);
    if (!session.startMonitoring(settings)) {
        printf("Failed to initialize capture session!\n");
        exit(1);
    }

    /* Loop forever. The SessionDelegate receives samples on a background thread
       while streaming. */
    while (true) {
		printf("%f depth %dx%d\n", _lastDepthFrame.timestamp(), _lastDepthFrame.width(), _lastDepthFrame.height());
		printf("%f visible %dx%d\n", _lastVisibleFrame.timestamp(), _lastVisibleFrame.width(), _lastVisibleFrame.height());
		printf("%f infrared %dx%d\n", _lastVisibleFrame.timestamp(), _lastInfraredFrame.width(), _lastInfraredFrame.height());
 }
	}
