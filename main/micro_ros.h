#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <stdint.h>

#include <visualization_msgs/msg/marker.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/camera_info.h>

#define IMAGE_HEIGHT 80
#define IMAGE_WIDTH 80
#define IMAGE_DEPTH 3
#define IMAGE_BYTES IMAGE_HEIGHT*IMAGE_WIDTH*IMAGE_DEPTH

#ifdef __cplusplus
#include "depthai-shared/datatype/RawImgDetections.hpp"

extern "C" {
#endif

typedef struct {
    uint32_t label;
    float confidence;
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    float xdepth;
    float ydepth;
    float zdepth;
} micro_ros_detection_t;

void init_microros();
void init_sensor_msgs_image(sensor_msgs__msg__Image * image_msg);
void init_sensor_msgs_camera_info(sensor_msgs__msg__CameraInfo * camera_info);
void init_visualization_msgs_marker(visualization_msgs__msg__Marker * marker);

typedef struct {
    uint8_t * data;
    size_t len;
} micro_ros_fragment_t;

static char const * yolo_labels[] = { "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

#ifdef __cplusplus
}

void convert_detection(dai::ImgDetection source, micro_ros_detection_t * dest){
    dest->label = source.label;
    dest->confidence = source.confidence;
    dest->xmax = source.xmax;
    dest->xmin = source.xmin;
    dest->ymax = source.ymax;
    dest->ymin = source.ymin;
    dest->xdepth = source.xdepth;
    dest->ydepth = source.ydepth;
    dest->zdepth = source.zdepth;
}
#endif

#endif // MICRO_ROS_H
