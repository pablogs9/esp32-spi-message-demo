#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_uros/options.h>
#include <uxr/client/config.h>
#include <ucdr/microcdr.h>

#include <micro_ros.h>
#include "decode_mobilenet.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);while(1);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void wifi_init_sta();

extern QueueHandle_t micro_ros_queue_detections_handle;
extern QueueHandle_t micro_ros_queue_image_handle;
extern SemaphoreHandle_t micro_ros_semaphore;

void size_cb(uint32_t * topic_length){
    *topic_length += ucdr_alignment(*topic_length, sizeof(uint32_t)) + sizeof(uint32_t);
    *topic_length += IMAGE_BYTES;
}

void serialization_cb(ucdrBuffer * ucdr){
    size_t len = 0;
    micro_ros_fragment_t fragment;
    ucdr_serialize_uint32_t(ucdr, IMAGE_BYTES); 
    while(len < IMAGE_BYTES){
       xQueueReceive(micro_ros_queue_image_handle, &fragment, portMAX_DELAY);
       xSemaphoreTake( micro_ros_semaphore, portMAX_DELAY );
       ucdr_serialize_array_uint8_t(ucdr, fragment.data, fragment.len);
       len += fragment.len;
       xSemaphoreGive(micro_ros_semaphore);
    }
}

void rmw_uros_set_continous_serialization_callbacks(rmw_uros_continous_serialization_size size, rmw_uros_continous_serialization cb);



void microros_task()
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	// RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "depthai_node", "", &support));

	// // create publisher
    rcl_publisher_t publisher_marker;
	RCCHECK(rclc_publisher_init_default(
		&publisher_marker,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, Marker),
		"/depthai_marker"));

    // create publisher
    rcl_publisher_t publisher_camerainfo;
	RCCHECK(rclc_publisher_init_default(
		&publisher_camerainfo,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CameraInfo),
		"/camera_info"));

	// create publisher
    rcl_publisher_t publisher_image;
	RCCHECK(rclc_publisher_init_default(
		&publisher_image,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
		"/depthai_image"));

	// create executor
    rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 100;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    struct timespec tv = {0};

    visualization_msgs__msg__Marker marker_msg;
    sensor_msgs__msg__Image image_msg;
    sensor_msgs__msg__CameraInfo camerainfo_msg;

    init_visualization_msgs_marker(&marker_msg);
    init_sensor_msgs_image(&image_msg);
    init_sensor_msgs_camera_info(&camerainfo_msg);
    
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(rcl_wait_timeout));

        micro_ros_detection_t det;
        if (xQueueReceive(micro_ros_queue_detections_handle, &det, 0) == pdTRUE)
        {
            clock_gettime(CLOCK_MONOTONIC, &tv);
            marker_msg.header.stamp.nanosec = tv.tv_nsec;
            marker_msg.header.stamp.sec = tv.tv_sec;
                    
            sprintf(marker_msg.text.data, "%s", yolo_labels[det.label]);
            marker_msg.text.size = strlen(marker_msg.text.data);

            marker_msg.pose.position.x = (double) det.xdepth;
            marker_msg.pose.position.y = (double) det.ydepth;
            marker_msg.pose.position.z = (double) det.zdepth;
            RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
        }

        micro_ros_fragment_t fragment;
        if (xQueueReceive(micro_ros_queue_image_handle, &fragment, 0) == pdTRUE)
        {
            clock_gettime(CLOCK_MONOTONIC, &tv);
            image_msg.header.stamp.nanosec = tv.tv_nsec;
            image_msg.header.stamp.sec = tv.tv_sec;
            
            camerainfo_msg.header.stamp.nanosec = tv.tv_nsec;
            camerainfo_msg.header.stamp.sec = tv.tv_sec;
            
            // image_msg.data.data = image;
            
            RCSOFTCHECK(rcl_publish(&publisher_camerainfo, &camerainfo_msg, NULL));
            rmw_uros_set_continous_serialization_callbacks(size_cb, serialization_cb);
            RCSOFTCHECK(rcl_publish(&publisher_image, &image_msg, NULL));
            rmw_uros_set_continous_serialization_callbacks(NULL, NULL);
        }
        usleep(50000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher_camerainfo, &node))
	RCCHECK(rcl_publisher_fini(&publisher_image, &node))
	RCCHECK(rcl_publisher_fini(&publisher_marker, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}

void init_microros()
{   
    // Start networkign if required
#ifdef UCLIENT_PROFILE_UDP
    wifi_init_sta();
#endif  // UCLIENT_PROFILE_UDP
    xTaskCreate(microros_task, "microros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);
}