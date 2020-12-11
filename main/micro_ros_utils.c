#include <string.h>

#include <micro_ros.h>

void init_sensor_msgs_image(sensor_msgs__msg__Image * image_msg){
    memset(image_msg, 0, sizeof(sensor_msgs__msg__Image));

    char string1[] = "/world";
    image_msg->header.frame_id.data = (char*)malloc((strlen(string1) + 1)*sizeof(char));
	memcpy(image_msg->header.frame_id.data, string1, strlen(string1) + 1);
	image_msg->header.frame_id.size = strlen(image_msg->header.frame_id.data);
	image_msg->header.frame_id.capacity = strlen(string1) + 1;

    image_msg->height = IMAGE_HEIGHT;
    image_msg->width = IMAGE_WIDTH;

    char string2[] = "bgr8";
    image_msg->encoding.data = (char*)malloc((strlen(string2) + 1)*sizeof(char));
	memcpy(image_msg->encoding.data, string2, strlen(string2) + 1);
	image_msg->encoding.size = strlen(image_msg->encoding.data);
	image_msg->encoding.capacity = strlen(string2) + 1;
    
    image_msg->is_bigendian = 0;
    image_msg->step = IMAGE_DEPTH*IMAGE_WIDTH;
}

void init_sensor_msgs_camera_info(sensor_msgs__msg__CameraInfo * camera_info){
    memset(camera_info, 0, sizeof(sensor_msgs__msg__CameraInfo));

    char string1[] = "/world";
    camera_info->header.frame_id.data = (char*)malloc((strlen(string1) + 1)*sizeof(char));
	memcpy(camera_info->header.frame_id.data, string1, strlen(string1) + 1);
	camera_info->header.frame_id.size = strlen(camera_info->header.frame_id.data);
	camera_info->header.frame_id.capacity = strlen(string1) + 1;

    camera_info->height = IMAGE_HEIGHT;
    camera_info->width = IMAGE_WIDTH;

    char string2[] = "plumb_bob";
    camera_info->distortion_model.data = (char*)malloc((strlen(string2) + 1)*sizeof(char));
	memcpy(camera_info->distortion_model.data, string2, strlen(string2) + 1);
	camera_info->distortion_model.size = strlen(camera_info->distortion_model.data);
	camera_info->distortion_model.capacity = strlen(string2) + 1;

    camera_info->d.data = (double*)malloc(5*sizeof(double));
    memset(camera_info->d.data, 0, 5*sizeof(double));
    camera_info->d.size = 5;
    camera_info->d.capacity = 5;

    camera_info->p[0] = 1;
    camera_info->p[2] = 1;
    camera_info->p[5] = 1;
    camera_info->p[6] = 1;
    camera_info->p[10] = 1;

    camera_info->roi.height = IMAGE_HEIGHT;
    camera_info->roi.width = IMAGE_WIDTH;
}

void init_visualization_msgs_marker(visualization_msgs__msg__Marker * marker){
    char string1[] = "/world";
    marker->header.frame_id.data = (char*)malloc(100*sizeof(char));
	memcpy(marker->header.frame_id.data, string1, strlen(string1) + 1);
	marker->header.frame_id.size = strlen(marker->header.frame_id.data);
	marker->header.frame_id.capacity = 100;
	
    char string2[] = "";
    marker->ns.data = (char*)malloc(100*sizeof(char));
	memcpy(marker->ns.data, string2, strlen(string2) + 1);
	marker->ns.size = strlen(marker->ns.data);
	marker->ns.capacity = 100;

    marker->id = 0;
    marker->type = visualization_msgs__msg__Marker__CUBE;
    marker->action = visualization_msgs__msg__Marker__ADD;

    marker->points.capacity = 0;
    marker->points.size = 0;

    marker->colors.capacity = 0;
    marker->colors.size = 0;
    
    marker->pose.position.x = 1;
    marker->pose.position.y = 1;
    marker->pose.position.z = 1;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    marker->scale.x = 1;
    marker->scale.y = 1;
    marker->scale.z = 1;

    marker->color.a = 1.0;
    marker->color.r = 0.0;
    marker->color.g = 1.0;
    marker->color.b = 0.0;

    char stringempty[] = "";
    marker->text.data = (char*)malloc(100*sizeof(char));
	memcpy(marker->text.data, stringempty, strlen(stringempty) + 1);
	marker->text.size = strlen(marker->text.data);
	marker->text.capacity = 100;

    marker->mesh_resource.data = (char*)malloc(100*sizeof(char));
	memcpy(marker->mesh_resource.data, stringempty, strlen(stringempty) + 1);
	marker->mesh_resource.size = strlen(marker->mesh_resource.data);
	marker->mesh_resource.capacity = 100;

    marker->lifetime.nanosec = 0;
    marker->lifetime.sec = 0;
}