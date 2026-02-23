#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include "nvdsmeta.h"
#include "nvbufsurface.h"
#include "nvdsinfer.h"
#include "gstnvdsmeta.h"
#include "nvdsmeta_schema.h"

using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::ObjectHypothesisWithPose;

cv::Mat convertToRGB8(const sensor_msgs::msg::Image::ConstSharedPtr& msg, rclcpp::Logger logger)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat frame;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
        return frame;
    }

    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        frame = cv_ptr->image;
    }
    else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2RGB);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_GRAY2RGB);
    }
    else if (msg->encoding == "nv12" || msg->encoding == "NV12") {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_YUV2RGB_NV12);
    }
    else if (msg->encoding == "yuv422" || msg->encoding == "YUV422") {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_YUV2RGB_Y422);
    }
    else {
        RCLCPP_WARN(logger, "Unhandled encoding: %s — passing through without conversion",
                    msg->encoding.c_str());
        frame = cv_ptr->image.clone();
    }

    return frame;
}

class DeepStreamTrackerNode : public rclcpp::Node
{
public:
    DeepStreamTrackerNode() : Node("deepstream_tracker_node")
    {
        declare_parameter<std::string>("source_topic", "");

        std::string pkg_share = ament_index_cpp::get_package_share_directory("deepstream_tracker");

        declare_parameter<std::string>("pgie_config", pkg_share + "/cfg/pgie_trafficcamnet_config.txt");
        // ADD THIS: SGIE config parameter
        declare_parameter<std::string>("sgie_config", pkg_share + "/cfg/sgie_vehicletypenet_config.txt");
        declare_parameter<std::string>("tracker_config", pkg_share + "/cfg/tracker_iou_config.txt");

        pub_ = create_publisher<Detection2DArray>("traffic_detections", 10);

        std::string source_topic = get_parameter("source_topic").as_string();
        if (source_topic.empty())
        {
            RCLCPP_FATAL(get_logger(), "Set source_topic to a valid input stream");
            rclcpp::shutdown();
            return;
        }

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            source_topic,
            10,
            std::bind(&DeepStreamTrackerNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "DeepStream tracker node started");
    }

    ~DeepStreamTrackerNode() override
    {
        if (pipeline_)
        {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

private:
GstElement *build_pipeline(int width, int height)
{
    std::string pgie = get_parameter("pgie_config").as_string();
    std::string sgie = get_parameter("sgie_config").as_string();
    std::string tracker_cfg = get_parameter("tracker_config").as_string();

    gst_init(nullptr, nullptr);

    // Create GStreamer elements
    pipeline_ = gst_pipeline_new("ds-pipeline");
    appsrc_ = gst_element_factory_make("appsrc", "source");
    auto videoconvert = gst_element_factory_make("videoconvert", "videoconvert");
    auto nvvconv_to_nvmm = gst_element_factory_make("nvvideoconvert", "nvvconv_to_nvmm");
    auto streammux = gst_element_factory_make("nvstreammux", "nvstreammux");
    auto pgie_elt = gst_element_factory_make("nvinfer", "primary-nvinfer");
    auto sgie_elt = gst_element_factory_make("nvinfer", "secondary-nvinfer");
    auto tracker = gst_element_factory_make("nvtracker", "tracker");
    auto sink = gst_element_factory_make("fakesink", "sink");

    if (!pipeline_ || !appsrc_ || !videoconvert || !nvvconv_to_nvmm ||
    !streammux || !pgie_elt || !sgie_elt || !tracker || !sink)
    {
        RCLCPP_FATAL(get_logger(), "Failed to create GStreamer elements");
        return nullptr;
    }

    // Appsrc caps
    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "RGB",
        "width", G_TYPE_INT, width,
        "height", G_TYPE_INT, height,
        NULL);
    g_object_set(appsrc_, "caps", caps, "format", GST_FORMAT_TIME, "is-live", TRUE, NULL);
    gst_caps_unref(caps);

    // NV video conversion
    g_object_set(G_OBJECT(nvvconv_to_nvmm),
                 "gpu-id", 0,
                 "nvbuf-memory-type", 0,
                 "compute-hw", 1,
                 NULL);

    // Streammux settings
    g_object_set(G_OBJECT(streammux),
                 "batch-size", 1,
                 "width", width,
                 "height", height,
                 "batched-push-timeout", 40000,
                 NULL);

    // nvinfer & tracker settings
    g_object_set(G_OBJECT(pgie_elt), "config-file-path", pgie.c_str(), NULL);
    g_object_set(G_OBJECT(sgie_elt), "config-file-path", sgie.c_str(), NULL);

    g_object_set(G_OBJECT(tracker), "ll-config-file", tracker_cfg.c_str(), NULL);
    g_object_set(G_OBJECT(tracker), "ll-lib-file", "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so", NULL);
    g_object_set(G_OBJECT(tracker),
                 "tracker-width", 640,
                 "tracker-height", 384,
                 "gpu-id", 0,
                 NULL);

    // Add elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, videoconvert, nvvconv_to_nvmm,
                 streammux, pgie_elt, tracker, sgie_elt, sink, NULL);

    // Link CPU elements
    if (!gst_element_link(appsrc_, videoconvert))
    {
        RCLCPP_FATAL(get_logger(), "Failed to link appsrc -> videoconvert");
        return nullptr;
    }
    if (!gst_element_link(videoconvert, nvvconv_to_nvmm))
    {
        RCLCPP_FATAL(get_logger(), "Failed to link videoconvert -> nvvconv_to_nvmm");
        return nullptr;
    }

    // Link to streammux using request_pad_simple
    GstPad *srcpad = gst_element_get_static_pad(nvvconv_to_nvmm, "src");
    GstPad *sinkpad = gst_element_request_pad_simple(streammux, "sink_0");
    if (!sinkpad)
    {
        RCLCPP_FATAL(get_logger(), "Streammux request sink pad failed");
        gst_object_unref(srcpad);
        return nullptr;
    }
    if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK)
    {
        RCLCPP_FATAL(get_logger(), "Failed to link nvvconv_to_nvmm to streammux");
        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);
        return nullptr;
    }
    gst_object_unref(srcpad);
    gst_object_unref(sinkpad);

    // GPU pipeline: streammux -> PGIE -> tracker -> SGIE
	if (!gst_element_link(streammux, pgie_elt))
	{
	    RCLCPP_FATAL(get_logger(), "Failed to link streammux -> pgie_elt");
	    return nullptr;
	}
	if (!gst_element_link(pgie_elt, tracker))
	{
	    RCLCPP_FATAL(get_logger(), "Failed to link pgie_elt -> tracker");
	    return nullptr;
	}
	if (!gst_element_link(tracker, sgie_elt))
	{
	    RCLCPP_FATAL(get_logger(), "Failed to link tracker -> sgie_elt");
	    return nullptr;
	}
	if (!gst_element_link(sgie_elt, sink))
	{
	    RCLCPP_FATAL(get_logger(), "Failed to link sgie_elt -> sink");
	    return nullptr;
	}


	GstPad *sgie_src_pad = gst_element_get_static_pad(sgie_elt, "src");
	if (sgie_src_pad)
	{
	    gst_pad_add_probe(sgie_src_pad,
		              GST_PAD_PROBE_TYPE_BUFFER,
		              (GstPadProbeCallback)tracker_src_pad_buffer_probe,
		              this,
		              NULL);
	}

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    RCLCPP_INFO(get_logger(), "Pipeline built successfully!");
    return pipeline_;
}


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = convertToRGB8(msg, this->get_logger());

        if (!has_set_up && !frame.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Received first frame, generating pipeline with size: %dx%d", frame.cols, frame.rows);
            build_pipeline(frame.cols, frame.rows);
            has_set_up = true;
        }

        GstBuffer *buffer = gst_buffer_new_allocate(NULL, frame.total() * frame.elemSize(), NULL);
        GstMapInfo map;

        if (gst_buffer_map(buffer, &map, GST_MAP_WRITE))
        {
            memcpy(map.data, frame.data, frame.total() * frame.elemSize());
            gst_buffer_unmap(buffer, &map);
        }

        uint64_t ns = (uint64_t)msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
        GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(ns, GST_SECOND, (guint64)1000000000ULL);
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, 30);
        
        gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    }

    static GstPadProbeReturn tracker_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    auto *node = static_cast<DeepStreamTrackerNode *>(user_data);

    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf)
        return GST_PAD_PROBE_OK;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (!batch_meta)
        return GST_PAD_PROBE_OK;

    // PGIE class labels
    const std::vector<std::string> ids = {"car", "bicycle", "person", "road_sign"};

    for (NvDsMetaList *l_frame = batch_meta->frame_meta_list; l_frame != nullptr; l_frame = l_frame->next)
    {
        NvDsFrameMeta *frame_meta = static_cast<NvDsFrameMeta *>(l_frame->data);
        vision_msgs::msg::Detection2DArray det_array;
        det_array.header.stamp = node->now();
        det_array.header.frame_id = "camera";

        RCLCPP_INFO(node->get_logger(), "Frame has %d objects", frame_meta->num_obj_meta);

        for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next)
        {
            NvDsObjectMeta *obj_meta = static_cast<NvDsObjectMeta *>(l_obj->data);

            // --- Safely get PGIE class name ---
            std::string class_name = "unknown";
            if (obj_meta->class_id >= 0 && static_cast<size_ft>(obj_meta->class_id) < ids.size())
                class_name = ids[obj_meta->class_id];

            // --- Safely get SGIE vehicle type ---
            std::string vehicle_type = "unknown";
            for (NvDsMetaList *l_classifier = obj_meta->classifier_meta_list; l_classifier != nullptr; l_classifier = l_classifier->next)
            {
                NvDsClassifierMeta *classifier_meta = static_cast<NvDsClassifierMeta *>(l_classifier->data);
                if (!classifier_meta)
                    continue;

                for (NvDsMetaList *l_label = classifier_meta->label_info_list; l_label != nullptr; l_label = l_label->next)
                {
                    NvDsLabelInfo *label_info = static_cast<NvDsLabelInfo *>(l_label->data);
                    if (!label_info || !label_info->result_label)
                        continue;

                    if (label_info->result_prob > 0.2)
                    {
                        vehicle_type = label_info->result_label;
                        break;
                    }
                }

                if (vehicle_type != "unknown")
                    break;
            }

            std::string overlay_text = class_name;
            if (!vehicle_type.empty())
                overlay_text += ":" + vehicle_type;

            RCLCPP_INFO(node->get_logger(),
                        "Object %lu: %s (conf=%.2f, type=%s)",
                        obj_meta->object_id,
                        class_name.c_str(),
                        obj_meta->confidence,
                        vehicle_type.c_str());

            // --- Fill ROS Detection2D message ---
            vision_msgs::msg::Detection2D det;
            det.bbox.center.position.x = obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0;
            det.bbox.center.position.y = obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0;
            det.bbox.size_x = obj_meta->rect_params.width;
            det.bbox.size_y = obj_meta->rect_params.height;
            det.id = std::to_string(obj_meta->object_id);

            ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = overlay_text;
            hyp.hypothesis.score = obj_meta->confidence;

            det.results.push_back(hyp);
            det_array.detections.push_back(det);

            // --- Draw on screen ---
            NvDsDisplayMeta *display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
            display_meta->num_labels = 1;
            NvOSD_TextParams *txt_params = &display_meta->text_params[0];

            txt_params->display_text = g_strdup(overlay_text.c_str());
            txt_params->x_offset = obj_meta->rect_params.left;
            txt_params->y_offset = obj_meta->rect_params.top - 10;
            txt_params->font_params.font_name = g_strdup("Serif");
            txt_params->font_params.font_size = 12;
            txt_params->font_params.font_color = (NvOSD_ColorParams){1.0, 1.0, 1.0, 1.0};
            txt_params->set_bg_clr = 1;
            txt_params->text_bg_clr = (NvOSD_ColorParams){0.0, 0.0, 0.0, 1.0};

            nvds_add_display_meta_to_frame(frame_meta, display_meta);
        }

        node->pub_->publish(det_array);
    }

    return GST_PAD_PROBE_OK;
}

    bool has_set_up{false};

    rclcpp::Publisher<Detection2DArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    GstElement *pipeline_{nullptr};
    GstElement *appsrc_{nullptr};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeepStreamTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
