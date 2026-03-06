#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include "nvdsmeta.h"
#include "nvbufsurface.h"
#include "nvdsinfer.h"
#include "gstnvdsmeta.h"
#include <mutex>

#define PGIE_CONFIG "/home/nvidia/Jetson-RT-Traffic-System/models/Primary_Detector/pgie_trafficcamnet_config.txt"
#define TRACKER_CONFIG "/opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/config_tracker_NvDCF_perf.yml"
#define TRACKER_LIB "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so"
#define VIDEO_W 960
#define VIDEO_H 544

class DeepStreamTrackerNode : public rclcpp::Node {
public:
    DeepStreamTrackerNode() : Node("deepstream_tracker_node") {
        this->declare_parameter("source_type", 0); 
        this->declare_parameter("video_path", "/home/nvidia/Jetson-RT-Traffic-System/video/fixed_vid.mp4"); // Use the FFmpeg-fixed video!
        
        source_type_ = this->get_parameter("source_type").as_int();
        video_path_ = this->get_parameter("video_path").as_string();

        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable(); 
        
        rclcpp::QoS qos_image(1);
        qos_image.best_effort();

        pub_detect_ = create_publisher<vision_msgs::msg::Detection2DArray>("traffic_detections", qos_reliable);
        pub_image_  = create_publisher<sensor_msgs::msg::Image>("image_raw", qos_image);
        
        gst_init(nullptr, nullptr);
        start_pipeline();
        
        first_frame_received_ = false;
        last_frame_time_ = this->now();
        last_image_pub_time_ = this->now();
        
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                if (this->first_frame_received_ && (this->now() - this->last_frame_time_).seconds() > 10.0) {
                    RCLCPP_ERROR(this->get_logger(), "WATCHDOG TRIGGERED: Pipeline silently froze for 10 seconds. Shutting down.");
                    rclcpp::shutdown();
                }
            }
        );
        
        bus_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&DeepStreamTrackerNode::bus_callback, this));

        RCLCPP_INFO(get_logger(), "--- DEEPSTREAM MASTER NODE ---");
    }

    ~DeepStreamTrackerNode() {
        if (pipeline_) { 
            gst_element_set_state(pipeline_, GST_STATE_NULL); 
            gst_object_unref(pipeline_); 
        }
    }

private:
    void start_pipeline() {
        pipeline_ = gst_pipeline_new("ds-tracker-pipeline");
        
        auto mux     = gst_element_factory_make("nvstreammux", "mux");
        auto infer   = gst_element_factory_make("nvinfer", "infer");
        auto tracker = gst_element_factory_make("nvtracker", "tracker");
        auto tee     = gst_element_factory_make("tee", "splitter");

        auto q_tracker = gst_element_factory_make("queue", "q_tracker");
        auto sink_tracker = gst_element_factory_make("fakesink", "sink_tracker");

        auto q_viz      = gst_element_factory_make("queue", "q_viz");
        auto conv       = gst_element_factory_make("nvvideoconvert", "conv"); 
        auto caps       = gst_element_factory_make("capsfilter", "caps");
        auto sink_viz   = gst_element_factory_make("appsink", "sink_viz"); 

        GstElement *source = nullptr, *demux = nullptr, *parser = nullptr, *decoder = nullptr, *dec_conv = nullptr;

        if (source_type_ == 0 || source_type_ == 1) { 
            if (source_type_ == 0) {
                 source = gst_element_factory_make("filesrc", "file-source");
                 demux  = gst_element_factory_make("qtdemux", "demuxer");
            } else {
                 source = gst_element_factory_make("souphttpsrc", "http-source");
                 demux  = gst_element_factory_make("hlsdemux", "hls-demux");
                 g_object_set(source, "is-live", TRUE, NULL);
            }
            g_object_set(source, "location", video_path_.c_str(), NULL);
            
            parser = gst_element_factory_make("h264parse", "parser");
            decoder = gst_element_factory_make("nvv4l2decoder", "nv-decoder");
            g_object_set(decoder, "enable-max-performance", 1, "num-extra-surfaces", 48, NULL);
            
            dec_conv = gst_element_factory_make("nvvideoconvert", "dec-conv");
            g_object_set(dec_conv, "compute-hw", 1, "nvbuf-memory-type", 4, NULL);
        } 

        // --- CONFIG ---
        // NATIVE SYNC RESTORED: GStreamer will now pace playback perfectly to 1x real-time
        g_object_set(sink_tracker, "sync", FALSE, "qos", FALSE, NULL);
        
        g_object_set(q_tracker, 
            "max-size-buffers", 10, 
            "max-size-bytes", 0, 
            "max-size-time", 0, 
            "leaky", 0, NULL);
            
        g_object_set(q_viz, 
            "max-size-buffers", 10, 
            "max-size-bytes", 0, 
            "max-size-time", 0, 
            "leaky", 0, NULL);
        
        bool is_live = (source_type_ != 0);
        g_object_set(mux, "batch-size", 1, "width", VIDEO_W, "height", VIDEO_H, 
                     "batched-push-timeout", 40000, 
                     "nvbuf-memory-type", 4, 
                     "live-source", is_live ? 1 : 0, 
                     "enable-padding", 1, NULL); 
        
        g_object_set(infer, "config-file-path", PGIE_CONFIG, NULL);
        g_object_set(tracker, "ll-config-file", TRACKER_CONFIG, "ll-lib-file", TRACKER_LIB,
                     "tracker-width", VIDEO_W, "tracker-height", VIDEO_H, 
                     "compute-hw", 1, "gpu-id", 0, NULL);
        
        g_object_set(conv, "compute-hw", 1, "nvbuf-memory-type", 0, NULL);
        
        GstCaps *caps_cfg = gst_caps_from_string("video/x-raw, format=RGBA"); 
        g_object_set(caps, "caps", caps_cfg, NULL);
        gst_caps_unref(caps_cfg);

        // NATIVE SYNC RESTORED for the appsink as well
        g_object_set(sink_viz,
                     "sync", TRUE,
                     "max-buffers", 2,
                     "drop", TRUE,
                     "emit-signals", TRUE, NULL);
                     
        g_signal_connect(sink_viz, "new-sample", G_CALLBACK(on_new_sample_viz), this);

        // --- LINKING ---
        gst_bin_add_many(GST_BIN(pipeline_), mux, infer, tracker, tee, NULL);
        gst_element_link_many(mux, infer, tracker, tee, NULL);

        gst_bin_add_many(GST_BIN(pipeline_), q_tracker, sink_tracker, NULL);
        gst_element_link_many(tee, q_tracker, sink_tracker, NULL);
        
        gst_bin_add_many(GST_BIN(pipeline_), q_viz, conv, caps, sink_viz, NULL);
        gst_element_link_many(tee, q_viz, conv, caps, sink_viz, NULL);

        if (source_type_ == 0 || source_type_ == 1) { 
            gst_bin_add_many(GST_BIN(pipeline_), source, demux, parser, decoder, dec_conv, NULL);
            gst_element_link(source, demux);
            gst_element_link_many(parser, decoder, dec_conv, NULL);
            g_signal_connect(demux, "pad-added", G_CALLBACK(on_pad_added), parser);
            
            GstPad *sinkpad = gst_element_request_pad_simple(mux, "sink_0");
            GstPad *srcpad = gst_element_get_static_pad(dec_conv, "src");
            gst_pad_link(srcpad, sinkpad);
            gst_object_unref(srcpad); gst_object_unref(sinkpad);
        }

        GstPad *tracker_pad = gst_element_get_static_pad(sink_tracker, "sink");
        gst_pad_add_probe(tracker_pad, GST_PAD_PROBE_TYPE_BUFFER, probe_boxes, this, NULL);
        gst_object_unref(tracker_pad);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    static void on_pad_added(GstElement *element, GstPad *pad, gpointer data) {
        GstElement *target = (GstElement *)data; 
        GstCaps *caps = gst_pad_query_caps(pad, NULL);
        if (!caps || gst_caps_get_size(caps) == 0) { if(caps) gst_caps_unref(caps); return; }
        
        const gchar *name = gst_structure_get_name(gst_caps_get_structure(caps, 0));
        if (g_str_has_prefix(name, "video")) {
            GstPad *sinkpad = gst_element_get_static_pad(target, "sink");
            if (!gst_pad_is_linked(sinkpad)) gst_pad_link(pad, sinkpad);
            gst_object_unref(sinkpad);
        }
        gst_caps_unref(caps);
    }

    void bus_callback() {
        GstBus *bus = gst_element_get_bus(pipeline_);
        while (GstMessage *msg = gst_bus_pop(bus)) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError *err = NULL;
                    gchar *debug_info = NULL;
                    gst_message_parse_error(msg, &err, &debug_info);
                    RCLCPP_ERROR(get_logger(), "Pipeline Error: %s", err->message);
                    g_clear_error(&err);
                    g_free(debug_info);
                    break;
                }
                case GST_MESSAGE_EOS:
                    RCLCPP_INFO(get_logger(), "EOS Reached. Shutting down.");
                    rclcpp::shutdown();
                    break;
                default:
                    break;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }

    static GstPadProbeReturn probe_boxes(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        auto *node = (DeepStreamTrackerNode *)user_data;
        node->first_frame_received_ = true;
        node->last_frame_time_ = node->now();

        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        NvDsBatchMeta *batch = gst_buffer_get_nvds_batch_meta(buf);
        if (batch) {
            vision_msgs::msg::Detection2DArray msg;
            msg.header.stamp = node->now();
            const char *class_names[] = {"background", "bicycle", "car", "person", "road_sign"};
            for (NvDsMetaList *l_frame = batch->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
                NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);
                for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
                    NvDsObjectMeta *obj = (NvDsObjectMeta *)(l_obj->data);
                    std::string final_id = (obj->object_id == (guint64)-1) ? "0" : std::to_string(obj->object_id);
                    if (obj->confidence < 0.35f) continue;
                    vision_msgs::msg::Detection2D det;
                    det.id = final_id;
                    det.bbox.center.position.x = obj->rect_params.left + obj->rect_params.width/2;
                    det.bbox.center.position.y = obj->rect_params.top + obj->rect_params.height/2;
                    det.bbox.size_x = obj->rect_params.width;
                    det.bbox.size_y = obj->rect_params.height;
                    vision_msgs::msg::ObjectHypothesisWithPose hyp;
                    int cls = obj->class_id;
                    if (cls < 5) hyp.hypothesis.class_id = class_names[cls];
                    else hyp.hypothesis.class_id = std::to_string(cls);
                    hyp.hypothesis.score = obj->confidence;
                    det.results.push_back(hyp);
                    msg.detections.push_back(det);
                }
            }
            if (!msg.detections.empty()) node->pub_detect_->publish(msg);
        }
        return GST_PAD_PROBE_OK;
    }

    static GstFlowReturn on_new_sample_viz(GstElement *sink, gpointer user_data) {
        auto *node = (DeepStreamTrackerNode *)user_data;
        
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) return GST_FLOW_OK;

        if (node->pub_image_->get_subscription_count() > 0) {
            rclcpp::Time now = node->now();
            if ((now - node->last_image_pub_time_).seconds() >= 0.033) {
                node->last_image_pub_time_ = now;
                
                GstBuffer *buf = gst_sample_get_buffer(sample);
                GstMapInfo map;
                if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
                    auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
                    img_msg->header.stamp = node->now();
                    img_msg->header.frame_id = "camera_frame";
                    img_msg->height = VIDEO_H;
                    img_msg->width = VIDEO_W;
                    img_msg->encoding = "rgba8";
                    img_msg->step = VIDEO_W * 4;
                    img_msg->data.resize(map.size);
                    memcpy(img_msg->data.data(), map.data, map.size);
                    
                    node->pub_image_->publish(std::move(img_msg));
                    gst_buffer_unmap(buf, &map);
                }
            }
        }
        
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detect_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr bus_timer_;
    rclcpp::Time last_frame_time_;
    rclcpp::Time last_image_pub_time_;
    bool first_frame_received_;
    GstElement *pipeline_{nullptr};
    int source_type_;
    std::string video_path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeepStreamTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
