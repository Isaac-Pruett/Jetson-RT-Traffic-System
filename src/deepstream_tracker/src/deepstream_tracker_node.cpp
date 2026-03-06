#include <rclcpp/rclcpp.hpp>


#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>


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

#include <string>
#include <vector>
#include <filesystem>
#include <mutex>
#include <unordered_map>

// Marco's Stuff
#define PGIE_CONFIG "/home/nvidia/Jetson-RT-Traffic-System/models/Primary_Detector/pgie_trafficcamnet_config.txt"
#define TRACKER_CONFIG "/opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/config_tracker_NvDCF_perf.yml"
#define TRACKER_LIB "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so"
#define VIDEO_W 960
#define VIDEO_H 544
//// 

using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::ObjectHypothesisWithPose;


class DeepStreamTrackerNode : public rclcpp::Node {
public:
    DeepStreamTrackerNode() : Node("deepstream_tracker_node") {
        this->declare_parameter("source_type", 0); 
        //this->declare_parameter("video_path", "/home/nvidia/Jetson-RT-Traffic-System/video/caltrans_captured.mp4"); 
        
        source_type_ = this->get_parameter("source_type").as_int();
        video_path_ = this->get_parameter("video_path").as_string();

        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable(); 
        
        rclcpp::QoS qos_image(1);
        qos_image.best_effort();

        pub_detect_ = create_publisher<vision_msgs::msg::Detection2DArray>("traffic_detections", qos_reliable);
        pub_summary_ = create_publisher<Detection2DArray>("tracked_object_summary", qos_reliable);
	pub_image_  = create_publisher<sensor_msgs::msg::Image>("image_raw", qos_image);
        
	//this is part of the old deepstream tracker used to 
        gst_init(nullptr, nullptr);
        
        first_frame_received_ = false;
        last_frame_time_ = this->now();
        last_image_pub_time_ = this->now();
        
        start_pipeline();
        
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

        // ==============================================================
        // CRITICAL FIX: Add core elements to the pipeline FIRST!
        // This ensures they share the same hierarchy before we link them.
        // ==============================================================
        gst_bin_add_many(GST_BIN(pipeline_), mux, infer, tracker, tee, NULL);
        gst_bin_add_many(GST_BIN(pipeline_), q_tracker, sink_tracker, NULL);
        gst_bin_add_many(GST_BIN(pipeline_), q_viz, conv, caps, sink_viz, NULL);

        if (source_type_ == 0 || source_type_ == 1) { 
            GstElement *source = gst_element_factory_make("nvurisrcbin", "uri-source");
            
            std::string uri = video_path_;
            if (source_type_ == 0 && uri.find("file://") != 0) {
                uri = "file://" + uri;
            }

            g_object_set(source, "uri", uri.c_str(), "num-extra-surfaces", 48, NULL);
            gst_bin_add(GST_BIN(pipeline_), source);
            g_signal_connect(source, "pad-added", G_CALLBACK(on_pad_added_mux), mux);
        } 
        else if (source_type_ == 2) {
            auto source = gst_element_factory_make("v4l2src", "usb-source");
            auto caps_v4l2 = gst_element_factory_make("capsfilter", "v4l2-caps");
            
            // hardware shock absorber queue to protect against USB jitter
            auto cam_q = gst_element_factory_make("queue", "cam_queue");
            
            auto nv_conv = gst_element_factory_make("nvvideoconvert", "usb-nv-conv");
            auto caps_nvmm = gst_element_factory_make("capsfilter", "nvmm-caps");

            g_object_set(source, "device", video_path_.c_str(), "do-timestamp", TRUE, NULL);

            GstCaps *cam_caps = gst_caps_from_string("video/x-raw, format=UYVY, width=1280, height=720, framerate=30/1");
            g_object_set(caps_v4l2, "caps", cam_caps, NULL);
            gst_caps_unref(cam_caps);

             
            // Force nvvideoconvert to use Unified Jetson Memory (type 4) to prevent segmentation faults
            g_object_set(nv_conv, "nvbuf-memory-type", 4, "compute-hw", 1, NULL);

            GstCaps *nv_caps = gst_caps_from_string("video/x-raw(memory:NVMM), format=NV12");
            g_object_set(caps_nvmm, "caps", nv_caps, NULL);
            gst_caps_unref(nv_caps);

            // Notice we removed 'cpu_conv' and replaced it with 'cam_q'
            gst_bin_add_many(GST_BIN(pipeline_), source, caps_v4l2, cam_q, nv_conv, caps_nvmm, NULL);
            gst_element_link_many(source, caps_v4l2, cam_q, nv_conv, caps_nvmm, NULL);

            GstPad *sinkpad = gst_element_request_pad_simple(mux, "sink_0");
            GstPad *srcpad = gst_element_get_static_pad(caps_nvmm, "src");
            
            GstPadLinkReturn link_ret = gst_pad_link(srcpad, sinkpad);
            if (link_ret != GST_PAD_LINK_OK) {
                RCLCPP_ERROR(get_logger(), "DEBUG: Failed to link camera to nvstreammux! Error code: %d", link_ret);
            } else {
                RCLCPP_INFO(get_logger(), "DEBUG: Successfully linked camera to nvstreammux.");
            }
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad);
        }
        // --- CONFIG ---
        g_object_set(sink_tracker, "sync", FALSE, "async", FALSE, "qos", FALSE, NULL);
        
        g_object_set(q_tracker, 
            "max-size-buffers", 10, "max-size-bytes", 0, "max-size-time", 0, "leaky", 0, NULL);
            
        g_object_set(q_viz, 
            "max-size-buffers", 10, "max-size-bytes", 0, "max-size-time", 0, "leaky", 0, NULL);
        
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

        g_object_set(sink_viz,
                     "sync", TRUE, "max-buffers", 2, "drop", TRUE, "emit-signals", TRUE, NULL);
                     
        g_signal_connect(sink_viz, "new-sample", G_CALLBACK(on_new_sample_viz), this);

        // ==============================================================
        // --- LINKING INTERNALS ---
        // (No gst_bin_add_many here, since we did it at the top)
        // ==============================================================
        gst_element_link_many(mux, infer, tracker, tee, NULL);
        gst_element_link_many(tee, q_tracker, sink_tracker, NULL);
        gst_element_link_many(tee, q_viz, conv, caps, sink_viz, NULL);

        GstPad *tracker_pad = gst_element_get_static_pad(sink_tracker, "sink");
        gst_pad_add_probe(tracker_pad, GST_PAD_PROBE_TYPE_BUFFER, probe_boxes, this, NULL);
        gst_object_unref(tracker_pad);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }
    static void on_pad_added_mux(GstElement *element, GstPad *pad, gpointer data) {
        GstElement *mux = (GstElement *)data; 
        GstPad *sinkpad = gst_element_request_pad_simple(mux, "sink_0");
        if (!gst_pad_is_linked(sinkpad)) {
            gst_pad_link(pad, sinkpad);
        }
        gst_object_unref(sinkpad);
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
                    RCLCPP_INFO(get_logger(), "EOS Reached. Rewinding video to loop...");
                    
                    last_frame_time_ = this->now();
                    
                    if (!gst_element_seek_simple(pipeline_, GST_FORMAT_TIME, 
                            (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_KEY_UNIT), 0)) {
                        RCLCPP_ERROR(get_logger(), "Failed to seek back to start. Shutting down.");
                        rclcpp::shutdown();
                    }
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
       
	if (!batch) {
		return GST_PAD_PROBE_OK;
	}
	
	vision_msgs::msg::Detection2DArray live_msg;
    	live_msg.header.stamp = node->now();
    	live_msg.header.frame_id = "camera";

    	const char *class_names[] = {"background", "bicycle", "car", "person", "road_sign"};

    	// Mark all tracks inactive first
    	{
        	std::lock_guard<std::mutex> lk(node->active_tracks_mutex_);
        	for (auto &entry : node->active_tracks_) {
            		entry.second.still_active = false;
    		}
	}

    	for (NvDsMetaList *l_frame = batch->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
        	NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);

        	for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
            		NvDsObjectMeta *obj = (NvDsObjectMeta *)(l_obj->data);
            		if (obj->confidence < 0.35f) continue;

            		uint64_t track_id = obj->object_id;
            		int cls = obj->class_id;
            		float conf = obj->confidence;

            		std::string class_name = (cls < 5) ? class_names[cls] : std::to_string(cls);

            		float cx = obj->rect_params.left + obj->rect_params.width / 2.0f;
            		float cy = obj->rect_params.top + obj->rect_params.height / 2.0f;

            		// ===== TRACK UPDATE =====
            		{
			    std::lock_guard<std::mutex> lk(node->active_tracks_mutex_);
			    auto it = node->active_tracks_.find(track_id);
			    
			    if (it == node->active_tracks_.end()) {
			    	TrackInfo t;
			    	t.class_name = class_name;
			    	t.max_confidence = conf;
				t.sum_confidence = conf;
			        t.count = 1;
				t.first_seen = node->now();
				t.last_seen = node->now();
				t.first_x = cx;
				t.first_y = cy;
				t.last_x = cx;
				t.last_y = cy;
				t.still_active = true;

				node->active_tracks_[track_id] = t;
			   } else {
			        TrackInfo &t = it->second;
			    	t.last_seen = node->now();
			    	t.max_confidence = std::max(t.max_confidence, conf);
			    	t.sum_confidence += conf;
			    	t.count++;
			    	t.last_x = cx;
			    	t.last_y = cy;
			    	t.still_active = true;
			   }
			}

			// ===== live detections =====
			vision_msgs::msg::Detection2D det;
			det.id = std::to_string(track_id);
			det.bbox.center.position.x = cx;
			det.bbox.center.position.y = cy;
			det.bbox.size_x = obj->rect_params.width;
			det.bbox.size_y = obj->rect_params.height;

			vision_msgs::msg::ObjectHypothesisWithPose hyp;
			hyp.hypothesis.class_id = class_name;
			hyp.hypothesis.score = conf;

			det.results.push_back(hyp);
			live_msg.detections.push_back(det);
		}
    }

    // ===== CHECK FOR ENDED TRACKS =====
    vision_msgs::msg::Detection2DArray summary_msg;
    summary_msg.header.stamp = node->now();
    summary_msg.header.frame_id = "camera";

    std::vector<uint64_t> to_remove;
    rclcpp::Time now = node->now();

    {
        std::lock_guard<std::mutex> lk(node->active_tracks_mutex_);

        for (auto &pair : node->active_tracks_) {
            uint64_t id = pair.first;
            TrackInfo &t = pair.second;

            if (!t.still_active) {
                if ((now - t.last_seen).seconds() >=
                    node->inactive_timeout_sec_)
                {
                    double avg_conf =
                        t.sum_confidence / t.count;

                    vision_msgs::msg::Detection2D det;
                    det.id = std::to_string(id);

                    det.bbox.center.position.x = t.last_x;
                    det.bbox.center.position.y = t.last_y;

                    vision_msgs::msg::ObjectHypothesisWithPose hyp;
                    hyp.hypothesis.class_id = t.class_name;
                    hyp.hypothesis.score = avg_conf;

                    // first position stored in pose
                    hyp.pose.pose.position.x = t.first_x;
                    hyp.pose.pose.position.y = t.first_y;

                    det.results.push_back(hyp);
                    summary_msg.detections.push_back(det);

                    to_remove.push_back(id);
                }
            }
        }

        for (auto id : to_remove)
            node->active_tracks_.erase(id);
    }

    if (!live_msg.detections.empty())
        node->pub_detect_->publish(live_msg);

    if (!summary_msg.detections.empty())
        node->pub_summary_->publish(summary_msg);

    return GST_PAD_PROBE_OK;
}
	
	// if (batch) {
       //     vision_msgs::msg::Detection2DArray msg;
       //     msg.header.stamp = node->now();
       //     const char *class_names[] = {"background", "bicycle", "car", "person", "road_sign"};
       //     for (NvDsMetaList *l_frame = batch->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
       //         NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);
       //        
       // 	vision_msgs::msg::Detection2DArray det_array;
       // 	det_array.header.stamp = node->now();
       // 	det_array.header.frame_id = "camera";

       // 	{
       //            // Mark all tracks inactive at the start of probe callback
       //            std::lock_guard<std::mutex> lk(node->active_tracks_mutex_);
       //            for (auto &entry : node->active_tracks_) {
       //                entry.second.still_active = false;
       //            }
       //     	}


       // 	// Object-update Loop
       //        	for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
       //             NvDsObjectMeta *obj = (NvDsObjectMeta *)(l_obj->data);
       //             
       // 	    std::string final_id = (obj->object_id == (guint64)-1) ? "0" : std::to_string(obj->object_id);
       //             if (obj->confidence < 0.35f) {
       // 		    continue;
       // 	    }

       //             vision_msgs::msg::Detection2D det;
       //             det.id = final_id;
       //             det.bbox.center.position.x = obj->rect_params.left + obj->rect_params.width/2;
       //             det.bbox.center.position.y = obj->rect_params.top + obj->rect_params.height/2;
       //             det.bbox.size_x = obj->rect_params.width;
       //             det.bbox.size_y = obj->rect_params.height;
       //             vision_msgs::msg::ObjectHypothesisWithPose hyp;
       //             int cls = obj->class_id;
       //             if (cls < 5) {
       // 		    hyp.hypothesis.class_id = class_names[cls];
       // 	    } else {
       // 		    hyp.hypothesis.class_id = std::to_string(cls);
       // 	    }
       // 	    hyp.hypothesis.score = obj->confidence;
       //             det.results.push_back(hyp);
       //             msg.detections.push_back(det);
       //         }
       //     }
       //     if (!msg.detections.empty()) node->pub_detect_->publish(msg);
       // }
       // return GST_PAD_PROBE_OK;
       //}

    static GstFlowReturn on_new_sample_viz(GstElement *sink, gpointer user_data) {
        auto *node = (DeepStreamTrackerNode *)user_data;
        
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) {
		return GST_FLOW_OK;
	}

	// Limit the frame rate
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
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_summary_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr bus_timer_;
    rclcpp::Time last_frame_time_;
    rclcpp::Time last_image_pub_time_;
    bool first_frame_received_;

    // Struck to track info
    struct TrackInfo {
    	std::string class_name;
    	float max_confidence{0.0f};
    	float sum_confidence{0.0f};
    	int count{0};

    	rclcpp::Time first_seen;
    	rclcpp::Time last_seen;

    	float first_x{0.0f};
    	float first_y{0.0f};
    	float last_x{0.0f};
    	float last_y{0.0f};

    	bool still_active{false};
    };

    std::unordered_map<uint64_t, TrackInfo> active_tracks_;
    std::mutex active_tracks_mutex_;
    const double inactive_timeout_sec_{1.0};

    GstElement *pipeline_{nullptr};
    int source_type_;
    std::string video_path_;
};

int main(int argc, char **argv) {
    // setenv("GST_DEBUG", "1", 1);
    // setenv("G_MESSAGES_DEBUG", "none", 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeepStreamTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
