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

#include <string>
#include <vector>
#include <filesystem>
#include <mutex>

using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::ObjectHypothesisWithPose;

cv::Mat convertToRGB8(const sensor_msgs::msg::Image::ConstSharedPtr &msg, rclcpp::Logger logger) {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat frame;

    try {
        // First copy in its native encoding
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
        return frame; // empty
    }

    // Handle cases
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        frame = cv_ptr->image; // already good
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2RGB);
    } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_GRAY2RGB);
    } else if (msg->encoding == "nv12" || msg->encoding == "NV12") {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_YUV2RGB_NV12);
    } else if (msg->encoding == "yuv422" || msg->encoding == "YUV422") {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_YUV2RGB_Y422);
    } else {
        RCLCPP_WARN(logger, "Unhandled encoding: %s — passing through without conversion",
                    msg->encoding.c_str());
        frame = cv_ptr->image.clone();
    }

    return frame;
}


class DeepStreamTrackerNode : public rclcpp::Node {
public:
    DeepStreamTrackerNode() : Node("deepstream_tracker_node") {
        declare_parameter<std::string>("source_topic", "");

        std::string pkg_share = ament_index_cpp::get_package_share_directory("deepstream_tracker");

        declare_parameter<std::string>("pgie_config", pkg_share + "/cfg/pgie_trafficcamnet_config.txt");
        declare_parameter<std::string>("tracker_config", pkg_share + "/cfg/tracker_iou_config.txt");

        pub_ = create_publisher<Detection2DArray>("traffic_detections", 10);
	summary_pub_ = create_publisher<Detection2DArray>("tracked_object_summary", 10);
        
	std::string source_topic = get_parameter("source_topic").as_string();
        if (source_topic.empty()) {
            RCLCPP_FATAL(get_logger(), "Set source_topic to a valid input stream");
            rclcpp::shutdown();
            return;
        }

        sub_ = create_subscription<sensor_msgs::msg::Image>(source_topic, 10, std::bind(&DeepStreamTrackerNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "DeepStream Tracker Node started");
    }



    ~DeepStreamTrackerNode() override {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

private:
    GstElement *build_pipeline(int width, int height) {
        std::string pgie = get_parameter("pgie_config").as_string();
        std::string tracker_cfg = get_parameter("tracker_config").as_string();
        gst_init(nullptr, nullptr);

        // Create GStreamer elements
        pipeline_ = gst_pipeline_new("ds-pipeline");
        appsrc_ = gst_element_factory_make("appsrc", "source");
        auto videoconvert = gst_element_factory_make("videoconvert", "videoconvert");         // CPU conversion
        auto nvvconv_to_nvmm = gst_element_factory_make("nvvideoconvert", "nvvconv_to_nvmm"); // CPU -> GPU
        // auto capsfilter = gst_element_factory_make("capsfilter", "capsfilter");           // Force NV12
        auto streammux = gst_element_factory_make("nvstreammux", "nvstreammux");
        auto pgie_elt = gst_element_factory_make("nvinfer", "primary-nvinfer");
        auto tracker = gst_element_factory_make("nvtracker", "tracker");
        // auto nvvconv = gst_element_factory_make("nvvideoconvert", "conv");
        // auto sink = gst_element_factory_make("fakesink", "fakesink");

        if (!pipeline_ || !appsrc_ || !videoconvert || !nvvconv_to_nvmm || !streammux || !pgie_elt || !tracker) {
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

        // nv video conversion on gpu memoty
        g_object_set(G_OBJECT(nvvconv_to_nvmm),
                     "gpu-id", 0,
                     "nvbuf-memory-type", 0, // non-pinned CPU mem
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
        g_object_set(G_OBJECT(tracker), "ll-config-file", tracker_cfg.c_str(), NULL);
        g_object_set(G_OBJECT(tracker), "ll-lib-file", "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so", NULL);
        g_object_set(G_OBJECT(tracker),
                     "tracker-width", 640,
                     "tracker-height", 384,
                     "gpu-id", 0,
                     NULL);
        // Add elements to pipeline
        gst_bin_add_many(GST_BIN(pipeline_),
                         appsrc_, videoconvert, nvvconv_to_nvmm,
                         streammux, pgie_elt, tracker, NULL);

        // Link CPU elements one by one
        if (!gst_element_link(appsrc_, videoconvert)) {
            RCLCPP_FATAL(get_logger(), "Failed to link appsrc -> videoconvert");
            perror("link err: ");
            return nullptr;
        }

        if (!gst_element_link(videoconvert, nvvconv_to_nvmm)) {
            RCLCPP_FATAL(get_logger(), "Failed to link videoconvert -> nvvconv_to_nvmm");
            perror("link err: ");
            return nullptr;
        }

        // Get sink pad from nvstreammux
        GstPad *sinkpad, *srcpad;

        // nvvideoconvert's src pad
        srcpad = gst_element_get_static_pad(nvvconv_to_nvmm, "src");

        // streammux sink pad (sink_0 for first source)
        sinkpad = gst_element_get_request_pad(streammux, "sink_0");
        if (!sinkpad) {
            RCLCPP_FATAL(get_logger(), "Streammux request sink pad failed. Exiting.\n");
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad);
            return nullptr;
        }

        // Link the two pads
        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            RCLCPP_FATAL(get_logger(), "Failed to link nvvconv_to_nvmm to streammux\n");
            gst_object_unref(srcpad);
            gst_object_unref(sinkpad);
            return nullptr;
        }

        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);

        // Link downstream GPU pipeline one by one
        if (!gst_element_link(streammux, pgie_elt)) {
            RCLCPP_FATAL(get_logger(), "Failed to link streammux -> pgie_elt");
            perror("link err: ");
            return nullptr;
        }




        if (!gst_element_link(pgie_elt, tracker)) {
            RCLCPP_FATAL(get_logger(), "Failed to link pgie_elt -> tracker");
            perror("link err: ");
            return nullptr;
        }




        // Add probe on tracker src pad
        GstPad *tracker_src_pad = gst_element_get_static_pad(tracker, "src");
        if (!tracker_src_pad) {
            RCLCPP_FATAL(get_logger(), "Unable to get tracker src pad");
        }




        else {
            gst_pad_add_probe(tracker_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                              (GstPadProbeCallback)tracker_src_pad_buffer_probe,
                              this, NULL);
        }

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        RCLCPP_INFO(get_logger(), "Pipeline built successfully!");
        return pipeline_;
    }



    // image_callback function
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = convertToRGB8(msg, this->get_logger());

        if (!has_set_up && !frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Recieved first frame, generating pipeline with size: %dx%d", frame.cols, frame.rows);
            build_pipeline(frame.cols, frame.rows);
            has_set_up = true;
        }

        GstBuffer *buffer = gst_buffer_new_allocate(NULL, frame.total() * frame.elemSize(), NULL);
        GstMapInfo map;

        if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
            memcpy(map.data, frame.data, frame.total() * frame.elemSize());
            gst_buffer_unmap(buffer, &map);
        }

        uint64_t ns = (uint64_t)msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
        GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(ns, GST_SECOND, (guint64)1000000000ULL);

        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, 30); // assume 30fps
        gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    }

    // GstPadProbe function
    static GstPadProbeReturn tracker_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        auto *node = static_cast<DeepStreamTrackerNode *>(user_data);

        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buf) {
            return GST_PAD_PROBE_OK;
        }

        NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
        if (!batch_meta) {
            return GST_PAD_PROBE_OK;
        }

        for (NvDsMetaList *l_frame = batch_meta->frame_meta_list; l_frame != nullptr; l_frame = l_frame->next) {

            // Struct that contains DeepStream's internal frame info
            // Accessible variables:
            // 	frame_meta->frame_num    .- Frame index, increasing every frame
            // 	frame_meta->buf_pts      - Timestamps of that frame
            // 	frame_meta->num_obj_meta - Number of tracked objects in the frame
            NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);

            const char *ids[] = {
                "car",
                "bicycle",
                "person",
                "road_sign"};


            vision_msgs::msg::Detection2DArray det_array;
            det_array.header.stamp = node->now();
            det_array.header.frame_id = "camera"; // or use your Image header

            {
                // Mark all tracks inactive at the start of probe callback
                std::lock_guard<std::mutex> lk(node->active_tracks_mutex_);
                for (auto &entry : node->active_tracks_) {
                    entry.second.still_active = false;
                }
            }

            // Object-update Loop
            for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
                NvDsObjectMeta *obj_meta = (NvDsObjectMeta *)(l_obj->data);

                // Onject ID Tracker Terminal Display
                uint64_t track_id = obj_meta->object_id;
                int class_id = obj_meta->class_id;
                float conf = obj_meta->confidence;

                // Check class_id < num of ids
                std::string class_name = "unknown";
                if (class_id >= 0 && class_id < (int)(sizeof(ids) / sizeof(ids[0]))) {
                    class_name = ids[class_id];
                }

                {
                    std::lock_guard<std::mutex> lk(node->active_tracks_mutex_);
                    auto it = node->active_tracks_.find(track_id);
                    if (it == node->active_tracks_.end()) {
                        // new object
                        TrackInfo track;
                        track.class_name = class_name;
                        track.max_confidence = conf;
                        track.sum_confidence = conf; // cumulative
                        track.count = 1;
                        track.first_seen = node->now();
                        track.last_seen = node->now();
                        track.still_active = true;
                 
			// First-seen pixel
			track.first_x = obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0f;
			track.first_y = obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0f;

			// Last-seen same as first-seen
			track.last_x = track.first_x;
			track.last_y = track.first_y;
                        
			node->active_tracks_.emplace(track_id, std::move(track));
		    } else {
                        // Existing object
                        TrackInfo &track = it->second;
                        track.last_seen = node->now();
                        track.max_confidence = std::max(track.max_confidence, conf);
                        track.sum_confidence += conf;
                        track.count++;
                        track.still_active = true;
                
			// Update last-seen pixels
			track.last_x = obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0f;
		    	track.last_y = obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0f;
		    }
                }

		// Publish Track Summary When Track Ends
		std::vector<uint64_t> to_remove;
		rclcpp::Time now = node->now();

		vision_msgs::msg::Detection2DArray summary_array;
		summary_array.header.stamp = now;
		summary_array.header.frame_id = "camera";
		
		for (auto &pair : node->active_tracks_) {
			uint64_t id = pair.first;
			TrackInfo &track = pair.second;

			if (!track.still_active) {
				rclcpp::Duration absent = now - track.last_seen;
				if (absent.seconds() >= node->inactive_timeout_sec_) {
					double duration = (track.last_seen - track.first_seen).seconds();
					double avg_conf = track.sum_confidence / track.count;

					vision_msgs::msg::Detection2D det;
					det.id = std::to_string(id);

					// Last-seen pixel (final position)
					det.bbox.center.position.x = track.last_x;
					det.bbox.center.position.y = track.last_y;
	
					ObjectHypothesisWithPose hyp;
            				hyp.hypothesis.class_id = track.class_name;
            				hyp.hypothesis.score = avg_conf;

            				// FIRST-SEEN pixel (initial position) stored here
            				hyp.pose.pose.position.x = track.first_x;
           				hyp.pose.pose.position.y = track.first_y;
            				hyp.pose.pose.position.z = 0.0;

					det.results.emplace_back(hyp);
					summary_array.detections.emplace_back(det);

					to_remove.push_back(id);
				}
			
			}
		
		}

		// Publish summaries
		if (!summary_array.detections.empty()) {
			node->summary_pub_->publish(summary_array);
		}

		// Clean up
		for (uint64_t id : to_remove) {
			node->active_tracks_.erase(id);
		}

                // Fill 2d Detection
                vision_msgs::msg::Detection2D det;
                det.bbox.center.position.x = obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0;
                det.bbox.center.position.y = obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0;
                det.bbox.size_x = obj_meta->rect_params.width;
                det.bbox.size_y = obj_meta->rect_params.height;

                ObjectHypothesisWithPose hyp;
                hyp.hypothesis.class_id = class_name;
                hyp.hypothesis.score = conf;

                det.id = std::to_string(obj_meta->object_id);

                det.results.emplace_back(hyp);
                det_array.detections.emplace_back(det);
            }

        node->pub_->publish(det_array);
    }
    return GST_PAD_PROBE_OK;
}



  bool has_set_up{false};

  // Adding Publishers and Subscribers
  rclcpp::Publisher<Detection2DArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<Detection2DArray>::SharedPtr summary_pub_;

  GstElement *pipeline_{nullptr};
  GstElement *appsrc_{nullptr};

  // Struct to track object
  struct TrackInfo {
	std::string class_name;
	float max_confidence{0.0f};
	float sum_confidence{0.0f};
	int count{0};
	rclcpp::Time first_seen;
	rclcpp::Time last_seen;

	// Store Pixel Positions
	float first_x{0.0f};
	float first_y{0.0f};
	float last_x{0.0f};
	float last_y{0.0f};
	bool still_active{false};
  };

  std::unordered_map<uint64_t, TrackInfo> active_tracks_;
  std::mutex active_tracks_mutex_;
  const double inactive_timeout_sec_{1.0}; // Adjustable
};

// Main
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeepStreamTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



