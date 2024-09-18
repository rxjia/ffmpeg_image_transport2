// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ffmpeg_image_transport/ffmpeg_subscriber.hpp"

#include <functional>
#include <unordered_map>

#include "ffmpeg_image_transport/safe_param.hpp"

using namespace std::placeholders;

namespace ffmpeg_image_transport
{
static const std::string ns = "ffmpeg_image_transport.";
static const std::string nsm = "ffmpeg_image_transport.map.";

FFMPEGSubscriber::FFMPEGSubscriber() : logger_(rclcpp::get_logger("FFMPEGSubscriber")) {}

FFMPEGSubscriber::~FFMPEGSubscriber() { decoder_.reset(); }

void FFMPEGSubscriber::frameReady(const ImageConstPtr & img, bool) const { (*userCallback_)(img); }

#ifdef IMAGE_TRANSPORT_API_V1
void FFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  initialize(node);
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);
}
#else
void FFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions opt)
{
  initialize(node);
  custom_qos = rmw_qos_profile_sensor_data;
  RCLCPP_DEBUG_STREAM(logger_, "SUBSCRIBER: decoder type: " << decoderType_.c_str());
#ifdef IMAGE_TRANSPORT_API_V2
  (void)opt;  // to suppress compiler warning
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);
#else
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos, opt);
#endif
}
#endif

void FFMPEGSubscriber::initialize(rclcpp::Node * node)
{
  node_ = node;

  // create parameters from default map
  for (const auto & kv : FFMPEGDecoder::getDefaultEncoderToDecoderMap()) {
    const std::string key = std::string(nsm) + kv.first;
    if (!node_->has_parameter(key)) {
      (void)node_->declare_parameter<std::string>(key, kv.second);
    }
  }
  const bool mp = get_safe_param<bool>(node_, ns + "measure_performance", false);
  decoder_.setMeasurePerformance(mp);
}

void FFMPEGSubscriber::internalCallback(const FFMPEGPacketConstPtr & msg, const Callback & user_cb)
{
  if (decoder_.isInitialized() && decoder_.needReset(msg)) decoder_.reset();

  if (!decoder_.isInitialized()) {
    if (msg->flags == 0) {
      return;  // wait for key frame!
    }

    if (msg->encoding.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "no encoding provided!");
      return;
    }

    const std::string decoder_name = get_safe_param<std::string>(node_, nsm + msg->encoding, "");
    if (decoder_name.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "no valid decoder found for encoding: " << msg->encoding);
      return;
    }
    if (decoderType_ != decoder_name) {
      decoder_.reset();
      decoderType_ = decoder_name;
      RCLCPP_INFO_STREAM(logger_, "using decoder " << decoderType_);
    }

    decoderHwAcc_ = get_safe_param<std::string>(node_, ns + "hwacc", "cuda");

    if (!decoder_.initialize(
          msg,
          [&user_cb](const ImageConstPtr & img, bool isKeyFrame) {
            (void)isKeyFrame;
            user_cb(img);
          },
          decoderType_, decoderHwAcc_)) {
      RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder!");
      return;
    }
  }
  decoder_.decodePacket(msg);
}
}  // namespace ffmpeg_image_transport
