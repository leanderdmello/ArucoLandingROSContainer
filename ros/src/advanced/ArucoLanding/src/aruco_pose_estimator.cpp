#include "aruco_pose_estimator.hpp"
#include "common/logging.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <thread>

namespace {
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr g_pub;
  rclcpp::TimerBase::SharedPtr g_timer;
  cv::VideoCapture g_cap;

  std::string g_camera_device = "/dev/video0";
  int g_camera_width  = 1280;
  int g_camera_height = 720;
  double g_camera_fps = 20.0;

  std::string g_ned_frame = "NED_odom";
  double g_marker_yaw_deg = 0.0;

  double g_cam_tx = 0.0, g_cam_ty = 0.0, g_cam_tz = 0.0;
  std::string g_cam_mount = "down";
  double g_cam_yaw_deg = 0.0;

  constexpr int    kInnerId    = 24;
  constexpr int    kOuterId    = 122;
  constexpr double kInnerSizeM = 0.022;
  constexpr double kOuterSizeM = 0.154;

  int g_median_window = 7;
  double g_ema_alpha = 0.5;
}

namespace aruco_landing {

ArucoPoseEstimator::ArucoPoseEstimator(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  camera_matrix = (cv::Mat1d(3, 3) <<
      1336.135004356143, 0.0, 956.1451052158596,
      0.0, 1333.886438722237, 540.7159410836905,
      0.0, 0.0, 1.0);
  dist_coeffs = (cv::Mat1d(1, 5) <<
      -0.3478036670023198, 0.1341258850882104,
       0.0001174232535739054, 0.00005508984106933928,
      -0.02215131320202772);

  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  detector_params_ = cv::aruco::DetectorParameters::create();

  g_camera_device  = node_->declare_parameter<std::string>("camera_device", g_camera_device);
  g_camera_width   = node_->declare_parameter<int>("camera_width",  g_camera_width);
  g_camera_height  = node_->declare_parameter<int>("camera_height", g_camera_height);
  g_camera_fps     = node_->declare_parameter<double>("camera_fps", g_camera_fps);

  g_ned_frame      = node_->declare_parameter<std::string>("ned_frame_id", g_ned_frame);
  g_marker_yaw_deg = node_->declare_parameter<double>("marker_yaw_ned_deg", 0.0);

  g_cam_tx         = node_->declare_parameter<double>("cam_tx_m", 0.0);
  g_cam_ty         = node_->declare_parameter<double>("cam_ty_m", 0.0);
  g_cam_tz         = node_->declare_parameter<double>("cam_tz_m", 0.0);
  g_cam_mount      = node_->declare_parameter<std::string>("camera_mount", "down");
  g_cam_yaw_deg    = node_->declare_parameter<double>("cam_yaw_deg", 0.0);

  g_median_window  = node_->declare_parameter<int>("filter_median_window", 7);
  g_ema_alpha      = node_->declare_parameter<double>("filter_ema_alpha", 0.5);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  g_pub = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/aruco_landing/estimated_pose", qos);

  auto warm_ok = [&](cv::VideoCapture& cap)->bool{
    cv::Mat f; bool ok = false;
    for (int i=0;i<10;i++) { ok = cap.read(f); if (ok && !f.empty()) break; std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
    return ok && !f.empty();
  };
  auto log_open = [&](const std::string& dev, const char* apistr){
    double rw=g_cap.get(cv::CAP_PROP_FRAME_WIDTH), rh=g_cap.get(cv::CAP_PROP_FRAME_HEIGHT), rfps=g_cap.get(cv::CAP_PROP_FPS);
    int fcc=(int)g_cap.get(cv::CAP_PROP_FOURCC);
    char fourcc[5] = {(char)(fcc&255),(char)((fcc>>8)&255),(char)((fcc>>16)&255),(char)((fcc>>24)&255),0};
    RCLCPP_INFO(node_->get_logger(), "Camera %s opened API=%s FOURCC=%s %.0fx%.0f@%.0f", dev.c_str(), apistr, fourcc, rw, rh, rfps);
  };
  auto try_mode_v4l2 = [&](const std::string& dev, int fourcc, int w, int h, double fps)->bool{
    cv::VideoCapture cap(dev, cv::CAP_V4L2);
    if(!cap.isOpened()) return false;
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap.set(cv::CAP_PROP_FOURCC, fourcc);
    cap.set(cv::CAP_PROP_FPS,          fps);
    if(!warm_ok(cap)){ cap.release(); return false; }
    g_cap = std::move(cap); log_open(dev, "V4L2"); return true;
  };
  auto try_raw_v4l2 = [&](const std::string& dev)->bool{
    cv::VideoCapture cap(dev, cv::CAP_V4L2);
    if(!cap.isOpened()) return false;
    if(!warm_ok(cap)){ cap.release(); return false; }
    g_cap = std::move(cap); log_open(dev, "V4L2"); return true;
  };
  auto try_mode_any = [&](const std::string& dev, int fourcc, int w, int h, double fps)->bool{
    cv::VideoCapture cap(dev, cv::CAP_ANY);
    if(!cap.isOpened()) return false;
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap.set(cv::CAP_PROP_FOURCC, fourcc);
    cap.set(cv::CAP_PROP_FPS,          fps);
    if(!warm_ok(cap)){ cap.release(); return false; }
    g_cap = std::move(cap); log_open(dev, "ANY"); return true;
  };
  auto try_raw_any = [&](const std::string& dev)->bool{
    cv::VideoCapture cap(dev, cv::CAP_ANY);
    if(!cap.isOpened()) return false;
    if(!warm_ok(cap)){ cap.release(); return false; }
    g_cap = std::move(cap); log_open(dev, "ANY"); return true;
  };
  auto open_sequence = [&](const std::string& dev)->bool{
    const int MJPG = cv::VideoWriter::fourcc('M','J','P','G');
    const int YUYV = cv::VideoWriter::fourcc('Y','U','Y','V');
    if (try_raw_v4l2(dev)) return true;
    if (try_mode_v4l2(dev, MJPG, 1280, 720, 30)) return true;
    if (try_mode_v4l2(dev, YUYV, 1280, 720, 30)) return true;
    if (try_mode_v4l2(dev, MJPG,  640, 480, 30)) return true;
    if (try_mode_v4l2(dev, YUYV,  640, 480, 30)) return true;
    if (try_mode_v4l2(dev, MJPG, 1280, 720, 60)) return true;
    if (try_mode_v4l2(dev, YUYV, 1280, 720, 60)) return true;
    if (try_raw_any(dev)) return true;
    if (try_mode_any(dev, MJPG, 1280, 720, 30)) return true;
    if (try_mode_any(dev, YUYV, 1280, 720, 30)) return true;
    if (try_mode_any(dev, MJPG,  640, 480, 30)) return true;
    if (try_mode_any(dev, YUYV,  640, 480, 30)) return true;
    return false;
  };

  bool opened = open_sequence(g_camera_device);
  if (!opened) opened = open_sequence("/dev/video0") || open_sequence("/dev/video1");
  if (!opened) RCLCPP_ERROR(node_->get_logger(), "No camera could be opened. Pose publishing will be inactive.");

  auto period=std::chrono::milliseconds(static_cast<int>(1000.0/std::max(1.0,g_camera_fps)));
  g_timer=node_->create_wall_timer(period,[this](){
    if(!g_cap.isOpened()) return;
    cv::Mat frame; if(!g_cap.read(frame)||frame.empty()) return;
    geometry_msgs::msg::Pose pose;
    if(this->estimate(frame, pose)) {
      geometry_msgs::msg::PoseWithCovarianceStamped out;
      out.header.stamp=node_->now();
      out.header.frame_id=g_ned_frame;
      out.pose.pose=pose;
      std::fill(std::begin(out.pose.covariance), std::end(out.pose.covariance), 0.0);
      g_pub->publish(out);
    }
  });
}

ArucoPoseEstimator::~ArucoPoseEstimator(){
  if(g_timer){g_timer->cancel(); g_timer.reset();}
  if(g_pub) g_pub.reset();
  if(g_cap.isOpened()) g_cap.release();
}

static inline cv::Mat Rz_deg(double deg){
  double th=deg*M_PI/180.0, c=std::cos(th), s=std::sin(th);
  return (cv::Mat1d(3,3) << c,-s,0, s,c,0, 0,0,1);
}

bool ArucoPoseEstimator::estimate(const cv::Mat& image, geometry_msgs::msg::Pose& pose_out)
{
  if(image.empty()) return false;

  std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners,rejected;
  cv::aruco::detectMarkers(image,dictionary_,corners,ids,detector_params_,rejected);
  if(ids.empty()) return false;

  int chosen=-1; double size=0.0;
  for(int id:ids){ if(id==kInnerId){ chosen=kInnerId; size=kInnerSizeM; break; } if(id==kOuterId){ chosen=kOuterId; size=kOuterSizeM; } }
  if(chosen==-1){ auto it=std::find(ids.begin(),ids.end(),kOuterId); if(it!=ids.end()){ chosen=kOuterId; size=kOuterSizeM; } }
  if(chosen==-1) return false;

  size_t idx=0; for(size_t i=0;i<ids.size();++i){ if(ids[i]==chosen){ idx=i; break; } }
  std::vector<std::vector<cv::Point2f>> sel{corners[idx]};
  std::vector<cv::Vec3d> rvecs,tvecs;
  cv::aruco::estimatePoseSingleMarkers(sel,size,camera_matrix,dist_coeffs,rvecs,tvecs);
  if(rvecs.empty()||tvecs.empty()) return false;

  cv::Mat R_cm; cv::Rodrigues(rvecs[0],R_cm);
  cv::Vec3d t_cm=tvecs[0];
  cv::Mat R_mc=R_cm.t();
  cv::Mat t_cm_mat=(cv::Mat1d(3,1)<<t_cm[0],t_cm[1],t_cm[2]);
  cv::Mat t_mc_mat=-R_mc*t_cm_mat;

  cv::Mat R_bc_base;
  if (g_cam_mount == "down") {
    R_bc_base = (cv::Mat1d(3,3) << 0,1,0, 1,0,0, 0,0,1);
  } else {
    R_bc_base = (cv::Mat1d(3,3) << 0,0,1, 1,0,0, 0,1,0);
  }
  cv::Mat R_bc = Rz_deg(g_cam_yaw_deg) * R_bc_base;
  cv::Mat t_bc = (cv::Mat1d(3,1) << g_cam_tx,g_cam_ty,g_cam_tz);
  cv::Mat R_cb = R_bc.t();
  cv::Mat t_cb = -R_cb * t_bc;

  cv::Mat R_mb=R_mc*R_cb;
  cv::Mat t_mb=R_mc*t_cb + t_mc_mat;

  double th=g_marker_yaw_deg*M_PI/180.0, c=std::cos(th), s=std::sin(th);
  cv::Mat R_nm=(cv::Mat1d(3,3) <<  c,-s,0,  s, c,0,  0,0,-1);

  cv::Mat R_nb=R_nm*R_mb;
  cv::Mat t_nb=R_nm*t_mb;

  tf2::Matrix3x3 m(
    R_nb.at<double>(0,0),R_nb.at<double>(0,1),R_nb.at<double>(0,2),
    R_nb.at<double>(1,0),R_nb.at<double>(1,1),R_nb.at<double>(1,2),
    R_nb.at<double>(2,0),R_nb.at<double>(2,1),R_nb.at<double>(2,2));
  tf2::Quaternion q; m.getRotation(q);

  geometry_msgs::msg::Pose p;
  p.position.x=t_nb.at<double>(0,0);
  p.position.y=t_nb.at<double>(1,0);
  p.position.z=t_nb.at<double>(2,0);
  p.orientation.x=q.x();
  p.orientation.y=q.y();
  p.orientation.z=q.z();
  p.orientation.w=q.w();

  const rclcpp::Time now=node_->now();
  buf.emplace_back(now,p);
  const rclcpp::Duration win=rclcpp::Duration::from_seconds(2.0);
  while(!buf.empty()&&(now-buf.front().first)>win) buf.pop_front();

  std::vector<double> X,Y,Z;
  int N = std::min((int)buf.size(), std::max(3, g_median_window));
  X.reserve(N); Y.reserve(N); Z.reserve(N);
  for(int i=std::max(0,(int)buf.size()-N); i<(int)buf.size(); ++i){
    X.push_back(buf[i].second.position.x);
    Y.push_back(buf[i].second.position.y);
    Z.push_back(buf[i].second.position.z);
  }
  auto med = [](std::vector<double>& v){ std::nth_element(v.begin(), v.begin()+v.size()/2, v.end()); return v[v.size()/2]; };

  double mx = med(X), my = med(Y), mz = med(Z);

  static bool have_prev=false;
  static double fx=0, fy=0, fz=0;
  if(!have_prev){ fx=mx; fy=my; fz=mz; have_prev=true; }
  else {
    fx = g_ema_alpha*mx + (1.0-g_ema_alpha)*fx;
    fy = g_ema_alpha*my + (1.0-g_ema_alpha)*fy;
    fz = g_ema_alpha*mz + (1.0-g_ema_alpha)*fz;
  }

  geometry_msgs::msg::Pose out=p;
  out.position.x=fx; out.position.y=fy; out.position.z=fz;
  pose_out=out;
  return true;
}

}
