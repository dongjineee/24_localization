#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <random>
#include <boost/filesystem.hpp>

#include <math.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry> 
#include <tf/tf.h> 
#include <tf/transform_listener.h> 
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <random>

namespace fs = boost::filesystem;
// tf::TransformListener* tf_listener;

class PointCloudConverter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;

    double start_time, end_time, time,sum;

    #define ground 0
    #define y_wall 1
    #define x_wall 2
    #define not_plane 33

    #define max_time 200

    int cnt = 0;

    int runnig_cnt = 0;
    double runnig_sum = 0;
    double runnig_mean = 0;

    double stop_flag = 0;

    std::vector<uint16_t> parameter_vector;

    geometry_msgs::TransformStamped transformStamped;
    // tf2_ros::TransformBroadcaster br;

    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    // tf::TransformListener tf_listener_;
    ros::Publisher t_1;

public:
  PointCloudConverter(){
    // Subscriber와 Publisher 설정
    //  pc_sub_ = nh_.subscribe("/velodyne_points/points", 1, &PointCloudConverter::pointCloud2Callback, this);
    // pc_sub_ = nh_.subscribe("/platform/velodyne_points", 1, &PointCloudConverter::pointCloud2Callback, this);
    pc_sub_ = nh_.subscribe("/os1_cloud_node1/points", 1, &PointCloudConverter::pointCloud2Callback, this);

    t_1 = nh_.advertise<sensor_msgs::PointCloud2>("/t_1", 1);
    parameter_vector = Iteration_initial(max_time);
  }

  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_cloud, *input_cloud_pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    ransac(*input_cloud_pcl);

  }

  void ransac(const pcl::PointCloud<pcl::PointXYZ>& input_cloud)
  {
    // 필터링

    start_time = ros::Time::now().toSec();

    sensor_msgs::PointCloud2 test_cloud;
    pcl::toROSMsg(input_cloud, test_cloud);

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud ;
    filtered_cloud = filter_cloud(input_cloud, 0.1);

    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> extract_vector;

    extract_vector = seg_plane(filtered_cloud.makeShared(),parameter_vector, 0.01, 200);
    // extract_vector = seg_plane(input_cloud.makeShared(),parameter_vector, 0.01, 200);

    pcl::PointCloud<pcl::PointNormal>::Ptr accumulated_cloud ;
    accumulated_cloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    for(int i = 0; i < extract_vector.size(); i ++)
    {
        *accumulated_cloud += *extract_vector[i];
    }

    // topic_publish(*accumulated_cloud, t_1);

    sensor_msgs::PointCloud2 pub_cloud;

    pcl::toROSMsg(*accumulated_cloud, pub_cloud); 
    pub_cloud.header.frame_id = input_cloud.header.frame_id; 
    pub_cloud.header.stamp = ros::Time::now(); 

    // t_1.publish(pub_cloud); 
    // pcl::PointCloud<pcl::PointNormal>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointNormal>());
    // for (auto& cloud : extract_vector) {
    //     *accumulated_cloud += *cloud; // 클라우드 통합
    // }
    // topic_publish(accumulated_cloud, t_1);



    end_time = ros::Time::now().toSec();

    time = end_time-start_time;
    sum += time;

    runnig_cnt ++;
    runnig_sum += time;
    runnig_mean = runnig_sum/runnig_cnt;

    //ROS_INFO("%f %f %d\n",runnig_mean,runnig_sum, runnig_cnt);
    ROS_INFO("all plane num : %d", extract_vector.size());
    ROS_INFO("%f sec\n",time);
    start_time = ros::Time::now().toSec();
    // Convert the first PointCloud to ROS message and publish
    sensor_msgs::PointCloud2 test_1_cloud;

    // pcl::toROSMsg(*(segmentResult.second[0]), test_1_cloud);
    // test_1_cloud.header.frame_id = frame_id;
    // test_1_cloud.header.stamp = ros::Time::now();

    // pub_2.publish(test_1_cloud);
  }

  pcl::PointCloud<pcl::PointXYZ> filter_cloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, float filterRes)
  {
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud.makeShared());
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(filtered_cloud);
    return filtered_cloud;
  }

uint8_t plane_jurdgment( Eigen::Vector3f normal_vector)
{
    normal_vector.normalize(); // 법선 벡터 정규화

    Eigen::Vector3f z_axis_pos(0, 0, 1);
    Eigen::Vector3f y_axis_pos(0, 1, 0);
    Eigen::Vector3f x_axis_pos(1, 0, 0);
    Eigen::Vector3f z_axis_nag(0, 0, -1);
    Eigen::Vector3f y_axis_nag(0, -1, 0);
    Eigen::Vector3f x_axis_nag(-1, 0, 0);

    std::vector<Eigen::Vector3f> axis = {z_axis_pos, y_axis_pos, x_axis_pos, z_axis_nag ,y_axis_nag,x_axis_nag};

    const float angleThreshold = 5.0 * M_PI / 180.0; // 5도를 라디안으로 변환

    for (int i = 0; i < 6; i++) {

        float angle = acos(normal_vector.dot(axis[i])); // 두 벡터 간의 각도 계산, 내적 계산

        if (abs(angle) <= angleThreshold) {
            return i; // 조건을 만족하는 축에 해당하는 포지션 반환

        }
    }
    return 33; // 어떤 축과도 매치되지 않는 경우
}

int ransac_number_calculate(double in_out_ratio)
{
    int T = std::log(1 - 0.99) / std::log(1 - std::pow(1 - in_out_ratio, 3));

    if(T > max_time) return max_time;

    return T;
} // ransac parmeter calculate

std::vector<uint16_t> Iteration_initial(uint16_t time)
{
    std::vector<uint16_t> plane_time;

    for(int i = 0; i < 5; i++) plane_time.push_back(time);

    return plane_time;
} // initial ransac_number parmeter

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>  seg_plane(
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
    std::vector<uint16_t> maxIterations,
    float distanceThreshold,
    uint8_t planes_num)
{

  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> extracted_cloud_vec;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  transformed_cloud->header = input_cloud->header;
  for (size_t i = 0; i < input_cloud->points.size(); ++i) {
    transformed_cloud->points.push_back(input_cloud->points[i]);
  }

  int i = 0;

  while (transformed_cloud->points.size() > 150) {
    try {

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      // Optional
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.2);
      seg.setInputCloud(transformed_cloud);
      seg.setNumberOfThreads(8);
      seg.setMaxIterations(500);
      // seg.setInputNormals(normal_cloud);
      // int model_type = seg.getModelType();
      seg.segment(*inliers, *coefficients);
      /* check if indicies are not empty for no crash */
      if (inliers->indices.empty()) {
        std::cout << "Breaking as no model found" << std::endl;
        break;
      }
      /* filtering out noisy ground plane measurements */
      if(fabs(coefficients->values[2]) > 0.8 ||
          inliers->indices.size() < 150) {
        extract.setInputCloud(transformed_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*transformed_cloud);
        continue;
      }

      if(coefficients->values[3] < 0){coefficients->values[0] *= -1; coefficients->values[1] *= -1; coefficients->values[2] *= -1; std::cout << "good" << std::endl;}

      Eigen::Vector4d normal(coefficients->values[0],
                             coefficients->values[1],
                             coefficients->values[2],
                             coefficients->values[3]);
      Eigen::Vector3d closest_point = normal.head(3) * normal(3);
      Eigen::Vector4d plane;

      plane.head(3) = closest_point / closest_point.norm();
      plane(3) = closest_point.norm();
      
      std::cout << "Model coefficients before " << std::to_string(i) << ": " <<
      coefficients->values[0] << " " << coefficients->values[1] << " " <<
      coefficients->values[2] << " " << coefficients->values[3] << std::endl;

      pcl::PointCloud<pcl::PointNormal>::Ptr extracted_cloud(
          new pcl::PointCloud<pcl::PointNormal>);
      for (const auto& idx : inliers->indices) {
        pcl::PointNormal tmp_cloud;
        tmp_cloud.x = transformed_cloud->points[idx].x;
        tmp_cloud.y = transformed_cloud->points[idx].y;
        tmp_cloud.z = transformed_cloud->points[idx].z;
        tmp_cloud.normal_x = plane(0);
        tmp_cloud.normal_y = plane(1);
        tmp_cloud.normal_z = plane(2);
        tmp_cloud.curvature = plane(3);

        extracted_cloud->points.push_back(tmp_cloud);
      }
      pcl::PointCloud<pcl::PointNormal>::Ptr extracted_cloud_filtered;
      extracted_cloud_filtered = extracted_cloud;

      std::random_device rd;  // 난수 생성을 위한 장치
      std::mt19937 gen(rd()); // Mersenne Twister 난수 생성기
      std::uniform_int_distribution<> dis(0, 255); // 0부터 255까지의 균일 분포

      uint8_t r = dis(gen);
      uint8_t g = dis(gen);
      uint8_t b = dis(gen);
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

      for (int pc = 0; pc < extracted_cloud_filtered->points.size(); ++pc) {
          pcl::PointXYZRGBNormal point;
          // extracted_cloud_filtered의 포인트 데이터 복사
          point.x = extracted_cloud_filtered->points[pc].x;
          point.y = extracted_cloud_filtered->points[pc].y;
          point.z = extracted_cloud_filtered->points[pc].z;
          point.normal_x = extracted_cloud_filtered->points[pc].normal_x;
          point.normal_y = extracted_cloud_filtered->points[pc].normal_y;
          point.normal_z = extracted_cloud_filtered->points[pc].normal_z;
          // RGB 값을 추가
          float rgb_float = *reinterpret_cast<float*>(&rgb);
          point.rgb = rgb_float;
          segmented_cloud->points.push_back(point);
      }

      extracted_cloud_vec.push_back(extracted_cloud_filtered);

      extract.setInputCloud(transformed_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*transformed_cloud);
      i++;
    } catch (const std::exception& e) {
      std::cout << "No ransac model found" << std::endl;
      break;
    }
  }
  sensor_msgs::PointCloud2 segmented_cloud_msg;

  pcl::toROSMsg(*segmented_cloud, segmented_cloud_msg);
  segmented_cloud_msg.header.frame_id = input_cloud->header.frame_id;

  segmented_cloud_msg.header.stamp = ros::Time::now();
  t_1.publish(segmented_cloud_msg);

  return extracted_cloud_vec;
}

    void topic_publish(pcl::PointCloud<pcl::PointNormal>const input_cloud, ros::Publisher input_publish)
    {
        sensor_msgs::PointCloud2 pub_cloud;

        pcl::toROSMsg(input_cloud, pub_cloud); 
        pub_cloud.header.frame_id = "test"; 
        pub_cloud.header.stamp = ros::Time::now(); 
        input_publish.publish(pub_cloud); 

    }


};




int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_converter_node");

  PointCloudConverter converter;

  ros::spin();

  return 0;
}